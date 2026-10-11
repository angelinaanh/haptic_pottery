//==============================================================================
//  Haptic thread: the connection to the haptic device.
//  See HapticDeviceController.h.
//==============================================================================
#include "HapticDeviceController.h"

#include <algorithm>
#include <iostream>
#include <thread>

using namespace chai3d;
using namespace std;

namespace vp {

//------------------------------------------------------------------------------

namespace {

const double kPeriod = 1.0 / 1000.0;    // servo period [s]

//! Busy-waits until \p a_time. Windows' sleep granularity (~15.6 ms) is far
//! coarser than the 1 ms period, so the wait spins and yields instead.
void waitUntil(double a_time)
{
    while (cPrecisionClock::getCPUTimeSeconds() < a_time)
    {
        std::this_thread::yield();
    }
}

} // namespace

//==============================================================================
//  The mouse as a haptic device.
//
//  cHapticDeviceHandler returns an inert device when no hardware is present.
//  This class presents the mouse through cGenericHapticDevice instead, so the
//  haptic thread has one code path with or without hardware. The graphics
//  thread injects the target position (in device coordinates); the device
//  moves towards it at a bounded speed - real hardware cannot teleport, and a
//  teleported HIP would land deep inside the clay - and paces itself to 1 kHz
//  the way a real device's encoder read does.
//==============================================================================

class HapticDeviceController::SimulatedMouseDevice : public cGenericHapticDevice
{
public:

    SimulatedMouseDevice() : cGenericHapticDevice(0)
    {
        m_switches   = 0;
        m_nextSample = 0.0;
        m_target.zero();
        m_position.zero();

        m_specifications.m_model              = C_HAPTIC_DEVICE_CUSTOM;
        m_specifications.m_manufacturerName   = "Virtual Pottery";
        m_specifications.m_modelName          = "mouse (simulated)";
        m_specifications.m_maxLinearForce     = 5.0;
        m_specifications.m_maxLinearStiffness = 1500.0;
        m_specifications.m_maxLinearDamping   = 10.0;
        m_specifications.m_workspaceRadius    = 0.15;
        m_specifications.m_sensedPosition     = true;
        m_specifications.m_actuatedPosition   = false;

        m_deviceAvailable = true;
    }

    virtual bool open()  { m_deviceReady = true;  return C_SUCCESS; }
    virtual bool close() { m_deviceReady = false; return C_SUCCESS; }
    virtual bool calibrate(bool a_force = false) { return C_SUCCESS; }

    virtual bool getPosition(cVector3d& a_position)
    {
        // Pace to 1 kHz.
        double now = cPrecisionClock::getCPUTimeSeconds();
        if (m_nextSample <= 0.0) { m_nextSample = now + kPeriod; }
        waitUntil(m_nextSample);
        m_nextSample += kPeriod;
        now = cPrecisionClock::getCPUTimeSeconds();
        if (m_nextSample < now) { m_nextSample = now + kPeriod; }

        // Chase the mouse at no more than 1.2 m/s.
        m_lock.acquire();
        cVector3d target = m_target;
        m_lock.release();

        const double maxStep = 1.2 * kPeriod;
        cVector3d delta = target - m_position;
        double dist = delta.length();
        m_position = (dist > maxStep) ? m_position + (maxStep / dist) * delta : target;

        a_position = m_position;
        estimateLinearVelocity(a_position);
        return C_SUCCESS;
    }

    virtual bool getRotation(cMatrix3d& a_rotation)
    {
        a_rotation.identity();
        return C_SUCCESS;
    }

    virtual bool getUserSwitches(unsigned int& a_switches)
    {
        m_lock.acquire();
        a_switches = m_switches;
        m_lock.release();
        return C_SUCCESS;
    }

    virtual bool setForceAndTorqueAndGripperForce(const cVector3d&, const cVector3d&, double)
    {
        return C_SUCCESS;   // nothing to drive; the force is drawn on screen
    }

    void setTarget(const cVector3d& a_device)
    {
        m_lock.acquire();
        m_target = a_device;
        m_lock.release();
    }

    void setButton(bool a_pressed)
    {
        m_lock.acquire();
        m_switches = a_pressed ? 1u : 0u;
        m_lock.release();
    }

private:

    cMutex       m_lock;
    cVector3d    m_target;      // written by the graphics thread
    cVector3d    m_position;    // haptic thread only
    unsigned int m_switches;
    double       m_nextSample;
};

//==============================================================================

HapticDeviceController::HapticDeviceController()
{
    m_handler      = nullptr;
    m_mouse        = nullptr;
    m_simulated    = false;
    m_maxStiffness = 1000.0;
    m_maxForce     = kMaxForce;
    m_spinLatched  = false;
    m_prevButtons  = 0;
    m_armed         = false;
    m_disarmRequest = false;
    m_freeTime      = 0.0;
    m_nextTick     = 0.0;
}

HapticDeviceController::~HapticDeviceController()
{
    close();
    delete m_handler;
}

//------------------------------------------------------------------------------

bool HapticDeviceController::open()
{
    m_handler = new cHapticDeviceHandler();

    if (m_handler->getNumDevices() > 0)
    {
        m_handler->getDevice(m_device, 0);
        m_simulated = false;
    }
    else
    {
        auto mouse = std::make_shared<SimulatedMouseDevice>();
        m_mouse     = mouse.get();
        m_device    = mouse;
        m_simulated = true;
    }

    if (!m_device || !m_device->open()) { return false; }
    m_device->calibrate();

    cHapticDeviceInfo info = m_device->getSpecifications();
    m_name = info.m_modelName;

    // A world stiffness K is felt as K * scale at the handle.
    if (info.m_maxLinearStiffness > 0.0) { m_maxStiffness = info.m_maxLinearStiffness / kWorkspaceScale; }
    if (info.m_maxLinearForce > 0.0)     { m_maxForce = std::min(kMaxForce, info.m_maxLinearForce); }

    cout << "[device] " << (m_simulated ? "none detected - using " : "found: ") << m_name << endl;
    return true;
}

//------------------------------------------------------------------------------

void HapticDeviceController::close()
{
    if (m_device)
    {
        m_device->setForce(cVector3d(0.0, 0.0, 0.0));
        m_device->close();
        m_device.reset();
        m_mouse = nullptr;
    }
}

//------------------------------------------------------------------------------

HipState HapticDeviceController::read()
{
    cVector3d pos, vel;
    unsigned int buttons = 0;

    m_device->getPosition(pos);
    m_device->getLinearVelocity(vel);
    m_device->getUserSwitches(buttons);

    // Han 2007: hold the button to turn the wheel. The second button latches.
    if ((buttons & 2u) && !(m_prevButtons & 2u)) { toggleSpinLatch(); }
    m_prevButtons = buttons;

    HipState hip;
    hip.position = kWorkspaceScale * pos + kWorkspaceOffset;
    hip.velocity = kWorkspaceScale * vel;
    hip.spinning = m_spinLatched || ((buttons & 1u) != 0);
    return hip;
}

//------------------------------------------------------------------------------

cVector3d HapticDeviceController::sendForce(const cVector3d& a_force, bool a_inContact, double a_dt)
{
    if (m_disarmRequest.exchange(false))
    {
        m_armed    = false;
        m_freeTime = 0.0;
    }

    if (!m_armed)
    {
        m_freeTime = a_inContact ? 0.0 : (m_freeTime + a_dt);
        if (m_freeTime >= kArmDelay) { m_armed = true; }
    }

    cVector3d force = m_armed ? a_force : cVector3d(0.0, 0.0, 0.0);

    double magnitude = force.length();
    if (magnitude > m_maxForce) { force *= m_maxForce / magnitude; }

    m_device->setForce(force);
    return force;
}

//------------------------------------------------------------------------------

void HapticDeviceController::waitForNextTick()
{
    double now = cPrecisionClock::getCPUTimeSeconds();
    if (m_nextTick <= 0.0) { m_nextTick = now; }

    // The simulated device already paces inside getPosition().
    if (!m_simulated) { waitUntil(m_nextTick + kPeriod); }

    m_nextTick += kPeriod;
    now = cPrecisionClock::getCPUTimeSeconds();
    if (m_nextTick < now - kPeriod) { m_nextTick = now; }
}

//------------------------------------------------------------------------------

void HapticDeviceController::setMouseTarget(const cVector3d& a_world)
{
    if (m_mouse) { m_mouse->setTarget((a_world - kWorkspaceOffset) / kWorkspaceScale); }
}

void HapticDeviceController::setMouseButton(bool a_pressed)
{
    if (m_mouse) { m_mouse->setButton(a_pressed); }
}

} // namespace vp
