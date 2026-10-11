//==============================================================================
//  Haptic thread: the connection to the haptic device.
//
//  Reads the Haptic Interface Point (HIP) - position, velocity, buttons - maps
//  it from device to world coordinates, and sends the force back. Also owns
//  the device-side policies:
//
//    - pacing of the servo loop to 1 kHz;
//    - the wheel control of Han 2007: the wheel turns while the stylus button
//      is held; the second button (or SPACE) latches it on or off;
//    - safety: force output is armed only after the HIP has been clear of the
//      clay for a moment, and the force is clamped to the device's maximum.
//
//  When no device is connected, a simulated device driven by the mouse takes
//  its place behind the same cGenericHapticDevice interface, so the rest of
//  the haptic thread runs unchanged; the force is then drawn, not felt.
//==============================================================================
#ifndef VP_HAPTIC_DEVICE_CONTROLLER_H
#define VP_HAPTIC_DEVICE_CONTROLLER_H

#include "chai3d.h"

#include <atomic>
#include <string>

namespace vp {

//------------------------------------------------------------------------------
// Device -> world mapping: world = device * scale + offset.
// Scale 1 keeps the felt stiffness equal to the simulated one. The offset puts
// the device's home position beside the clay, not inside it.
//------------------------------------------------------------------------------

const double            kWorkspaceScale = 1.0;
const chai3d::cVector3d kWorkspaceOffset(0.0, 0.050, 0.0);

const double kMaxForce = 3.0;   //!< ceiling on the commanded force [N]
const double kArmDelay = 0.2;   //!< contact-free time before force is armed [s]

//------------------------------------------------------------------------------

//! The HIP as read in one haptic tick, world frame.
struct HipState
{
    chai3d::cVector3d position;     //!< [m]
    chai3d::cVector3d velocity;     //!< [m/s]
    bool              spinning;     //!< the user wants the wheel turning
};

//==============================================================================

class HapticDeviceController
{
public:

    HapticDeviceController();
    ~HapticDeviceController();

    //! Connects to the first device found, or to the simulated mouse device.
    bool open();
    void close();

    bool isSimulated() const { return m_simulated; }
    const std::string& deviceName() const { return m_name; }

    //! Largest stiffness the device renders stably, in world units [N/m].
    double maxStiffness() const { return m_maxStiffness; }

    //--------------------------------------------------------------------------
    // Haptic thread
    //--------------------------------------------------------------------------

    //! Reads the HIP. The simulated device blocks here until its next sample.
    HipState read();

    //! Sends a world-frame force, applying the arming and the clamp, and
    //! returns the force actually sent. \p a_inContact tells whether the HIP
    //! touches anything this tick.
    chai3d::cVector3d sendForce(const chai3d::cVector3d& a_force, bool a_inContact, double a_dt);

    //! Forces are output only once armed (see kArmDelay).
    bool isArmed() const { return m_armed; }

    //! Withholds force until the HIP is clear again, e.g. after the clay
    //! changed shape under it. Safe from any thread; takes effect at the next
    //! sendForce().
    void disarm() { m_disarmRequest = true; }

    //! Waits for the next 1 ms slot.
    void waitForNextTick();

    //--------------------------------------------------------------------------
    // Graphics thread
    //--------------------------------------------------------------------------

    void toggleSpinLatch() { m_spinLatched = !m_spinLatched; }
    bool isSpinLatched() const { return m_spinLatched; }

    //! Mouse fallback: where the HIP should go (world frame), and the button
    //! standing in for the stylus button.
    void setMouseTarget(const chai3d::cVector3d& a_world);
    void setMouseButton(bool a_pressed);

private:

    class SimulatedMouseDevice;

    chai3d::cHapticDeviceHandler*   m_handler;
    chai3d::cGenericHapticDevicePtr m_device;
    SimulatedMouseDevice*           m_mouse;        //!< owned through m_device

    bool        m_simulated;
    std::string m_name;
    double      m_maxStiffness;
    double      m_maxForce;

    std::atomic<bool> m_spinLatched;
    unsigned int      m_prevButtons;

    std::atomic<bool> m_armed;
    std::atomic<bool> m_disarmRequest;
    double            m_freeTime;

    double m_nextTick;
};

} // namespace vp

#endif // VP_HAPTIC_DEVICE_CONTROLLER_H
