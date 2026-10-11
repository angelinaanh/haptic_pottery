//==============================================================================
//  Virtual Pottery - system integration
//
//  A haptic pottery modelling system: the user deforms a body of virtual clay
//  turning on a potter's wheel with a haptic tool, sees it change on screen
//  and feels the clay push back. The clay is a set of circular sector
//  elements built on the cylindrical symmetry of pottery, with local response
//  force propagation between neighbouring elements.
//
//  Two interaction methods can be switched at run time (MethodConfig.h):
//  the original one (baseline) and the improved one, Method 2.
//
//  Threads
//
//    Servo thread (1 kHz)       reads the HIP, forms the force, sends it.
//                               Baseline: runs the full physics step itself.
//                               Method 2: evaluates the local model
//                               F = F0 - K (x - x0), cross-fading from one
//                               model to the next, handed over by ...
//    Physics thread (250 Hz)    ... the full model: commands, wheel, collision
//                               detection, deformation, propagation, force
//                               and local stiffness. Idle for the baseline.
//    Graphic thread (screen)    GPU texture synchronisation, drawing, input.
//
//  Whichever thread runs a physics step holds physicsLock, so the
//  VirtualPottery is only ever touched by one thread at a time. The servo
//  thread never waits for a lock: when one is busy it carries on with the
//  data it has.
//
//  Usage: VirtualPottery [--method 0|2] [--open] [--bench]
//         --method 0 starts with the baseline, 2 with Method 2 (default).
//         --bench runs the scripted comparison of both methods, writes the
//         results to Results/ and exits.
//==============================================================================

#include "chai3d.h"
#include <GLFW/glfw3.h>

// Core data structures
#include "MethodConfig.h"
#include "VirtualPottery.h"
// Haptic thread
#include "HapticDeviceController.h"
#include "CollisionDetector.h"
#include "CollisionResponder.h"
#include "LocalForceModel.h"
// Graphic thread
#include "GraphicSynchronizer.h"
#include "VisualizePottery.h"
// Evaluation
#include "Benchmark.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

using namespace chai3d;
using namespace vp;
using namespace std;

//==============================================================================
// SYSTEM COMPONENTS
//==============================================================================

// Core data. Touched only under physicsLock.
VirtualPottery* pottery = nullptr;
cMutex          physicsLock;

// Physics step.
CollisionDetector  detector;
CollisionResponder responder;
ContactSet         contacts;
int                methodResolution[kNumMethods];
int                publishCounter = 0;
uint64_t           physicsSequence = 0;
cFrequencyCounter  physicsRate;

// Haptic device and the servo <-> physics hand-over.
HapticDeviceController device;
LocalModelExchange     modelExchange;
cMutex                 hipLock;
HipState               sharedHip;
bool                   sharedArmed = false;

// Settings the servo thread reads, written when commands are applied.
std::atomic<int>    activeMethod(METHOD_2);
std::atomic<double> servoDamping(1.0);
std::atomic<double> servoLowPassHz(0.0);
std::atomic<double> servoHipRadius(0.006);

// Graphic thread.
GraphicSynchronizer synchronizer;
VisualizePottery    visual;

// Threads.
cThread*      servoThread   = nullptr;
cThread*      physicsThread = nullptr;
volatile bool running       = false;
volatile bool servoDone     = true;
volatile bool physicsDone   = true;

// Evaluation.
Benchmark*        benchmark = nullptr;
bool              runBenchmark = false;
std::atomic<bool> quitRequested(false);
cMutex            benchTextLock;
string            benchText;

//------------------------------------------------------------------------------
// Commands from the graphic thread (and the benchmark) to the physics step
//------------------------------------------------------------------------------

struct Commands
{
    bool              paramsChanged = false;
    PotteryParameters params;
    int               method     = -1;     // -1 unchanged
    int               resolution = -1;     // preset index for the active method
    bool              reset      = false;
    int               hollow     = -1;     // -1 unchanged, 0 close, 1 open
};

cMutex   commandLock;
Commands pendingCommands;

// The graphic thread's view of the settings.
PotteryParameters uiParams;
PotteryMethod     uiMethod = METHOD_2;
int               uiResolution[kNumMethods];
bool              uiHollow = false;

//------------------------------------------------------------------------------
// Window and input
//------------------------------------------------------------------------------

GLFWwindow* window = nullptr;
int  windowW = 0, windowH = 0;
int  framebufferW = 0, framebufferH = 0;
bool fullscreen = false;

double mouseX = 0.0, mouseY = 0.0;
bool   mouseSeen  = false;      // until the cursor moves over the window
double mouseDepth = 0.0;        // x of the plane the mouse moves the HIP in [m]

//==============================================================================
// PHYSICS STEP (servo thread for the baseline, physics thread otherwise)
//==============================================================================

//! Applies pending commands. Runs at the start of a physics step, so the
//! pottery never changes in the middle of one.
void applyCommands()
{
    if (!commandLock.tryAcquire()) { return; }
    Commands c = pendingCommands;
    pendingCommands = Commands();
    commandLock.release();

    PotteryParameters& p = pottery->parameters();
    if (c.paramsChanged)
    {
        double kMax = device.maxStiffness();
        c.params.resistanceSpinning = std::min(c.params.resistanceSpinning, kMax);
        c.params.resistanceStopped  = std::min(c.params.resistanceStopped,  kMax);
        c.params.rigidStiffness     = std::min(c.params.rigidStiffness,     kMax);
        p = c.params;
    }

    bool reshaped = false;

    if (c.method >= 0)
    {
        activeMethod = c.method;
        p.lowPassHz  = methodConfig((PotteryMethod)c.method).lowPassHz;
        if (c.resolution < 0) { c.resolution = methodResolution[c.method]; }
        reshaped = true;            // the grid may change with the method
    }

    if (c.resolution >= 0)
    {
        methodResolution[activeMethod] = c.resolution;
        const int* r = kResolutionPresets[c.resolution];
        if ((r[0] != pottery->rows()) || (r[1] != pottery->cols())) { pottery->resize(r[0], r[1]); }
        reshaped = true;
    }

    if (c.hollow >= 0) { pottery->setHollow(c.hollow == 1); reshaped = true; }
    if (c.reset)       { pottery->reset();                  reshaped = true; }

    servoDamping   = p.damping;
    servoLowPassHz = p.lowPassHz;
    servoHipRadius = p.hipRadius;

    if (reshaped)
    {
        synchronizer.publishAll(*pottery);
        device.disarm();            // the clay may now be where the HIP is
    }
}

//------------------------------------------------------------------------------

//! Never hand the servo loop a spring stiffer than the device renders stably.
//! K is positive semi-definite, so its trace bounds its largest eigenvalue.
cMatrix3d boundStiffness(const cMatrix3d& a_K)
{
    double trace = a_K(0, 0) + a_K(1, 1) + a_K(2, 2);
    double kMax  = device.maxStiffness();
    if (trace <= kMax) { return a_K; }
    cMatrix3d K = a_K;
    K.mul(kMax / trace);
    return K;
}

//------------------------------------------------------------------------------

//! One step of the full model at the HIP position \p a_hip. Caller holds
//! physicsLock.
LocalForceModel physicsStep(const HipState& a_hip, bool a_armed, double a_dt)
{
    double t0 = cPrecisionClock::getCPUTimeSeconds();

    applyCommands();
    const MethodConfig& cfg = methodConfig((PotteryMethod)(int)activeMethod);

    // The wheel turns the clay; the HIP goes into the clay's frame.
    pottery->advanceWheel(a_dt, a_hip.spinning);
    cVector3d hipLocal = pottery->worldToLocal(a_hip.position);

    // Collision detection and response.
    detector.detect(*pottery, hipLocal, pottery->parameters().hipRadius, contacts);

    ResponseResult r;
    responder.respond(*pottery, contacts, hipLocal, a_armed, a_dt * kHapticRate,
                      cfg.force, cfg.propagation, r);

    // Local model for the servo loop.
    LocalForceModel m;
    m.x0             = a_hip.position;
    m.F0             = r.force;
    m.K              = boundStiffness(r.stiffness);
    m.normal         = r.normal;
    m.inContact      = r.inContact;
    m.dampingScale   = r.dampingScale;
    m.contacts       = (int)contacts.sectors.size();
    m.maxPenetration = contacts.maxPenetration;
    m.meanDepth      = r.meanDepth;
    m.sequence       = ++physicsSequence;

    // Changed rows to the graphic thread: every step at the physics rate,
    // every fourth at 1 kHz.
    if ((cfg.multirate || (++publishCounter >= 4)) && synchronizer.publishShape(*pottery))
    {
        publishCounter = 0;
    }

    physicsRate.signal(1);
    m.physicsUs = 1e6 * (cPrecisionClock::getCPUTimeSeconds() - t0);

    HapticFrame f;
    f.method         = (PotteryMethod)(int)activeMethod;
    f.hipRadius      = pottery->parameters().hipRadius;
    f.wheelAngle     = pottery->wheelAngle();
    f.contacts       = m.contacts;
    f.meanDepth      = m.meanDepth;
    f.rigidContact   = !contacts.rigid.empty();
    f.physicsRate    = physicsRate.getFrequency();
    f.rows           = pottery->rows();
    f.cols           = pottery->cols();
    synchronizer.publishFrame(f, FRAME_PHYSICS);

    return m;
}

//==============================================================================
// PHYSICS THREAD (Method 2)
//==============================================================================

void physicsLoop()
{
    physicsDone = false;

    double last = 0.0;
    bool   wasActive = false;

    while (running)
    {
        const MethodConfig& cfg = methodConfig((PotteryMethod)(int)activeMethod);
        if (!cfg.multirate)
        {
            // The baseline runs its physics in the servo loop.
            wasActive = false;
            cSleepMs(1);
            continue;
        }

        const double period = 1.0 / cfg.physicsRate;
        double now = cPrecisionClock::getCPUTimeSeconds();
        double dt  = wasActive ? (now - last) : period;
        dt = cClamp(dt, 1e-4, 0.05);
        last = now;
        wasActive = true;

        hipLock.acquire();
        HipState hip = sharedHip;
        bool armed   = sharedArmed;
        hipLock.release();

        physicsLock.acquire();
        LocalForceModel m = physicsStep(hip, armed, dt);
        physicsLock.release();

        modelExchange.publish(m);

        while (running && (cPrecisionClock::getCPUTimeSeconds() < last + period))
        {
            std::this_thread::yield();
        }
    }

    physicsDone = true;
}

//==============================================================================
// SERVO THREAD (1 kHz)
//==============================================================================

void issueBenchmarkSetup(const BenchmarkStep& a_step)
{
    commandLock.acquire();
    pendingCommands.method     = a_step.method;
    pendingCommands.resolution = a_step.resolution;
    pendingCommands.reset      = true;
    pendingCommands.hollow     = 0;
    commandLock.release();

    benchTextLock.acquire();
    benchText = benchmark->progress();
    benchTextLock.release();
}

//------------------------------------------------------------------------------

void servoLoop()
{
    servoDone = false;

    const double period = 1.0 / kHapticRate;
    double last = cPrecisionClock::getCPUTimeSeconds();

    LocalModelBlend   blend;
    LowPassFilter     filter;
    cFrequencyCounter rate;

    if (benchmark && runBenchmark) { benchmark->start(last); }

    while (running)
    {
        double tickStart = cPrecisionClock::getCPUTimeSeconds();
        double dt = tickStart - last;
        last = tickStart;
        if ((dt <= 0.0) || (dt > 0.05)) { dt = period; }

        //--- HIP: the device, or the benchmark's scripted tool -----------------
        HipState hip = device.read();           // also paces the mouse device
        bool scripted  = benchmark && benchmark->isRunning();
        bool recording = false;
        if (scripted)
        {
            BenchmarkStep st = benchmark->step(tickStart, servoHipRadius);
            if (st.finished)
            {
                benchTextLock.acquire(); benchText.clear(); benchTextLock.release();
                quitRequested = true;
            }
            else
            {
                if (st.setup) { issueBenchmarkSetup(st); }
                hip.position = st.hip;
                hip.velocity = st.velocity;
                hip.spinning = st.spinning;
                recording    = st.recording;
            }
        }

        double workStart = cPrecisionClock::getCPUTimeSeconds();
        bool armed = device.isArmed();

        // Hand the HIP to the physics thread.
        if (hipLock.tryAcquire())
        {
            sharedHip   = hip;
            sharedArmed = armed;
            hipLock.release();
        }

        //--- contact force: full step (baseline) or local model ----------------
        const MethodConfig& cfg = methodConfig((PotteryMethod)(int)activeMethod);
        if (!cfg.multirate)
        {
            if (physicsLock.tryAcquire())
            {
                blend.reset(physicsStep(hip, armed, dt));
                physicsLock.release();
            }
        }
        else
        {
            LocalForceModel fresh;
            if (modelExchange.tryFetch(fresh)) { blend.update(fresh, tickStart, 1.0 / cfg.physicsRate); }
        }

        const LocalForceModel& model = blend.current();
        cVector3d force     = blend.evaluate(hip.position, tickStart);
        bool      inContact = blend.inContact(tickStart);

        // Han 2007, eq. (6): viscous resistance to HIP motion while in contact,
        // scaled by the contact area for the volume force.
        force -= (servoDamping.load() * blend.dampingScale(tickStart)) * hip.velocity;

        // Lee 2008: low-pass filter (baseline). Held at rest while force is
        // withheld, so it starts from zero when output is armed.
        double rawForce = force.length();
        double cutoff = servoLowPassHz;
        if (!armed || (cutoff <= 0.0))
        {
            filter.reset(armed ? force : cVector3d(0.0, 0.0, 0.0));
        }
        else
        {
            if (filter.cutoff() != cutoff) { filter.design(cutoff, kHapticRate); }
            force = filter.process(force);
        }

        double servoUs = 1e6 * (cPrecisionClock::getCPUTimeSeconds() - workStart);

        // The scripted tool is not the user's hand: nothing goes to the device.
        cVector3d sent = device.sendForce(scripted ? cVector3d(0.0, 0.0, 0.0) : force,
                                          inContact, dt);

        if (recording)
        {
            BenchmarkSample s;
            s.fx              = (float)force.x();
            s.fy              = (float)force.y();
            s.fz              = (float)force.z();
            s.fRaw            = (float)rawForce;
            s.maxDepth        = (float)model.maxPenetration;
            s.meanDepth       = (float)model.meanDepth;
            s.contacts        = model.contacts;
            s.servoUs         = (float)servoUs;
            s.physicsUs       = (float)model.physicsUs;
            s.physicsSequence = (int)model.sequence;
            benchmark->record(s, tickStart);
        }

        rate.signal(1);
        HapticFrame f;
        f.hipPosition = hip.position;
        f.force       = scripted ? force : sent;
        f.armed       = armed;
        f.hapticRate  = rate.getFrequency();
        synchronizer.publishFrame(f, FRAME_SERVO);

        device.waitForNextTick();
    }

    servoDone = true;
}

//==============================================================================
// GRAPHIC THREAD: input
//==============================================================================

void pushParams()
{
    commandLock.acquire();
    pendingCommands.paramsChanged = true;
    pendingCommands.params        = uiParams;
    commandLock.release();
}

void selectMethod(PotteryMethod a_method)
{
    uiMethod = a_method;
    uiParams.lowPassHz = methodConfig(a_method).lowPassHz;

    commandLock.acquire();
    pendingCommands.method        = a_method;
    pendingCommands.paramsChanged = true;
    pendingCommands.params        = uiParams;
    commandLock.release();

    visual.setStatus(methodConfig(a_method).name);
}

double stepped(double a_value, bool a_down, double a_lo, double a_hi)
{
    return cClamp(a_down ? a_value / 1.25 : a_value * 1.25, a_lo, a_hi);
}

//------------------------------------------------------------------------------

void onKey(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods)
{
    if ((a_action != GLFW_PRESS) && (a_action != GLFW_REPEAT)) { return; }

    // A benchmark must not be disturbed: only quitting is allowed.
    if (runBenchmark && (a_key != GLFW_KEY_ESCAPE) && (a_key != GLFW_KEY_Q)) { return; }

    const bool down = (a_mods & GLFW_MOD_SHIFT) != 0;
    PotteryParameters& p = uiParams;
    ostringstream msg;
    msg << fixed;

    switch (a_key)
    {
        case GLFW_KEY_ESCAPE:
        case GLFW_KEY_Q:
            glfwSetWindowShouldClose(a_window, GLFW_TRUE);
            return;

        case GLFW_KEY_F:
        {
            fullscreen = !fullscreen;
            GLFWmonitor* monitor = glfwGetPrimaryMonitor();
            const GLFWvidmode* mode = glfwGetVideoMode(monitor);
            if (fullscreen)
            {
                glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
            }
            else
            {
                int w = (int)(0.8 * mode->height), h = (int)(0.6 * mode->height);
                glfwSetWindowMonitor(window, NULL, (mode->width - w) / 2, (mode->height - h) / 2,
                                     w, h, mode->refreshRate);
            }
            glfwSwapInterval(1);
            return;
        }

        case GLFW_KEY_F1: visual.toggleHelp();      return;
        case GLFW_KEY_W:  visual.toggleWireframe(); return;

        //--- methods ------------------------------------------------------------
        case GLFW_KEY_0: selectMethod(METHOD_BASELINE); return;
        case GLFW_KEY_2: selectMethod(METHOD_2);        return;

        //--- wheel --------------------------------------------------------------
        case GLFW_KEY_SPACE:
            device.toggleSpinLatch();
            visual.setStatus(device.isSpinLatched() ? "Wheel spinning" : "Wheel stopped");
            return;

        case GLFW_KEY_UP:
        case GLFW_KEY_DOWN:
            p.wheelSpeed = cClamp(p.wheelSpeed + ((a_key == GLFW_KEY_UP) ? 0.25 : -0.25), 0.25, 4.0);
            msg << setprecision(0) << "Wheel " << p.wheelSpeed * 60.0 << " rpm";
            break;

        //--- tool ---------------------------------------------------------------
        case GLFW_KEY_LEFT_BRACKET:
        case GLFW_KEY_RIGHT_BRACKET:
            p.hipRadius = cClamp(p.hipRadius + ((a_key == GLFW_KEY_RIGHT_BRACKET) ? 0.001 : -0.001), 0.002, 0.015);
            msg << setprecision(1) << "Tool radius " << p.hipRadius * 1000.0 << " mm";
            break;

        //--- clay ---------------------------------------------------------------
        case GLFW_KEY_R:
            commandLock.acquire(); pendingCommands.reset = true; commandLock.release();
            visual.setStatus("Clay reset");
            return;

        case GLFW_KEY_O:
            uiHollow = !uiHollow;
            commandLock.acquire(); pendingCommands.hollow = uiHollow ? 1 : 0; commandLock.release();
            visual.setStatus(uiHollow ? "Clay opened" : "Clay closed");
            return;

        case GLFW_KEY_COMMA:
        case GLFW_KEY_PERIOD:
        {
            int& res = uiResolution[uiMethod];
            int next = cClamp(res + ((a_key == GLFW_KEY_PERIOD) ? 1 : -1), 0, kNumResolutionPresets - 1);
            if (next == res) { return; }
            res = next;
            commandLock.acquire(); pendingCommands.resolution = next; commandLock.release();
            ostringstream r;
            r << "Resolution " << kResolutionPresets[next][0] << " x " << kResolutionPresets[next][1];
            visual.setStatus(r.str());
            return;
        }

        //--- physical constants -------------------------------------------------
        case GLFW_KEY_K:
            p.resistanceSpinning       = stepped(p.resistanceSpinning,       down, 20.0,  3000.0);
            p.resistanceStopped        = stepped(p.resistanceStopped,        down, 20.0,  3000.0);
            p.volumeResistanceSpinning = stepped(p.volumeResistanceSpinning, down, 3.0e5, 2.0e8);
            p.volumeResistanceStopped  = stepped(p.volumeResistanceStopped,  down, 3.0e5, 2.0e8);
            msg << (down ? "Softer clay" : "Stiffer clay");
            break;

        case GLFW_KEY_V:
            p.viscositySpinning = stepped(p.viscositySpinning, down, 5.0, 5000.0);
            p.viscosityStopped  = stepped(p.viscosityStopped,  down, 5.0, 5000.0);
            msg << setprecision(0) << "Viscosity " << p.viscositySpinning << " / " << p.viscosityStopped;
            break;

        case GLFW_KEY_D:
            p.damping = stepped(p.damping, down, 0.05, 5.0);
            msg << setprecision(2) << "Damping " << p.damping << " N.s/m";
            break;

        case GLFW_KEY_C:
            p.friction      = stepped(p.friction,      down, 1.0,  2000.0);
            p.frictionCoeff = stepped(p.frictionCoeff, down, 0.02, 2.0);
            msg << (down ? "Less friction" : "More friction");
            break;

        case GLFW_KEY_P:
            p.propagation = !p.propagation;
            msg << "Propagation " << (p.propagation ? "on" : "off");
            break;

        default:
            return;
    }

    pushParams();
    visual.setStatus(msg.str());
}

//------------------------------------------------------------------------------

void onMouseButton(GLFWwindow*, int a_button, int a_action, int)
{
    if (runBenchmark) { return; }

    // Left button: the method buttons.
    if ((a_button == GLFW_MOUSE_BUTTON_LEFT) && (a_action == GLFW_PRESS))
    {
        int m = visual.methodButtonAt(mouseX, mouseY, windowW, windowH);
        if (m >= 0) { selectMethod((PotteryMethod)m); }
        return;
    }

    // Right button stands in for the stylus button: hold to spin.
    if (a_button == GLFW_MOUSE_BUTTON_RIGHT) { device.setMouseButton(a_action == GLFW_PRESS); }
}

void onCursor(GLFWwindow*, double a_x, double a_y)
{
    mouseX = a_x;
    mouseY = a_y;
    mouseSeen = true;
}

void onScroll(GLFWwindow*, double, double a_dy)
{
    // Moves the HIP plane towards (+) or away from (-) the viewer.
    mouseDepth = cClamp(mouseDepth + 0.002 * a_dy, -0.06, 0.06);
}

void onWindowSize(GLFWwindow*, int a_w, int a_h)       { windowW = a_w; windowH = a_h; }
void onFramebufferSize(GLFWwindow*, int a_w, int a_h)  { framebufferW = a_w; framebufferH = a_h; }
void onContentScale(GLFWwindow*, float a_x, float a_y) { visual.setContentScale(a_x, a_y); }
void onError(int, const char* a_text)                  { cout << "GLFW error: " << a_text << endl; }

//------------------------------------------------------------------------------

//! The shaders are copied next to the executable at build time; the source
//! folder is the fallback when running from elsewhere.
string findShaderDir(const char* a_argv0)
{
    string exe = a_argv0;
    size_t slash = exe.find_last_of("/\\");
    string dir = ((slash == string::npos) ? string(".") : exe.substr(0, slash)) + "/Shaders";

    if (ifstream((dir + "/pottery.vert").c_str())) { return dir; }
#ifdef VP_SHADER_DIR
    return VP_SHADER_DIR;
#else
    return dir;
#endif
}

//==============================================================================
// MAIN
//==============================================================================

int main(int argc, char* argv[])
{
    cout << endl
         << "---------------------------------------------------------" << endl
         << " VIRTUAL POTTERY" << endl
         << " circular sector elements + response force propagation" << endl
         << " Baseline / Method 2 - press F1 for the keys" << endl
         << "---------------------------------------------------------" << endl << endl;

    for (int m = 0; m < kNumMethods; m++)
    {
        methodResolution[m] = uiResolution[m] = methodConfig((PotteryMethod)m).resolution;
    }

    for (int a = 1; a < argc; a++)
    {
        string arg = argv[a];
        if ((arg == "--method") && (a + 1 < argc)) { uiMethod = (atoi(argv[++a]) == 0) ? METHOD_BASELINE : METHOD_2; }
        else if (arg == "--open")                  { uiHollow = true; }
        else if (arg == "--bench")                 { runBenchmark = true; }
        else                                       { cout << "unknown argument: " << arg << endl; }
    }

    //--------------------------------------------------------------------------
    // Window and OpenGL context
    //--------------------------------------------------------------------------
    if (!glfwInit()) { cout << "GLFW initialisation failed" << endl; return 1; }
    glfwSetErrorCallback(onError);

    // GLFW_SCALE_TO_MONITOR enlarges the window by the content scale, so the
    // requested size is divided by it up front.
    GLFWmonitor* monitor = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = glfwGetVideoMode(monitor);
    float scaleX = 1.0f, scaleY = 1.0f;
    glfwGetMonitorContentScale(monitor, &scaleX, &scaleY);

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_TRUE);
    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_SCALE_TO_MONITOR, GLFW_TRUE);

    window = glfwCreateWindow((int)(0.9 * mode->height / scaleX), (int)(0.7 * mode->height / scaleY),
                              "Virtual Pottery", NULL, NULL);
    if (!window) { cout << "cannot create the window" << endl; glfwTerminate(); return 1; }

    glfwGetWindowSize(window, &windowW, &windowH);
    glfwSetWindowPos(window, std::max(0, (mode->width - windowW) / 2), std::max(30, (mode->height - windowH) / 2));
    glfwGetFramebufferSize(window, &framebufferW, &framebufferH);

    glfwSetKeyCallback(window, onKey);
    glfwSetCursorPosCallback(window, onCursor);
    glfwSetScrollCallback(window, onScroll);
    glfwSetMouseButtonCallback(window, onMouseButton);
    glfwSetWindowSizeCallback(window, onWindowSize);
    glfwSetFramebufferSizeCallback(window, onFramebufferSize);
    glfwSetWindowContentScaleCallback(window, onContentScale);

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

#ifdef GLEW_VERSION
    if (glewInit() != GLEW_OK) { cout << "GLEW initialisation failed" << endl; glfwTerminate(); return 1; }
#endif

    //--------------------------------------------------------------------------
    // Core data, in the starting method's resolution
    //--------------------------------------------------------------------------
    const MethodConfig& startCfg = methodConfig(uiMethod);
    const int* res = kResolutionPresets[startCfg.resolution];
    pottery = new VirtualPottery(res[0], res[1]);
    pottery->setHollow(uiHollow);
    activeMethod = uiMethod;
    pottery->parameters().lowPassHz = startCfg.lowPassHz;
    uiParams = pottery->parameters();

    //--------------------------------------------------------------------------
    // Haptic device
    //--------------------------------------------------------------------------
    if (!device.open()) { cout << "cannot open the haptic device" << endl; glfwTerminate(); return 1; }
    pushParams();       // clamps the stiffness to what the device can render

    //--------------------------------------------------------------------------
    // Graphic side
    //--------------------------------------------------------------------------
    float contentX, contentY;
    glfwGetWindowContentScale(window, &contentX, &contentY);

    string shaderDir = findShaderDir(argv[0]);
    if (!visual.init(&synchronizer, shaderDir, contentX, contentY))
    {
        cout << "cannot set up the pottery shaders (looked in " << shaderDir << ")" << endl;
        device.close();
        glfwTerminate();
        return 1;
    }

    synchronizer.publishAll(*pottery);

#ifdef VP_RESULTS_DIR
    if (runBenchmark) { benchmark = new Benchmark(VP_RESULTS_DIR); }
#endif

    //--------------------------------------------------------------------------
    // Start the haptic-side threads
    //--------------------------------------------------------------------------
    running     = true;
    servoDone   = false;
    physicsDone = false;
    physicsThread = new cThread();
    physicsThread->start(physicsLoop, CTHREAD_PRIORITY_HAPTICS);
    servoThread = new cThread();
    servoThread->start(servoLoop, CTHREAD_PRIORITY_HAPTICS);

    //--------------------------------------------------------------------------
    // Graphic loop
    //--------------------------------------------------------------------------
    HudInfo hud;

    while (!glfwWindowShouldClose(window))
    {
        // Mouse fallback: the HIP follows the cursor on a vertical plane
        // through the wheel axis, where the clay's silhouette lies.
        if (device.isSimulated() && mouseSeen)
        {
            cVector3d target;
            if (visual.cursorToPlane(mouseX, mouseY, windowW, windowH, mouseDepth, target))
            {
                device.setMouseTarget(target);
            }
        }

        // Data Synchronization: changed radius rows -> GPU texture.
        synchronizer.synchronize();

        // Visualize Pottery.
        benchTextLock.acquire();
        hud.benchmark = benchText;
        benchTextLock.release();
        visual.render(framebufferW, framebufferH, synchronizer.frame(), hud);

        glfwSwapBuffers(window);
        glfwPollEvents();

        if (quitRequested) { glfwSetWindowShouldClose(window, GLFW_TRUE); }
    }

    //--------------------------------------------------------------------------
    // Shut down: haptic-side threads, the device, then the scene.
    //--------------------------------------------------------------------------
    running = false;
    while (!servoDone || !physicsDone) { cSleepMs(1); }
    delete servoThread;
    delete physicsThread;

    device.close();
    delete pottery;
    delete benchmark;

    visual.shutdown();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
