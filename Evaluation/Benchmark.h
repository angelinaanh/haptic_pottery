//==============================================================================
//  Evaluation: scripted comparison of the interaction methods.
//
//  A scripted HIP replaces the device and drives the full system - both
//  haptic threads, the real collision detection and response - through the
//  same trajectories for every method and resolution. Every servo tick is
//  recorded: the force the device would receive, the penetration depth, the
//  number of colliding elements and the time spent.
//
//  Scenarios
//    push    wheel stopped, the HIP moves slowly into the side of the clay and
//            holds (Lee 2008, Fig. 8): force, depth and contacts against time.
//    groove  wheel turning, the HIP travels from the top to the bottom at a
//            fixed depth (Lee 2009, Sec. III): load and timing.
//
//  The trajectories have continuous velocity (smoothstep profiles), as a hand
//  does; an instant change of velocity would step the viscous term -c2 v and
//  show up as a force jump that no method could avoid.
//
//  Each run is written to <dir>/<method>_<scenario>_<rows>x<cols>.csv, and an
//  index of all runs to <dir>/runs.csv. Evaluation/plot_comparison.py turns
//  them into figures and a summary table.
//
//  Threading: the plan runs inside the servo thread; file writes happen while
//  the HIP is parked between runs.
//==============================================================================
#ifndef VP_BENCHMARK_H
#define VP_BENCHMARK_H

#include "MethodConfig.h"
#include "math/CVector3d.h"

#include <string>
#include <vector>

namespace vp {

//------------------------------------------------------------------------------

enum Scenario
{
    SCENARIO_PUSH   = 0,
    SCENARIO_GROOVE = 1
};

//! One servo tick of a run.
struct BenchmarkSample
{
    float t;                    //!< time since the start of the run [s]
    float fx, fy, fz;           //!< force for the device, world frame [N]
    float fRaw;                 //!< its magnitude before the low-pass filter [N]
    float maxDepth;             //!< deepest penetration [m]
    float meanDepth;            //!< area-weighted penetration [m]
    int   contacts;             //!< colliding elements
    float servoUs;              //!< work of the servo tick [us]
    float physicsUs;            //!< duration of the newest physics step [us]
    int   physicsSequence;      //!< increments with every physics step
};

//! What the servo thread should do this tick.
struct BenchmarkStep
{
    chai3d::cVector3d hip;          //!< scripted HIP, world frame
    chai3d::cVector3d velocity;
    bool spinning   = false;
    bool setup      = false;        //!< prepare the next run now (once)
    PotteryMethod method = METHOD_BASELINE;
    int  resolution = 0;
    bool recording  = false;
    bool finished   = false;        //!< the whole plan is done
};

//==============================================================================

class Benchmark
{
public:

    explicit Benchmark(const std::string& a_outputDir);

    //! Builds the plan and starts it.
    void start(double a_now);

    bool isRunning() const { return m_running; }

    //! Servo thread, every tick.
    BenchmarkStep step(double a_now, double a_hipRadius);

    //! Servo thread, on ticks where step() said recording. Stamps the
    //! sample with the time since the start of the run.
    void record(BenchmarkSample a_sample, double a_now);

    //! Short text for the screen, e.g. "Benchmark 4/12".
    std::string progress() const;

private:

    struct Run
    {
        PotteryMethod method;
        Scenario      scenario;
        int           resolution;
    };

    enum Phase { PHASE_SETUP, PHASE_SETTLE, PHASE_RUN };

    double duration(Scenario a_scenario) const;
    void   trajectory(const Run& a_run, double a_t, double a_hipRadius,
                      BenchmarkStep& a_step) const;
    void   writeRun(const Run& a_run);
    void   writeIndex();

    std::string m_dir;
    std::vector<Run> m_plan;
    std::vector<BenchmarkSample> m_samples;
    std::vector<std::string> m_index;

    bool   m_running;
    size_t m_current;
    Phase  m_phase;
    double m_phaseStart;
};

} // namespace vp

#endif // VP_BENCHMARK_H
