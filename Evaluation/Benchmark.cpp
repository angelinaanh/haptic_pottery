//==============================================================================
//  Evaluation: scripted comparison of the interaction methods.
//  See Benchmark.h.
//==============================================================================
#include "Benchmark.h"
#include "VirtualPottery.h"

#include <cstdio>
#include <iostream>
#include <sstream>

using namespace chai3d;
using namespace std;

namespace vp {

//------------------------------------------------------------------------------

namespace {

const double kSettleTime   = 0.8;       // HIP parked at the start pose [s]

const double kPushStartGap = 0.002;     // clearance at the start of a push [m]
const double kPushSpeed    = 0.005;     // [m/s]
const double kPushMoveTime = 0.8;       // [s]
const double kPushHoldTime = 0.5;       // [s]

const double kGrooveDepth  = 0.0025;    // [m]
const double kGrooveEntry  = 0.3;       // time to reach the depth [s]
const double kGrooveTravel = 5.0;       // top to bottom [s]

//! 0 -> 1 with zero slope at both ends, and its derivative.
double smooth(double a_u)  { a_u = cClamp(a_u, 0.0, 1.0); return a_u * a_u * (3.0 - 2.0 * a_u); }
double smoothD(double a_u) { if ((a_u <= 0.0) || (a_u >= 1.0)) { return 0.0; } return 6.0 * a_u * (1.0 - a_u); }

const char* scenarioName(Scenario a_s) { return (a_s == SCENARIO_PUSH) ? "push" : "groove"; }

const char* methodTag(PotteryMethod a_m) { return (a_m == METHOD_BASELINE) ? "baseline" : "method2"; }

} // namespace

//==============================================================================

Benchmark::Benchmark(const string& a_outputDir)
    : m_dir(a_outputDir), m_running(false), m_current(0),
      m_phase(PHASE_SETUP), m_phaseStart(0.0)
{
}

//------------------------------------------------------------------------------

void Benchmark::start(double a_now)
{
    m_plan.clear();
    m_index.clear();

    const PotteryMethod methods[] = { METHOD_BASELINE, METHOD_2 };
    for (PotteryMethod m : methods)
    {
        // Force, depth and contacts at a coarse and a fine grid.
        Run p1 = { m, SCENARIO_PUSH, 1 };
        Run p2 = { m, SCENARIO_PUSH, 4 };
        m_plan.push_back(p1);
        m_plan.push_back(p2);

        // Load and timing over the resolutions.
        const int grooveRes[] = { 1, 3, 4, 5 };
        for (int r : grooveRes)
        {
            Run g = { m, SCENARIO_GROOVE, r };
            m_plan.push_back(g);
        }
    }

    m_running    = true;
    m_current    = 0;
    m_phase      = PHASE_SETUP;
    m_phaseStart = a_now;

    cout << "[bench] " << m_plan.size() << " runs, results in " << m_dir << endl;
}

//------------------------------------------------------------------------------

double Benchmark::duration(Scenario a_scenario) const
{
    return (a_scenario == SCENARIO_PUSH) ? (kPushMoveTime + kPushHoldTime)
                                         : (kGrooveEntry + kGrooveTravel);
}

//------------------------------------------------------------------------------

void Benchmark::trajectory(const Run& a_run, double a_t, double a_hipRadius,
                           BenchmarkStep& a_step) const
{
    // The HIP works the +y side of the clay: the right-hand silhouette.
    const double surface = kClayInitialRadius + a_hipRadius;

    if (a_run.scenario == SCENARIO_PUSH)
    {
        // In by kPushSpeed * kPushMoveTime, mean speed kPushSpeed, then hold.
        double z    = kWheelTopZ + 0.5 * kClayHeight;
        double L    = kPushSpeed * kPushMoveTime;
        double u    = a_t / kPushMoveTime;
        double d    = surface + kPushStartGap - L * smooth(u);

        a_step.hip.set(0.0, d, z);
        a_step.velocity.set(0.0, -L * smoothD(u) / kPushMoveTime, 0.0);
        a_step.spinning = false;
    }
    else
    {
        double zTop    = kWheelTopZ + kClayHeight - 0.004;
        double zBottom = kWheelTopZ + 0.008;
        double dIn     = surface - kGrooveDepth;
        double dOut    = surface + 0.003;

        // In to the depth, then down; each leg starts and ends at rest.
        double ue = a_t / kGrooveEntry;
        double ud = (a_t - kGrooveEntry) / kGrooveTravel;
        double d  = dOut + (dIn - dOut) * smooth(ue);
        double z  = zTop + (zBottom - zTop) * smooth(ud);
        cVector3d v(0.0, (dIn - dOut) * smoothD(ue) / kGrooveEntry,
                         (zBottom - zTop) * smoothD(ud) / kGrooveTravel);

        a_step.hip.set(0.0, d, z);
        a_step.velocity = v;
        a_step.spinning = true;
    }
}

//------------------------------------------------------------------------------

BenchmarkStep Benchmark::step(double a_now, double a_hipRadius)
{
    BenchmarkStep st;
    if (!m_running) { st.finished = true; return st; }

    const Run& run = m_plan[m_current];
    st.method     = run.method;
    st.resolution = run.resolution;

    double t = a_now - m_phaseStart;

    switch (m_phase)
    {
        case PHASE_SETUP:
            // Ask for the method, the grid and a fresh cylinder; then wait at
            // the start pose so the commands land and force output arms.
            st.setup = true;
            trajectory(run, 0.0, a_hipRadius, st);
            st.velocity.zero();
            m_samples.clear();
            m_phase = PHASE_SETTLE;
            m_phaseStart = a_now;
            break;

        case PHASE_SETTLE:
            trajectory(run, 0.0, a_hipRadius, st);
            st.velocity.zero();
            if (t >= kSettleTime)
            {
                m_phase = PHASE_RUN;
                m_phaseStart = a_now;
            }
            break;

        case PHASE_RUN:
            trajectory(run, t, a_hipRadius, st);
            st.recording = true;
            if (t >= duration(run.scenario))
            {
                st.recording = false;
                writeRun(run);

                if (++m_current >= m_plan.size())
                {
                    writeIndex();
                    m_running   = false;
                    st.finished = true;
                    cout << "[bench] done" << endl;
                }
                else
                {
                    m_phase = PHASE_SETUP;
                    m_phaseStart = a_now;
                }
            }
            break;
    }

    return st;
}

//------------------------------------------------------------------------------

void Benchmark::record(BenchmarkSample a_sample, double a_now)
{
    a_sample.t = (float)(a_now - m_phaseStart);
    m_samples.push_back(a_sample);
}

//------------------------------------------------------------------------------

string Benchmark::progress() const
{
    if (!m_running) { return ""; }
    ostringstream s;
    const Run& r = m_plan[m_current];
    s << "Benchmark " << (m_current + 1) << "/" << m_plan.size() << ": "
      << methodConfig(r.method).name << ", " << scenarioName(r.scenario) << ", "
      << kResolutionPresets[r.resolution][0] << "x" << kResolutionPresets[r.resolution][1];
    return s.str();
}

//------------------------------------------------------------------------------

void Benchmark::writeRun(const Run& a_run)
{
    int rows = kResolutionPresets[a_run.resolution][0];
    int cols = kResolutionPresets[a_run.resolution][1];

    ostringstream name;
    name << methodTag(a_run.method) << "_" << scenarioName(a_run.scenario) << "_"
         << rows << "x" << cols << ".csv";

    string path = m_dir + "/" + name.str();
    FILE* f = fopen(path.c_str(), "w");
    if (f)
    {
        fprintf(f, "t,fx,fy,fz,f_raw,max_depth,mean_depth,contacts,servo_us,physics_us,physics_seq\n");
        for (const BenchmarkSample& s : m_samples)
        {
            fprintf(f, "%.5f,%.5f,%.5f,%.5f,%.5f,%.7f,%.7f,%d,%.2f,%.2f,%d\n",
                    s.t, s.fx, s.fy, s.fz, s.fRaw, s.maxDepth, s.meanDepth, s.contacts,
                    s.servoUs, s.physicsUs, s.physicsSequence);
        }
        fclose(f);
    }

    const MethodConfig& cfg = methodConfig(a_run.method);
    ostringstream row;
    row << methodTag(a_run.method) << "," << cfg.name << "," << scenarioName(a_run.scenario) << ","
        << rows << "," << cols << ","
        << (cfg.multirate ? cfg.physicsRate : 1000.0) << "," << name.str() << ","
        << m_samples.size();
    m_index.push_back(row.str());

    cout << "[bench] " << name.str() << " (" << m_samples.size() << " ticks)" << endl;
}

//------------------------------------------------------------------------------

void Benchmark::writeIndex()
{
    FILE* f = fopen((m_dir + "/runs.csv").c_str(), "w");
    if (!f) { return; }
    fprintf(f, "tag,method,scenario,rows,cols,physics_rate,file,ticks\n");
    for (const string& r : m_index) { fprintf(f, "%s\n", r.c_str()); }
    fclose(f);
}

} // namespace vp
