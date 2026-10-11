//==============================================================================
//  Haptic thread: multirate local stiffness model. See LocalForceModel.h.
//==============================================================================
#include "LocalForceModel.h"
#include "math/CMaths.h"

#include <algorithm>
#include <cmath>

using namespace chai3d;

namespace vp {

//------------------------------------------------------------------------------

namespace {

//! Furthest the HIP may move from x0 before the model stops extrapolating [m].
//! At 1 kHz and a few hundred Hz of physics this is several times the
//! distance a hand covers between two physics steps.
const double kMaxExtrapolation = 0.002;

} // namespace

//------------------------------------------------------------------------------

cVector3d LocalForceModel::evaluate(const cVector3d& a_x) const
{
    if (!inContact) { return cVector3d(0.0, 0.0, 0.0); }

    cVector3d dx = a_x - x0;
    double len = dx.length();
    if (len > kMaxExtrapolation) { dx *= kMaxExtrapolation / len; }

    cVector3d force = F0 - K * dx;

    // A penalty force pushes out of the clay; extrapolating past the surface
    // must not turn it into a pull.
    double along = cDot(force, normal);
    if (along < 0.0) { force -= along * normal; }

    return force;
}

//------------------------------------------------------------------------------

void LocalModelBlend::update(const LocalForceModel& a_model, double a_now, double a_period)
{
    m_previous = m_current;
    m_current  = a_model;
    m_start    = a_now;
    m_period   = std::max(1e-4, a_period);
}

void LocalModelBlend::reset(const LocalForceModel& a_model)
{
    m_previous = a_model;
    m_current  = a_model;
    m_start    = 0.0;
}

double LocalModelBlend::weight(double a_now) const
{
    double w = (a_now - m_start) / m_period;
    return (w < 0.0) ? 0.0 : ((w > 1.0) ? 1.0 : w);
}

cVector3d LocalModelBlend::evaluate(const cVector3d& a_x, double a_now) const
{
    double w = weight(a_now);
    if (w >= 1.0) { return m_current.evaluate(a_x); }
    return w * m_current.evaluate(a_x) + (1.0 - w) * m_previous.evaluate(a_x);
}

double LocalModelBlend::dampingScale(double a_now) const
{
    double w = weight(a_now);
    return w * m_current.dampingScale + (1.0 - w) * m_previous.dampingScale;
}

bool LocalModelBlend::inContact(double a_now) const
{
    return m_current.inContact || ((weight(a_now) < 1.0) && m_previous.inContact);
}

//------------------------------------------------------------------------------

void LocalModelExchange::publish(const LocalForceModel& a_model)
{
    m_lock.acquire();
    m_model = a_model;
    m_fresh = true;
    m_lock.release();
}

bool LocalModelExchange::tryFetch(LocalForceModel& a_model)
{
    if (!m_lock.tryAcquire()) { return false; }
    bool fresh = m_fresh;
    if (fresh)
    {
        a_model = m_model;
        m_fresh = false;
    }
    m_lock.release();
    return fresh;
}

//==============================================================================
//  LowPassFilter
//==============================================================================

LowPassFilter::LowPassFilter()
{
    m_cutoff = -1.0;
    design(10.0, 1000.0);
    reset(cVector3d(0.0, 0.0, 0.0));
}

void LowPassFilter::design(double a_cutoffHz, double a_sampleHz)
{
    m_cutoff = a_cutoffHz;

    double K    = std::tan(C_PI * a_cutoffHz / a_sampleHz);
    double K2   = K * K;
    double s2   = std::sqrt(2.0);
    double norm = 1.0 / (1.0 + s2 * K + K2);

    m_b0 = K2 * norm;
    m_b1 = 2.0 * m_b0;
    m_b2 = m_b0;
    m_a1 = 2.0 * (K2 - 1.0) * norm;
    m_a2 = (1.0 - s2 * K + K2) * norm;
}

void LowPassFilter::reset(const cVector3d& a_value)
{
    m_x1 = m_x2 = m_y1 = m_y2 = a_value;
}

cVector3d LowPassFilter::process(const cVector3d& a_x)
{
    cVector3d y = m_b0 * a_x + m_b1 * m_x1 + m_b2 * m_x2 - m_a1 * m_y1 - m_a2 * m_y2;
    m_x2 = m_x1;  m_x1 = a_x;
    m_y2 = m_y1;  m_y1 = y;
    return y;
}

} // namespace vp
