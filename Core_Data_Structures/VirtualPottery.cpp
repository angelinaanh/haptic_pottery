//==============================================================================
//  Core data: the virtual pottery. See VirtualPottery.h.
//==============================================================================
#include "VirtualPottery.h"

#include <algorithm>
#include <cmath>

using namespace chai3d;

namespace vp {

//------------------------------------------------------------------------------
// Helpers
//------------------------------------------------------------------------------

namespace {

//! Bilinear sample of a radius grid: linear in height, wrapping in angle.
double sampleGrid(const std::vector<float>& a_r, int a_rows, int a_cols,
                  double a_baseZ, double a_dh, double a_dTheta,
                  double a_z, double a_theta)
{
    double fr = cClamp((a_z - a_baseZ) / a_dh, 0.0, (double)(a_rows - 1));
    int    i0 = std::min((int)fr, a_rows - 1);
    int    i1 = std::min(i0 + 1, a_rows - 1);
    double a  = fr - i0;

    double t = std::fmod(a_theta, 2.0 * C_PI);
    if (t < 0.0) { t += 2.0 * C_PI; }

    double fc = t / a_dTheta;
    int    j0 = ((int)fc) % a_cols;
    int    j1 = (j0 + 1) % a_cols;
    double b  = fc - std::floor(fc);

    double r0 = (1.0 - b) * a_r[i0 * a_cols + j0] + b * a_r[i0 * a_cols + j1];
    double r1 = (1.0 - b) * a_r[i1 * a_cols + j0] + b * a_r[i1 * a_cols + j1];

    return (1.0 - a) * r0 + a * r1;
}

} // namespace

//==============================================================================

VirtualPottery::VirtualPottery(int a_rows, int a_cols)
{
    m_rows = std::max(2, a_rows);
    m_cols = std::max(4, a_cols);
    buildTables();

    m_radius.assign((size_t)m_rows * m_cols, (float)kClayInitialRadius);

    m_wheelAngle = 0.0;
    m_omega      = 0.0;

    markAllDirty();
}

//------------------------------------------------------------------------------

void VirtualPottery::buildTables()
{
    m_dh     = m_geometry.height / (double)(m_rows - 1);
    m_dTheta = 2.0 * C_PI / (double)m_cols;

    m_cos.resize(m_cols);
    m_sin.resize(m_cols);
    for (int j = 0; j < m_cols; j++)
    {
        m_cos[j] = std::cos(j * m_dTheta);
        m_sin[j] = std::sin(j * m_dTheta);
    }
}

//------------------------------------------------------------------------------

void VirtualPottery::reset(double a_radius)
{
    std::fill(m_radius.begin(), m_radius.end(), (float)clampRadius(a_radius));
    markAllDirty();
}

//------------------------------------------------------------------------------

void VirtualPottery::resize(int a_rows, int a_cols)
{
    std::vector<float> old = m_radius;
    int    oldRows   = m_rows;
    int    oldCols   = m_cols;
    double oldDh     = m_dh;
    double oldDTheta = m_dTheta;

    m_rows = std::max(2, a_rows);
    m_cols = std::max(4, a_cols);
    buildTables();

    m_radius.resize((size_t)m_rows * m_cols);
    for (int i = 0; i < m_rows; i++)
    {
        for (int j = 0; j < m_cols; j++)
        {
            double r = sampleGrid(old, oldRows, oldCols, m_geometry.baseZ, oldDh, oldDTheta,
                                  rowHeight(i), j * m_dTheta);
            m_radius[index(i, j)] = (float)clampRadius(r);
        }
    }

    markAllDirty();
}

//------------------------------------------------------------------------------

void VirtualPottery::setHollow(bool a_hollow)
{
    m_geometry.hollow = a_hollow;

    // Opening the body raises the lower radius limit to the wall thickness.
    for (size_t k = 0; k < m_radius.size(); k++)
    {
        m_radius[k] = (float)clampRadius(m_radius[k]);
    }

    markAllDirty();
}

//------------------------------------------------------------------------------

CircularSectorElement VirtualPottery::element(int a_row, int a_col) const
{
    CircularSectorElement e;
    e.radius = m_radius[index(a_row, a_col)];
    e.theta  = a_col * m_dTheta;
    e.height = rowHeight(a_row);
    return e;
}

//------------------------------------------------------------------------------

void VirtualPottery::setRadiusAt(int a_index, double a_radius)
{
    m_radius[a_index] = (float)clampRadius(a_radius);

    int row = a_index / m_cols;
    if (row < m_dirtyMin) { m_dirtyMin = row; }
    if (row > m_dirtyMax) { m_dirtyMax = row; }
}

//------------------------------------------------------------------------------

double VirtualPottery::sampleRadius(double a_z, double a_theta) const
{
    return sampleGrid(m_radius, m_rows, m_cols, m_geometry.baseZ, m_dh, m_dTheta, a_z, a_theta);
}

//------------------------------------------------------------------------------

double VirtualPottery::clampRadius(double a_radius) const
{
    // A hollow body keeps a few millimetres of opening inside its wall.
    double lo = m_geometry.hollow ? (m_geometry.wall + 0.003) : kClayMinRadius;
    return cClamp(a_radius, lo, kClayMaxRadius);
}

//------------------------------------------------------------------------------

void VirtualPottery::advanceWheel(double a_dt, bool a_spinning)
{
    m_omega      = a_spinning ? 2.0 * C_PI * m_params.wheelSpeed : 0.0;
    m_wheelAngle = std::fmod(m_wheelAngle + m_omega * a_dt, 2.0 * C_PI);
}

//------------------------------------------------------------------------------

cVector3d VirtualPottery::worldToLocal(const cVector3d& a_v) const
{
    double c = std::cos(m_wheelAngle), s = std::sin(m_wheelAngle);
    return cVector3d(c * a_v.x() + s * a_v.y(), -s * a_v.x() + c * a_v.y(), a_v.z());
}

cVector3d VirtualPottery::localToWorld(const cVector3d& a_v) const
{
    double c = std::cos(m_wheelAngle), s = std::sin(m_wheelAngle);
    return cVector3d(c * a_v.x() - s * a_v.y(), s * a_v.x() + c * a_v.y(), a_v.z());
}

} // namespace vp
