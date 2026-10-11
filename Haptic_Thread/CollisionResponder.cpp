//==============================================================================
//  Haptic thread: collision response. See CollisionResponder.h.
//==============================================================================
#include "CollisionResponder.h"

#include <algorithm>
#include <cmath>

using namespace chai3d;

namespace vp {

//------------------------------------------------------------------------------

CollisionResponder::CollisionResponder()
{
    m_propTick = 0;
    m_cacheRt  = m_cacheRh = -1;
    m_cacheA   = m_cacheB  = 0.0;
}

//------------------------------------------------------------------------------

void CollisionResponder::respond(VirtualPottery& a_pottery, ContactSet& a_contacts,
                                 const cVector3d& a_hip, bool a_armed, double a_tickScale,
                                 ForceModel a_forceModel, PropagationMode a_propagation,
                                 ResponseResult& a_result)
{
    a_result.force.zero();
    a_result.stiffness.set(0, 0, 0, 0, 0, 0, 0, 0, 0);
    a_result.normal.zero();
    a_result.propagated = 0;
    a_result.meanDepth  = 0.0;
    a_result.inContact  = a_contacts.touching();

    //--- 1 and 2: shape ------------------------------------------------------
    if (a_armed && !a_contacts.sectors.empty())
    {
        deform(a_pottery, a_contacts, a_tickScale);

        if (a_pottery.parameters().propagation)
        {
            a_result.propagated = (a_propagation == PROPAGATION_SEPARABLE)
                                ? propagateSeparable(a_pottery, a_contacts)
                                : propagateHulls(a_pottery, a_contacts);
        }
    }

    //--- 3: force and local stiffness ----------------------------------------
    contactForce(a_pottery, a_contacts, a_hip, a_forceModel, a_result);
}

//------------------------------------------------------------------------------

void CollisionResponder::deform(VirtualPottery& a_pottery, ContactSet& a_contacts,
                                double a_tickScale)
{
    const PotteryParameters& prm = a_pottery.parameters();

    double cEta = a_pottery.isSpinning() ? prm.viscositySpinning : prm.viscosityStopped;
    double rate = std::min(1.0, a_tickScale / std::max(1.0, cEta));

    for (size_t k = 0; k < a_contacts.sectors.size(); k++)
    {
        SectorContact& c = a_contacts.sectors[k];
        int    idx = a_pottery.index(c.row, c.col);
        double Sr  = a_pottery.radiusAt(idx);

        // Eqs. (5)-(6): towards the radius that clears the sphere; inwards
        // when contracting, outwards when expanding.
        a_pottery.setRadiusAt(idx, Sr + (c.clearRadius - Sr) * rate);
        c.delta = (float)(a_pottery.radiusAt(idx) - Sr);
    }
}

//------------------------------------------------------------------------------

bool CollisionResponder::isBoundary(const VirtualPottery& a_pottery, const ContactSet& a_contacts,
                                    const SectorContact& c) const
{
    // Past the ends of the grid there is no element: not a boundary.
    const int rows = a_pottery.rows();
    const int cols = a_pottery.cols();
    int left  = (c.col + cols - 1) % cols;
    int right = (c.col + 1) % cols;

    return ((c.row > 0)        && !a_contacts.inContact(a_pottery.index(c.row - 1, c.col))) ||
           ((c.row < rows - 1) && !a_contacts.inContact(a_pottery.index(c.row + 1, c.col))) ||
           !a_contacts.inContact(a_pottery.index(c.row, left)) ||
           !a_contacts.inContact(a_pottery.index(c.row, right));
}

//------------------------------------------------------------------------------
// Propagation along tetragonal hulls (Lee 2009)
//------------------------------------------------------------------------------

int CollisionResponder::propagateHulls(VirtualPottery& a_pottery, const ContactSet& a_contacts)
{
    const PotteryParameters& prm = a_pottery.parameters();
    const int rows = a_pottery.rows();
    const int cols = a_pottery.cols();

    if (m_propStamp.size() != (size_t)a_pottery.numElements())
    {
        m_propDelta.assign(a_pottery.numElements(), 0.0f);
        m_propStamp.assign(a_pottery.numElements(), 0);
        m_propTick = 0;
    }
    if (++m_propTick == 0)
    {
        std::fill(m_propStamp.begin(), m_propStamp.end(), 0);
        m_propTick = 1;
    }

    double maxAbs = 0.0;
    for (size_t k = 0; k < a_contacts.sectors.size(); k++)
    {
        maxAbs = std::max(maxAbs, (double)std::fabs(a_contacts.sectors[k].delta));
    }
    if (maxAbs <= 0.0) { return 0; }

    const double cl  = prm.propagationCoeff;
    const double thr = prm.propagationThreshold;
    const double dh  = a_pottery.dh();
    const double dTh = a_pottery.dTheta();
    const int maxHulls = std::max(1, std::min(prm.propagationMaxHulls, cols / 2 - 1));

    for (size_t k = 0; k < a_contacts.sectors.size(); k++)
    {
        const SectorContact& c = a_contacts.sectors[k];
        if ((c.delta == 0.0f) || !isBoundary(a_pottery, a_contacts, c)) { continue; }

        double arc     = a_pottery.radius(c.row, c.col) * dTh;
        double spacing = std::min(arc, dh);

        for (int hull = 1; hull <= maxHulls; hull++)
        {
            double nearest = cl * hull * spacing;
            if (std::fabs(c.delta) / (nearest * nearest + 1.0) < thr) { break; }

            for (int di = -hull; di <= hull; di++)
            {
                int ii = c.row + di;
                if ((ii < 0) || (ii >= rows)) { continue; }

                // Top and bottom of the hull are full rows; the sides are
                // just its two end columns.
                bool fullRow = (di == -hull) || (di == hull);
                int  stepJ   = fullRow ? 1 : 2 * hull;

                for (int dj = -hull; dj <= hull; dj += stepJ)
                {
                    int jj  = ((c.col + dj) % cols + cols) % cols;
                    int idx = a_pottery.index(ii, jj);
                    if (a_contacts.inContact(idx)) { continue; }

                    double dx = dj * arc;
                    double dy = di * dh;
                    double cd = cl * std::sqrt(dx * dx + dy * dy);

                    if (m_propStamp[idx] != m_propTick)
                    {
                        m_propStamp[idx] = m_propTick;
                        m_propDelta[idx] = 0.0f;
                        m_propTouched.push_back(idx);
                    }
                    m_propDelta[idx] += (float)(c.delta / (cd * cd + 1.0));
                }
            }
        }
    }

    // Bounded by the largest direct change: propagation smooths an edge but
    // never overtakes the contact that caused it.
    int moved = (int)m_propTouched.size();
    for (size_t k = 0; k < m_propTouched.size(); k++)
    {
        int idx = m_propTouched[k];
        double v = cClamp((double)m_propDelta[idx], -maxAbs, maxAbs);
        a_pottery.setRadiusAt(idx, a_pottery.radiusAt(idx) + v);
    }
    m_propTouched.clear();

    return moved;
}

//------------------------------------------------------------------------------
// Propagation by separable convolution (Method 2)
//
// The paper's radial kernel 1/((c_l delta)^2 + 1) is not separable. It is
// replaced by a Gaussian of the same total weight over the same window (see
// below), which is radial and factors exactly into g(d_theta) g(d_h). That
// turns the 2D spread into two 1D passes:
//
//   source   the radius changes of the boundary elements, in a band around
//            the contact region (band = contact box + kernel radius);
//   pass 1   every non-zero source cell scatters along its row;
//   pass 2   every non-zero cell of pass 1 scatters along its column.
//
// Both kernels are tabulated once per step. The extents are cut where the
// strongest change would fall below the threshold, with the same cap as the
// hull method, so the two cover the same neighbourhood.
//------------------------------------------------------------------------------

int CollisionResponder::propagateSeparable(VirtualPottery& a_pottery, const ContactSet& a_contacts)
{
    const PotteryParameters& prm = a_pottery.parameters();
    const int rows = a_pottery.rows();
    const int cols = a_pottery.cols();
    const std::vector<SectorContact>& sectors = a_contacts.sectors;

    double maxAbs = 0.0;
    for (size_t k = 0; k < sectors.size(); k++)
    {
        maxAbs = std::max(maxAbs, (double)std::fabs(sectors[k].delta));
    }
    if (maxAbs <= 0.0) { return 0; }

    const double cl  = prm.propagationCoeff;
    const double thr = prm.propagationThreshold;
    const double dh  = a_pottery.dh();
    const double arc = a_pottery.radius(sectors[0].row, sectors[0].col) * a_pottery.dTheta();
    const int    cap = std::max(1, std::min(prm.propagationMaxHulls, cols / 2 - 1));

    //--- extents: where the paper's kernel drops below the threshold ---------
    auto extent = [&](double a_spacing) -> int
    {
        int n = 0;
        while (n < cap)
        {
            double c = cl * (n + 1) * a_spacing;
            if (maxAbs / (c * c + 1.0) < thr) { break; }
            n++;
        }
        return n;
    };
    const int Rt = extent(arc);
    const int Rh = extent(dh);

    //--- separable kernel: a Gaussian of the same total weight ----------------
    // The spacing differs around and along the axis, so the product of two 1D
    // kernels of the paper's form falls off far too fast along the diagonals.
    // The Gaussian is the one kernel that is both radial and separable; its
    // width is chosen so that its weight over the window equals that of
    // 1/((c_l delta)^2 + 1) over the same window. Cached, since the window and
    // the spacings rarely change between steps.
    const double a1 = cl * arc, b1 = cl * dh;     // c_l delta per step
    if ((Rt != m_cacheRt) || (Rh != m_cacheRh) ||
        (std::fabs(a1 - m_cacheA) > 0.01 * m_cacheA) || (std::fabs(b1 - m_cacheB) > 0.01 * m_cacheB))
    {
        double radial = 0.0;
        for (int i = -Rh; i <= Rh; i++)
            for (int j = -Rt; j <= Rt; j++)
                radial += 1.0 / (1.0 + (j * a1) * (j * a1) + (i * b1) * (i * b1));

        auto gaussianWeight = [&](double a_s) -> double
        {
            double st = 0.0, sh = 0.0;
            for (int j = -Rt; j <= Rt; j++) { st += std::exp(-(j * a1) * (j * a1) / (2.0 * a_s * a_s)); }
            for (int i = -Rh; i <= Rh; i++) { sh += std::exp(-(i * b1) * (i * b1) / (2.0 * a_s * a_s)); }
            return st * sh;
        };

        double lo = 1e-3, hi = 1e3;           // bisection on the width
        for (int it = 0; it < 60; it++)
        {
            double mid = std::sqrt(lo * hi);
            if (gaussianWeight(mid) < radial) { lo = mid; } else { hi = mid; }
        }
        double s = std::sqrt(lo * hi);

        m_kernelTheta.resize(Rt + 1);
        m_kernelHeight.resize(Rh + 1);
        for (int j = 0; j <= Rt; j++) { m_kernelTheta[j]  = (float)std::exp(-(j * a1) * (j * a1) / (2.0 * s * s)); }
        for (int i = 0; i <= Rh; i++) { m_kernelHeight[i] = (float)std::exp(-(i * b1) * (i * b1) / (2.0 * s * s)); }

        m_cacheRt = Rt;  m_cacheRh = Rh;
        m_cacheA  = a1;  m_cacheB  = b1;
    }

    //--- band around the contact region ---------------------------------------
    // Columns are taken as offsets from a reference column, in [-cols/2, cols/2),
    // so a region straddling theta = 0 stays contiguous.
    const int ref = sectors[0].col;
    auto offsetOf = [&](int a_col) { return ((a_col - ref + cols + cols / 2) % cols) - cols / 2; };

    int rowMin = rows, rowMax = -1, offMin = cols, offMax = -cols;
    for (size_t k = 0; k < sectors.size(); k++)
    {
        rowMin = std::min(rowMin, sectors[k].row);
        rowMax = std::max(rowMax, sectors[k].row);
        int o = offsetOf(sectors[k].col);
        offMin = std::min(offMin, o);
        offMax = std::max(offMax, o);
    }

    const int rb0 = std::max(0, rowMin - Rh);
    const int rb1 = std::min(rows - 1, rowMax + Rh);
    const int H   = rb1 - rb0 + 1;

    int ob0 = offMin - Rt;
    int W   = offMax + Rt - ob0 + 1;
    const bool fullCircle = (W >= cols);
    if (fullCircle) { W = cols; ob0 = -(cols / 2); }

    const size_t cells = (size_t)H * W;
    m_bandSource.assign(cells, 0.0f);
    m_bandRows.assign(cells, 0.0f);
    m_bandOut.assign(cells, 0.0f);
    m_rowUsed.assign(H, 0);

    //--- sources: the boundary elements ---------------------------------------
    for (size_t k = 0; k < sectors.size(); k++)
    {
        const SectorContact& c = sectors[k];
        if ((c.delta == 0.0f) || !isBoundary(a_pottery, a_contacts, c)) { continue; }

        int r = c.row - rb0;
        int q = offsetOf(c.col) - ob0;
        m_bandSource[(size_t)r * W + q] += c.delta;
        m_rowUsed[r] = 1;
    }

    //--- pass 1: along the rows (angle) ---------------------------------------
    for (int r = 0; r < H; r++)
    {
        if (!m_rowUsed[r]) { continue; }
        const float* src = &m_bandSource[(size_t)r * W];
        float*       dst = &m_bandRows[(size_t)r * W];

        for (int q = 0; q < W; q++)
        {
            float s = src[q];
            if (s == 0.0f) { continue; }

            int lo = q - Rt, hi = q + Rt;
            if (!fullCircle)
            {
                lo = std::max(lo, 0);
                hi = std::min(hi, W - 1);
                for (int qq = lo; qq <= hi; qq++) { dst[qq] += m_kernelTheta[std::abs(qq - q)] * s; }
            }
            else
            {
                for (int qq = lo; qq <= hi; qq++) { dst[(qq + W) % W] += m_kernelTheta[std::abs(qq - q)] * s; }
            }
        }
    }

    //--- pass 2: along the columns (height) -----------------------------------
    for (int r = 0; r < H; r++)
    {
        if (!m_rowUsed[r]) { continue; }
        const float* src = &m_bandRows[(size_t)r * W];

        int lo = std::max(0, r - Rh);
        int hi = std::min(H - 1, r + Rh);
        for (int rr = lo; rr <= hi; rr++)
        {
            float  g   = m_kernelHeight[std::abs(rr - r)];
            float* dst = &m_bandOut[(size_t)rr * W];
            for (int q = 0; q < W; q++) { dst[q] += g * src[q]; }
        }
    }

    //--- apply to the elements outside the contact set ------------------------
    int moved = 0;
    for (int r = 0; r < H; r++)
    {
        int i = rb0 + r;
        const float* out = &m_bandOut[(size_t)r * W];
        for (int q = 0; q < W; q++)
        {
            float v = out[q];
            if (std::fabs(v) < thr) { continue; }

            int col = ((ref + ob0 + q) % cols + cols) % cols;
            int idx = a_pottery.index(i, col);
            if (a_contacts.inContact(idx)) { continue; }

            a_pottery.setRadiusAt(idx, a_pottery.radiusAt(idx) + cClamp((double)v, -maxAbs, maxAbs));
            moved++;
        }
    }

    return moved;
}

//------------------------------------------------------------------------------
// Force and local stiffness
//------------------------------------------------------------------------------

void CollisionResponder::contactForce(const VirtualPottery& a_pottery, const ContactSet& a_contacts,
                                      const cVector3d& a_hip, ForceModel a_model,
                                      ResponseResult& a_result) const
{
    const PotteryParameters& prm = a_pottery.parameters();
    const bool   spinning = a_pottery.isSpinning();
    const double dArea    = a_pottery.dTheta() * a_pottery.dh();   // A_i = R_i * dArea
    const double wall     = (a_contacts.mode == MODE_EXPANSION) ? a_pottery.geometry().wall : 0.0;

    cVector3d sum(0.0, 0.0, 0.0);       // sum w_i p_i f_i, world
    double    K[3][3] = { { 0 } };      // sum w_i f_i f_i^T, world
    double    sumPA = 0.0, sumA = 0.0;

    for (size_t k = 0; k < a_contacts.sectors.size(); k++)
    {
        const SectorContact& c = a_contacts.sectors[k];
        cVector3d f = a_pottery.localToWorld(c.direction);

        // Element area: the surface the sector stands for, on the side touched.
        double A = (a_pottery.radius(c.row, c.col) - wall) * dArea;
        sumPA += c.penetration * A;
        sumA  += A;

        double w = (a_model == FORCE_PENETRATION_VOLUME) ? A : 1.0;
        sum += (w * c.penetration) * f;

        for (int r = 0; r < 3; r++)
            for (int s = 0; s < 3; s++)
                K[r][s] += w * f(r) * f(s);
    }

    cVector3d force(0.0, 0.0, 0.0);
    cVector3d normal(0.0, 0.0, 0.0);

    if (!a_contacts.sectors.empty())
    {
        double coef;
        if (a_model == FORCE_PENETRATION_VOLUME)
        {
            coef = spinning ? prm.volumeResistanceSpinning : prm.volumeResistanceStopped;
        }
        else
        {
            double cv = spinning ? prm.resistanceSpinning : prm.resistanceStopped;
            coef = cv / a_contacts.sectors.size();
        }

        force = coef * sum;
        for (int r = 0; r < 3; r++)
            for (int s = 0; s < 3; s++)
                K[r][s] *= coef;

        normal = force;
        a_result.meanDepth = (sumA > 0.0) ? sumPA / sumA : 0.0;

        // Rotational friction along the surface velocity at the HIP.
        double d = std::sqrt(a_hip.x() * a_hip.x() + a_hip.y() * a_hip.y());
        if (spinning && (d > 1e-9))
        {
            cVector3d t = a_pottery.localToWorld(cVector3d(-a_hip.y() / d, a_hip.x() / d, 0.0));
            if (a_pottery.angularVelocity() < 0.0) { t.negate(); }

            double magnitude = (a_model == FORCE_PENETRATION_VOLUME)
                             ? prm.frictionCoeff * force.length()           // mu |F_n|
                             : prm.friction * a_contacts.maxPenetration;    // eq. (10)
            force += magnitude * t;
        }
    }

    // Rigid surfaces.
    for (size_t k = 0; k < a_contacts.rigid.size(); k++)
    {
        const RigidContact& c = a_contacts.rigid[k];
        cVector3d n = a_pottery.localToWorld(c.normal);
        force  += (prm.rigidStiffness * c.depth) * n;
        normal += (prm.rigidStiffness * c.depth) * n;
        for (int r = 0; r < 3; r++)
            for (int s = 0; s < 3; s++)
                K[r][s] += prm.rigidStiffness * n(r) * n(s);
    }

    // Share of the viscous resistance (see ResponseResult::dampingScale).
    if (a_model == FORCE_PENETRATION_VOLUME)
    {
        double scale = sumA / prm.dampingReferenceArea;
        for (size_t k = 0; k < a_contacts.rigid.size(); k++)
        {
            scale = std::max(scale, a_contacts.rigid[k].depth / 0.001);   // full at 1 mm
        }
        a_result.dampingScale = std::min(1.0, scale);
    }
    else
    {
        a_result.dampingScale = a_contacts.touching() ? 1.0 : 0.0;
    }

    double len = normal.length();
    a_result.normal = (len > 1e-12) ? normal / len : cVector3d(0.0, 0.0, 0.0);
    a_result.force  = force;
    a_result.stiffness.set(K[0][0], K[0][1], K[0][2],
                           K[1][0], K[1][1], K[1][2],
                           K[2][0], K[2][1], K[2][2]);
}

} // namespace vp
