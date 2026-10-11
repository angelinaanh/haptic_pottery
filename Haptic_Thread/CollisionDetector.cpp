//==============================================================================
//  Haptic thread: collision detection. See CollisionDetector.h.
//==============================================================================
#include "CollisionDetector.h"

#include <algorithm>
#include <cmath>

using namespace chai3d;

namespace vp {

//------------------------------------------------------------------------------

CollisionDetector::CollisionDetector()
{
    m_wasInContact = false;
    m_lastMode     = MODE_NONE;
}

//------------------------------------------------------------------------------

void CollisionDetector::detect(const VirtualPottery& a_pottery, const cVector3d& a_hip,
                               double a_hipRadius, ContactSet& a_contacts)
{
    //--------------------------------------------------------------------------
    // New tick: stamps are compared for equality only, so they need no
    // clearing except on wrap-around or a change of grid size.
    //--------------------------------------------------------------------------
    a_contacts.sectors.clear();
    a_contacts.rigid.clear();
    a_contacts.mode            = MODE_NONE;
    a_contacts.maxPenetration  = 0.0;
    a_contacts.meanPenetration = 0.0;

    if (a_contacts.stamp.size() != (size_t)a_pottery.numElements())
    {
        a_contacts.stamp.assign(a_pottery.numElements(), 0);
        a_contacts.tick = 0;
    }
    if (++a_contacts.tick == 0)
    {
        std::fill(a_contacts.stamp.begin(), a_contacts.stamp.end(), 0);
        a_contacts.tick = 1;
    }

    // d: distance from the axis; theta_H: angle of the HIP.
    double d      = std::sqrt(a_hip.x() * a_hip.x() + a_hip.y() * a_hip.y());
    double thetaH = std::atan2(a_hip.y(), a_hip.x());
    if (thetaH < 0.0) { thetaH += 2.0 * C_PI; }

    if (rigidContacts(a_pottery, a_hip, a_hipRadius, d, thetaH, a_contacts))
    {
        m_wasInContact = false;
        return;
    }

    DeformMode mode = m_wasInContact ? m_lastMode : chooseSide(a_pottery, a_hip, d, thetaH);

    // Inner floor of a hollow body, reachable only from inside.
    const PotteryGeometry& g = a_pottery.geometry();
    if (mode == MODE_EXPANSION)
    {
        double inner = a_pottery.sampleRadius(g.floorZ, thetaH) - g.wall;
        double depth = g.floorZ - (a_hip.z() - a_hipRadius);
        if ((depth > 0.0) && (d < inner))
        {
            RigidContact c = { cVector3d(0.0, 0.0, 1.0), depth };
            a_contacts.rigid.push_back(c);
        }
    }

    //--------------------------------------------------------------------------
    // Broad phase
    //--------------------------------------------------------------------------
    int row0, row1, col0, numCols;
    if (!broadPhase(a_pottery, a_hip, a_hipRadius, d, thetaH, mode, row0, row1, col0, numCols))
    {
        m_wasInContact = false;
        return;
    }

    //--------------------------------------------------------------------------
    // Narrow phase over the candidates
    //--------------------------------------------------------------------------
    const int cols = a_pottery.cols();
    double sum = 0.0;

    for (int i = row0; i <= row1; i++)
    {
        double dz = a_pottery.rowHeight(i) - a_hip.z();
        double r2 = a_hipRadius * a_hipRadius - dz * dz;
        if (r2 <= 0.0) { continue; }
        double r = std::sqrt(r2);       // cross-section of the sphere at S_h

        for (int k = 0; k < numCols; k++)
        {
            int j = (col0 + k) % cols;
            if (j < 0) { j += cols; }

            SectorContact c;
            if (narrowPhase(a_pottery, a_hip, d, r, i, j, mode, c))
            {
                a_contacts.stamp[a_pottery.index(i, j)] = a_contacts.tick;
                a_contacts.sectors.push_back(c);
                a_contacts.maxPenetration = std::max(a_contacts.maxPenetration, c.penetration);
                sum += c.penetration;
            }
        }
    }

    m_wasInContact = !a_contacts.sectors.empty();
    m_lastMode     = mode;

    if (m_wasInContact)
    {
        a_contacts.mode            = mode;
        a_contacts.meanPenetration = sum / a_contacts.sectors.size();
    }
}

//------------------------------------------------------------------------------

bool CollisionDetector::broadPhase(const VirtualPottery& a_pottery, const cVector3d& a_hip,
                                   double a_hipRadius, double a_d, double a_thetaH,
                                   DeformMode a_mode, int& a_row0, int& a_row1,
                                   int& a_col0, int& a_numCols) const
{
    const PotteryGeometry& g = a_pottery.geometry();
    const double dh = a_pottery.dh();

    // (1) height range -> rows.
    a_row0 = (int)std::ceil ((a_hip.z() - a_hipRadius - g.baseZ) / dh);
    a_row1 = (int)std::floor((a_hip.z() + a_hipRadius - g.baseZ) / dh);
    a_row0 = std::max(a_row0, 0);
    a_row1 = std::min(a_row1, a_pottery.rows() - 1);

    // Below the floor of a hollow body there is no inner wall to push.
    if (a_mode == MODE_EXPANSION)
    {
        a_row0 = std::max(a_row0, (int)std::floor((g.floorZ - g.baseZ) / dh) + 1);
    }

    if (a_row0 > a_row1) { return false; }

    // (2) angle range -> columns. asin(H_r/d) is the exact half-angle the
    // sphere subtends; the paper's H_r/d is its small-angle approximation.
    // With the axis inside the sphere every column is a candidate.
    a_col0    = 0;
    a_numCols = a_pottery.cols();
    if (a_d > a_hipRadius)
    {
        double half = std::asin(a_hipRadius / a_d);
        double dTh  = a_pottery.dTheta();
        double jc   = a_thetaH / dTh;
        a_col0 = (int)std::floor(jc - half / dTh);
        int col1 = (int)std::ceil(jc + half / dTh);
        a_numCols = std::min(col1 - a_col0 + 1, a_pottery.cols());
    }

    return true;
}

//------------------------------------------------------------------------------

bool CollisionDetector::narrowPhase(const VirtualPottery& a_pottery, const cVector3d& a_hip,
                                    double a_d, double a_sectionRadius, int a_row, int a_col,
                                    DeformMode a_mode, SectorContact& a_contact) const
{
    const double ux = a_pottery.cosTheta(a_col);
    const double uy = a_pottery.sinTheta(a_col);
    const double z  = a_pottery.rowHeight(a_row);
    const double r  = a_sectionRadius;

    // Intersections of the line C -> S with the cross-section:
    // t = tm -/+ sqrt(r^2 - dist^2), tm being the projection of H on the line.
    double tm    = ux * a_hip.x() + uy * a_hip.y();
    double dist2 = std::max(0.0, a_d * a_d - tm * tm);
    double disc  = r * r - dist2;
    if (disc <= 0.0) { return false; }          // the line misses the circle

    double sq = std::sqrt(disc);
    double t1 = tm - sq;                        // |E1 - C|
    double t2 = tm + sq;                        // |E2 - C|
    if (t2 <= 0.0) { return false; }            // circle behind the axis

    const double Sr   = a_pottery.radiusAt(a_pottery.index(a_row, a_col));
    const double wall = a_pottery.geometry().wall;

    cVector3d fallback;

    if (a_mode != MODE_EXPANSION)
    {
        //--- contraction: the clay is the segment [0, S_r] -------------------
        // E1 behind the axis: the section covers the axis, and the whole
        // element lies inside the sphere.
        double e1 = std::max(t1, 0.0);
        if (Sr <= e1) { return false; }

        a_contact.clearRadius = e1;

        if (Sr < a_d)
        {
            a_contact.type        = CONTACT_POINT;
            a_contact.penetration = Sr - e1;
            a_contact.direction   = a_hip - cVector3d(e1 * ux, e1 * uy, z);
        }
        else
        {
            a_contact.type        = CONTACT_LINE;
            a_contact.penetration = r - std::sqrt(dist2);
            a_contact.direction   = a_hip - cVector3d(tm * ux, tm * uy, z);
        }
        fallback.set(ux, uy, 0.0);
    }
    else
    {
        //--- expansion: the clay lies beyond the inner radius ----------------
        double Ri = Sr - wall;
        if (t2 <= Ri) { return false; }

        a_contact.clearRadius = t2 + wall;

        if (Ri > a_d)
        {
            a_contact.type        = CONTACT_POINT;
            a_contact.penetration = t2 - Ri;
            a_contact.direction   = a_hip - cVector3d(t2 * ux, t2 * uy, z);
        }
        else
        {
            a_contact.type        = CONTACT_LINE;
            a_contact.penetration = r - std::sqrt(dist2);
            a_contact.direction   = a_hip - cVector3d(tm * ux, tm * uy, z);
        }
        fallback.set(-ux, -uy, 0.0);
    }

    // Eq. (8): unit vector out of the penetration.
    double len = a_contact.direction.length();
    a_contact.direction = (len > 1e-12) ? a_contact.direction / len : fallback;

    a_contact.row   = a_row;
    a_contact.col   = a_col;
    a_contact.delta = 0.0f;
    return true;
}

//------------------------------------------------------------------------------

DeformMode CollisionDetector::chooseSide(const VirtualPottery& a_pottery, const cVector3d& a_hip,
                                         double a_d, double a_thetaH) const
{
    const PotteryGeometry& g = a_pottery.geometry();
    if (!g.hollow) { return MODE_CONTRACTION; }

    // Inside when nearer the axis than the middle of the wall, above the floor.
    double outer  = a_pottery.sampleRadius(a_hip.z(), a_thetaH);
    bool   inside = (a_d < outer - 0.5 * g.wall) && (a_hip.z() > g.floorZ);

    return inside ? MODE_EXPANSION : MODE_CONTRACTION;
}

//------------------------------------------------------------------------------

bool CollisionDetector::rigidContacts(const VirtualPottery& a_pottery, const cVector3d& a_hip,
                                      double a_hipRadius, double a_d, double a_thetaH,
                                      ContactSet& a_contacts) const
{
    const cVector3d up(0.0, 0.0, 1.0);

    //--- wheel head ---------------------------------------------------------
    double plate = kWheelTopZ - (a_hip.z() - a_hipRadius);
    if ((plate > 0.0) && (a_d < kWheelRadius) && (a_hip.z() > kWheelTopZ - kWheelThickness))
    {
        RigidContact c = { up, plate };
        a_contacts.rigid.push_back(c);
    }

    //--- top of the body ----------------------------------------------------
    const PotteryGeometry& g = a_pottery.geometry();
    double top = a_pottery.topHeight();

    if (a_hip.z() <= top) { return false; }                 // radial model below the top

    double topRadius = a_pottery.sampleRadius(top, a_thetaH);
    if (a_d > topRadius) { return false; }                  // beside the body
    if (g.hollow && (a_d < topRadius - g.wall)) { return false; }   // over the opening

    // Above the flat top of a solid body, or above the rim of a hollow one.
    double depth = top - (a_hip.z() - a_hipRadius);
    if (depth > 0.0)
    {
        RigidContact c = { up, depth };
        a_contacts.rigid.push_back(c);
    }
    return true;
}

} // namespace vp
