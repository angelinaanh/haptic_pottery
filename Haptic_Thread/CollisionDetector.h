//==============================================================================
//  Haptic thread: collision detection between the HIP sphere and the clay.
//
//  Broad phase (Lee 2008, step 1). Only elements inside the sphere's extent in
//  height and in angle can touch it:
//
//      S_h     in [H_y - H_r, H_y + H_r]                                   (1)
//      S_theta in [theta_H - asin(H_r/d), theta_H + asin(H_r/d)]           (2)
//
//  Both ranges map to index ranges of the grid in O(1), so the cost depends on
//  the size of the tool, not on the size of the model.
//
//  Narrow phase (steps 2 and 3). At the element's height the sphere's cross-
//  section is a circle of radius r = sqrt(H_r^2 - (S_h - H_y)^2). The line from
//  the axis C through the element meets it at E1 and E2, with middle point M.
//  The element is then in
//
//      point collision  if S_r < d    p = S_r - |E1 - C|
//      line collision   otherwise     p = r - |H - M|                      (3)
//
//  (types reversed, E2 instead of E1, for the expansion mode, eq. 4), and in
//  contact when p > 0.
//
//  Side of the wall. A hollow body is worked from outside (contraction) or
//  from inside (expansion). The side is decided when a contact begins and kept
//  until it ends, so a HIP inside a thin wall cannot flip between the two.
//
//  Rigid surfaces. Neither paper handles the top of the body; the HIP resting
//  on it would be pushed sideways by the radial model. The flat top of a solid
//  body, the rim and inner floor of a hollow one and the wheel head are
//  detected as rigid contacts instead.
//==============================================================================
#ifndef VP_COLLISION_DETECTOR_H
#define VP_COLLISION_DETECTOR_H

#include "VirtualPottery.h"

#include <cstdint>
#include <vector>

namespace vp {

//------------------------------------------------------------------------------

//! Everything the HIP touches in one tick. Reused from tick to tick.
struct ContactSet
{
    std::vector<SectorContact> sectors;
    std::vector<RigidContact>  rigid;

    DeformMode mode = MODE_NONE;

    double maxPenetration  = 0.0;   //!< [m]
    double meanPenetration = 0.0;   //!< [m]

    //! Element k is in contact this tick iff stamp[k] == tick. Lets the
    //! responder find the boundary of the contact region without clearing a
    //! grid-sized array every tick.
    std::vector<uint32_t> stamp;
    uint32_t tick = 0;

    bool inContact(int a_index) const { return stamp[a_index] == tick; }
    bool touching() const { return !sectors.empty() || !rigid.empty(); }
};

//==============================================================================

class CollisionDetector
{
public:

    CollisionDetector();

    //! Fills \p a_contacts for the HIP at \p a_hip (clay frame).
    void detect(const VirtualPottery& a_pottery, const chai3d::cVector3d& a_hip,
                double a_hipRadius, ContactSet& a_contacts);

private:

    //! Step 1: index ranges of the candidate elements. False if none.
    bool broadPhase(const VirtualPottery& a_pottery, const chai3d::cVector3d& a_hip,
                    double a_hipRadius, double a_d, double a_thetaH, DeformMode a_mode,
                    int& a_row0, int& a_row1, int& a_col0, int& a_numCols) const;

    //! Steps 2 and 3 for one element. True and \p a_contact filled if touching.
    bool narrowPhase(const VirtualPottery& a_pottery, const chai3d::cVector3d& a_hip,
                     double a_d, double a_sectionRadius, int a_row, int a_col,
                     DeformMode a_mode, SectorContact& a_contact) const;

    //! Contraction or expansion, for a contact that is just beginning.
    DeformMode chooseSide(const VirtualPottery& a_pottery, const chai3d::cVector3d& a_hip,
                          double a_d, double a_thetaH) const;

    //! Rigid surfaces. Returns true when the HIP is above the top of the body
    //! and the radial elements must not be tested.
    bool rigidContacts(const VirtualPottery& a_pottery, const chai3d::cVector3d& a_hip,
                       double a_hipRadius, double a_d, double a_thetaH,
                       ContactSet& a_contacts) const;

    bool       m_wasInContact;
    DeformMode m_lastMode;
};

} // namespace vp

#endif // VP_COLLISION_DETECTOR_H
