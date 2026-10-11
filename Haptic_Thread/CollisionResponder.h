//==============================================================================
//  Haptic thread: collision response - the core of the method.
//
//  For the elements the CollisionDetector found in contact, every physics step:
//
//  1. Deformation (Lee 2008, eqs. 5-6). Each touched element moves a fixed
//     fraction of the way to where it would clear the sphere,
//         dS_r = (clearRadius - S_r) / c_eta,
//     c_eta being higher with the wheel stopped (firm clay, Han 2007).
//
//  2. Response force propagation (Lee 2009). The boundary of the contact region
//     passes its radius change on to the neighbours, attenuated as
//         g(delta) = 1 / ((c_l delta)^2 + 1).
//     Two implementations:
//       hulls      the paper's: every boundary element visits the tetragonal
//                  hulls around it, O(B n^2) for B boundary elements and n hulls;
//       separable  the kernel is replaced by a Gaussian of equal weight, which
//                  factors into g(d_theta) g(d_h): a sparse scatter along the
//                  rows then one along the columns, O((B + band) n), with both
//                  1D kernels tabulated.
//
//  3. Contact force, world frame, with its local stiffness K = -dF/dx:
//       mean penetration    F = (c_v / n) sum p_i f_i        (Lee 2008 eq. 9,
//                                                             Han 2007 eq. 5)
//                           friction c_r max(p) t            (Lee 2008 eq. 10)
//       penetration volume  F = k_V sum p_i A_i f_i,  A_i = R_i dtheta dh
//                           friction mu |F_n| t
//                           K = k_V sum A_i f_i f_i^T
//     plus k_rigid d n (and k_rigid n n^T) for each rigid contact.
//
//  Viscous damping of the HIP and the low-pass filter act on the force sent
//  to the device, so they are applied by the servo loop, not here.
//==============================================================================
#ifndef VP_COLLISION_RESPONDER_H
#define VP_COLLISION_RESPONDER_H

#include "CollisionDetector.h"
#include "MethodConfig.h"
#include "math/CMatrix3d.h"

#include <cstdint>
#include <vector>

namespace vp {

//------------------------------------------------------------------------------

struct ResponseResult
{
    chai3d::cVector3d force;        //!< contact force, world frame [N]
    chai3d::cMatrix3d stiffness;    //!< K = -dF/dx, world frame [N/m]
    chai3d::cVector3d normal;       //!< mean contact normal, world frame (unit)
    bool   inContact  = false;
    int    propagated = 0;          //!< elements moved by propagation
    double meanDepth  = 0.0;        //!< area-weighted penetration [m]

    //! How much of the HIP's viscous resistance applies, 0..1: 1 in contact
    //! for the mean-penetration force (Han 2007), min(1, A / A_ref) for the
    //! volume force, so it does not switch on in a step.
    double dampingScale = 0.0;
};

//==============================================================================

class CollisionResponder
{
public:

    CollisionResponder();

    //! Deforms \p a_pottery and computes the contact force for this step.
    //! \param a_hip        HIP centre, clay frame
    //! \param a_armed      false while the device withholds force: contact is
    //!                     reported but the clay is left alone
    //! \param a_tickScale  step length in nominal 1 ms ticks; keeps the clay
    //!                     equally viscous in real time at any step rate
    void respond(VirtualPottery& a_pottery, ContactSet& a_contacts,
                 const chai3d::cVector3d& a_hip, bool a_armed, double a_tickScale,
                 ForceModel a_forceModel, PropagationMode a_propagation,
                 ResponseResult& a_result);

private:

    void deform(VirtualPottery& a_pottery, ContactSet& a_contacts, double a_tickScale);

    //! Elements of the contact set whose 4-neighbourhood leaves the set.
    bool isBoundary(const VirtualPottery& a_pottery, const ContactSet& a_contacts,
                    const SectorContact& a_contact) const;

    int propagateHulls(VirtualPottery& a_pottery, const ContactSet& a_contacts);
    int propagateSeparable(VirtualPottery& a_pottery, const ContactSet& a_contacts);

    void contactForce(const VirtualPottery& a_pottery, const ContactSet& a_contacts,
                      const chai3d::cVector3d& a_hip, ForceModel a_model,
                      ResponseResult& a_result) const;

    //--- hull propagation scratch (stamped, nothing grid-sized is cleared) ---
    std::vector<float>    m_propDelta;
    std::vector<uint32_t> m_propStamp;
    std::vector<int>      m_propTouched;
    uint32_t              m_propTick;

    //--- separable propagation scratch, sized to the band around the contact -
    std::vector<float> m_bandSource;
    std::vector<float> m_bandRows;      // after the pass along the rows
    std::vector<float> m_bandOut;       // after the pass along the columns
    std::vector<char>  m_rowUsed;
    std::vector<float> m_kernelTheta;
    std::vector<float> m_kernelHeight;
    int    m_cacheRt, m_cacheRh;        // window and spacings the kernels were
    double m_cacheA,  m_cacheB;         // built for

};

} // namespace vp

#endif // VP_COLLISION_RESPONDER_H
