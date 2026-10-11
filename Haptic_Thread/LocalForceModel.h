//==============================================================================
//  Haptic thread: multirate local stiffness model.
//
//  The full clay model (collision detection, deformation, propagation) costs
//  more the finer the grid; the haptic device needs a fresh force every
//  millisecond regardless. Following the multirate scheme for deformable
//  haptics (local stiffness matrix around the contact, IEEE TIE 2019), the two
//  are decoupled:
//
//    physics thread (e.g. 250 Hz)   runs the full model at the HIP position x0
//                                   and linearises the contact force there:
//                                     F0 = F(x0),   K = -dF/dx at x0
//    servo thread (1 kHz)           evaluates  F(x) ~ F0 - K (x - x0)
//                                   for the HIP position of every tick
//
//  For the penetration-volume force F = k_V sum_i p_i A_i f_i, moving the HIP
//  by dx changes each penetration by about -f_i . dx, so
//
//      K = k_V sum_i A_i f_i f_i^T   (+ k_rigid n n^T for rigid contacts)
//
//  a symmetric positive semi-definite matrix: the local model is a spring
//  that is stiff along the contact normals and soft along the surface.
//
//  The file also holds the low-pass filter applied to the force sent to the
//  device, and the lock-free-for-the-servo exchange of the model.
//==============================================================================
#ifndef VP_LOCAL_FORCE_MODEL_H
#define VP_LOCAL_FORCE_MODEL_H

#include "math/CMatrix3d.h"
#include "math/CVector3d.h"
#include "system/CMutex.h"

#include <cstdint>

namespace vp {

//------------------------------------------------------------------------------

//! Linearised contact force around x0, world frame, plus the statistics of
//! the physics step that produced it.
struct LocalForceModel
{
    chai3d::cVector3d x0;           //!< HIP position the model was built at [m]
    chai3d::cVector3d F0;           //!< contact force at x0 [N]
    chai3d::cMatrix3d K;            //!< local stiffness, F = F0 - K (x - x0) [N/m]
    chai3d::cVector3d normal;       //!< mean contact normal (unit)
    bool              inContact = false;
    double            dampingScale = 0.0;   //!< share of the HIP damping, 0..1

    // Statistics of the physics step.
    int      contacts       = 0;
    double   maxPenetration = 0.0;  //!< [m]
    double   meanDepth      = 0.0;  //!< area-weighted penetration [m]
    double   physicsUs      = 0.0;  //!< duration of the physics step [us]
    uint64_t sequence       = 0;    //!< increments with every physics step

    LocalForceModel() { K.set(0, 0, 0, 0, 0, 0, 0, 0, 0); }

    //! Force at \p a_x. The displacement is bounded so the model is not
    //! extrapolated far from where it is valid, and the force is never allowed
    //! to pull the HIP into the clay.
    chai3d::cVector3d evaluate(const chai3d::cVector3d& a_x) const;
};

//------------------------------------------------------------------------------

//! Servo-side holder of the local model. A new model from the physics thread
//! does not replace the old one at once: the force cross-fades from the old
//! model to the new one over one physics period. Without this the force
//! steps every time a model arrives - most visibly when contact begins,
//! where the old model has none - at the cost of at most one physics period
//! of extra delay, far less than the 10 Hz filter it replaces.
class LocalModelBlend
{
public:

    LocalModelBlend() : m_start(0.0), m_period(0.004) {}

    //! A new model arrived at time \p a_now; blend over \p a_period seconds.
    void update(const LocalForceModel& a_model, double a_now, double a_period);

    //! Starts over with \p a_model alone (e.g. when the method changes).
    void reset(const LocalForceModel& a_model);

    //! Force at \p a_x at time \p a_now.
    chai3d::cVector3d evaluate(const chai3d::cVector3d& a_x, double a_now) const;

    //! True while either model of the blend touches the clay.
    bool inContact(double a_now) const;

    //! Blended share of the HIP damping.
    double dampingScale(double a_now) const;

    //! The newest model.
    const LocalForceModel& current() const { return m_current; }

private:

    double weight(double a_now) const;

    LocalForceModel m_previous;
    LocalForceModel m_current;
    double          m_start;
    double          m_period;
};

//------------------------------------------------------------------------------

//! Hands the newest model from the physics thread to the servo thread. The
//! servo side only ever tries the lock, so it never waits.
class LocalModelExchange
{
public:

    LocalModelExchange() : m_fresh(false) {}

    //! Physics thread.
    void publish(const LocalForceModel& a_model);

    //! Servo thread: copies the newest model into \p a_model if one could be
    //! read this time; otherwise leaves it as it was.
    bool tryFetch(LocalForceModel& a_model);

private:

    chai3d::cMutex  m_lock;
    LocalForceModel m_model;
    bool            m_fresh;
};

//------------------------------------------------------------------------------

//! Second-order Butterworth low-pass filter on a 3D vector (Lee 2008, Sec. 5),
//! bilinear transform with frequency pre-warping.
class LowPassFilter
{
public:

    LowPassFilter();
    void design(double a_cutoffHz, double a_sampleHz);
    void reset(const chai3d::cVector3d& a_value);
    chai3d::cVector3d process(const chai3d::cVector3d& a_x);
    double cutoff() const { return m_cutoff; }

private:

    double m_cutoff;
    double m_b0, m_b1, m_b2, m_a1, m_a2;
    chai3d::cVector3d m_x1, m_x2, m_y1, m_y2;
};

} // namespace vp

#endif // VP_LOCAL_FORCE_MODEL_H
