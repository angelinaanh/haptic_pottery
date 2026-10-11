//==============================================================================
//  Core data: the interaction methods that can be compared at run time.
//
//  Both share the circular sector element model, the collision detection and
//  the viscous deformation. They differ in how the force is formed, how often
//  the full model runs and how the deformation spreads:
//
//    Baseline   the original method: resistance averaged over the contacts
//               (Lee 2008 eq. 9 + Han 2007 eq. 5), everything in the 1 kHz
//               loop, propagation along tetragonal hulls (Lee 2009), a 10 Hz
//               low-pass filter to hide the jagged force.
//
//    Method 2   baseline +
//               - penetration-volume force  F = k_V sum_i p_i A_i f_i:
//                 elements enter the contact set with p ~ 0, so the force is
//                 continuous and independent of the resolution;
//               - multirate local stiffness: the full model runs at a lower
//                 rate in its own thread and hands the 1 kHz loop a linear
//                 model F ~ F0 - K (x - x0);
//               - propagation by separable convolution instead of hulls.
//==============================================================================
#ifndef VP_METHOD_CONFIG_H
#define VP_METHOD_CONFIG_H

namespace vp {

//------------------------------------------------------------------------------

enum PotteryMethod
{
    METHOD_BASELINE = 0,
    METHOD_2        = 1
};

const int kNumMethods = 2;

//! How the contact force is formed from the touched elements.
enum ForceModel
{
    FORCE_MEAN_PENETRATION = 0,     //!< (c_v / n) sum p_i f_i
    FORCE_PENETRATION_VOLUME = 1    //!< k_V sum p_i A_i f_i
};

//! How the deformation spreads to the neighbours of the contact region.
enum PropagationMode
{
    PROPAGATION_HULLS     = 0,      //!< tetragonal hulls, Lee 2009
    PROPAGATION_SEPARABLE = 1       //!< row pass then column pass
};

//------------------------------------------------------------------------------

struct MethodConfig
{
    const char*     name;
    ForceModel      force;
    PropagationMode propagation;

    //! Run the full model in its own thread at physicsRate and give the 1 kHz
    //! loop a local linear model; otherwise run it in the 1 kHz loop.
    bool            multirate;
    double          physicsRate;        //!< [Hz]

    //! Grid on which collision and force are computed and the clay is drawn
    //! (index into kResolutionPresets).
    int             resolution;

    //! Butterworth cut-off on the force [Hz]; 0 = none.
    double          lowPassHz;
};

inline const MethodConfig& methodConfig(PotteryMethod a_method)
{
    static const MethodConfig configs[kNumMethods] =
    {
        // name          force                     propagation            multi  rate   res  LPF
        { "Baseline",    FORCE_MEAN_PENETRATION,   PROPAGATION_HULLS,     false, 1000.0, 3, 10.0 },
        { "Method 2",    FORCE_PENETRATION_VOLUME, PROPAGATION_SEPARABLE, true,   250.0, 3,  0.0 },
    };
    return configs[a_method];
}

} // namespace vp

#endif // VP_METHOD_CONFIG_H
