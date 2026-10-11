//==============================================================================
//  Core data: one circular sector element, and the record of its contact with
//  the haptic tool.
//
//  The clay body is a surface of revolution. Exploiting that cylindrical
//  symmetry, it is cut into thin slices along the height (the cylindrical
//  elements of Han 2007) and every slice into circular sectors around the axis
//  (Lee 2008). A sector element is fully described by the cylindrical
//  coordinates of its end-point,
//
//      S = (S_r, S_theta, S_h)
//
//  and deforms along the radius only: S_theta and S_h are fixed by the grid,
//  only S_r ever changes. VirtualPottery therefore stores just the radii, in
//  one contiguous array that can be uploaded as is into the 2D alpha texture
//  of the graphics thread (Lee 2009); this struct is the element as seen
//  through VirtualPottery::element().
//
//  Coordinates are in the clay's own frame, which turns with the wheel: the
//  axis is z (the papers use y), theta is measured from +x towards +y.
//==============================================================================
#ifndef VP_CIRCULAR_SECTOR_ELEMENT_H
#define VP_CIRCULAR_SECTOR_ELEMENT_H

#include "math/CVector3d.h"

namespace vp {

//------------------------------------------------------------------------------

//! A circular sector element S = (S_r, S_theta, S_h).
struct CircularSectorElement
{
    double radius;      //!< S_r     distance of the end-point from the axis [m]
    double theta;       //!< S_theta angle of the sector [rad]
    double height;      //!< S_h     height of the slice [m]
};

//------------------------------------------------------------------------------

//! Side of the clay wall being worked (Lee 2008, Sec. 3).
enum DeformMode
{
    MODE_NONE        = 0,
    MODE_CONTRACTION = 1,   //!< from outside, towards the axis
    MODE_EXPANSION   = 2    //!< from inside a hollow body, away from the axis
};

//! How the HIP sphere touches an element (Lee 2008, Fig. 4).
enum ContactType
{
    CONTACT_POINT = 0,      //!< at the end-point; the element shortens radially
    CONTACT_LINE  = 1       //!< along its side; the element is swept tangentially
};

//------------------------------------------------------------------------------

//! One element touched by the HIP in the current haptic tick. Written by the
//! CollisionDetector, read and completed by the CollisionResponder.
struct SectorContact
{
    int         row;            //!< height index
    int         col;            //!< angle index
    ContactType type;

    //! Penetration depth p, Lee 2008 eqs. (3)/(4) [m].
    double      penetration;

    //! Outer radius at which the element would just clear the sphere:
    //! |E1 - C| when contracting, |E2 - C| + wall when expanding [m].
    //! The deformation drives S_r towards it, eqs. (5)/(6).
    double      clearRadius;

    //! Unit vector f pointing out of the penetration, eq. (8), clay frame.
    chai3d::cVector3d direction;

    //! Radius change applied this tick, filled in by the responder [m].
    float       delta;
};

//------------------------------------------------------------------------------

//! Contact with a rigid surface: the wheel head, the flat top of a solid body,
//! the rim or the inner floor of a hollow one.
struct RigidContact
{
    chai3d::cVector3d normal;   //!< unit, clay frame
    double            depth;    //!< [m]
};

} // namespace vp

#endif // VP_CIRCULAR_SECTOR_ELEMENT_H
