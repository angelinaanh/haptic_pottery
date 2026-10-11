//==============================================================================
//  Core data: the virtual pottery.
//
//  Owns the 2D grid of circular sector elements (rows along the height,
//  columns around the axis), the potter's wheel the clay turns on, and the
//  physical constants of the clay.
//
//  The method combines the three papers:
//
//    Han 2007 (CEM)   cylindrical symmetry: the clay as a stack of thin slices;
//                     viscous resistance of the HIP; firmer clay with the wheel
//                     stopped; the force averaged over the contacts.
//    Lee 2008 (CSEM)  every slice cut into circular sectors deformed radially,
//                     giving local deformation and a hollow inside.
//    Lee 2009         response force propagation between neighbouring
//                     elements; radii synchronised to the GPU as a texture.
//
//  Threading: the haptic thread is the only one that reads or writes an
//  instance. The graphics thread receives copies of the changed rows through
//  GraphicSynchronizer and never touches this object.
//==============================================================================
#ifndef VP_VIRTUAL_POTTERY_H
#define VP_VIRTUAL_POTTERY_H

#include "CircularSectorElement.h"
#include "math/CMaths.h"

#include <vector>

namespace vp {

//------------------------------------------------------------------------------
// Scene dimensions [m]. World frame: z up, camera on +x.
//------------------------------------------------------------------------------

const double kWheelTopZ         = -0.045;   //!< the clay stands on this plane
const double kWheelRadius       =  0.075;
const double kWheelThickness    =  0.012;

const double kClayHeight        =  0.090;   //!< row 0 to the top row
const double kClayInitialRadius =  0.030;
const double kClayMinRadius     =  0.004;
const double kClayMaxRadius     =  0.068;

const double kWallThickness     =  0.006;   //!< hollow body
const double kFloorThickness    =  0.008;

//! Rate of the haptic loop the deformation constants refer to [Hz].
const double kHapticRate        = 1000.0;

//------------------------------------------------------------------------------
// Grid resolutions (height x angle). Entries 1-4 are those of Lee 2008, Fig. 7.
// Start low and raise until the haptic loop no longer holds 1 kHz.
//------------------------------------------------------------------------------

const int kResolutionPresets[][2] = { {100, 90}, {200, 180}, {200, 360}, {400, 360},
                                      {800, 360}, {1600, 720}, {3200, 1440} };
const int kNumResolutionPresets   = sizeof(kResolutionPresets) / sizeof(kResolutionPresets[0]);
const int kDefaultResolution      = 1;

//------------------------------------------------------------------------------

//! Geometry of the body that is not stored per element.
struct PotteryGeometry
{
    double baseZ  = kWheelTopZ;
    double height = kClayHeight;
    bool   hollow = false;                          //!< open top, inner wall
    double wall   = kWallThickness;                 //!< inner radius = S_r - wall
    double floorZ = kWheelTopZ + kFloorThickness;   //!< inner floor
};

//------------------------------------------------------------------------------

//! Physical constants of the clay and of the interaction.
struct PotteryParameters
{
    //--- tool and wheel ------------------------------------------------------
    double hipRadius = 0.006;           //!< HIP sphere radius [m]
    double wheelSpeed = 1.5;            //!< [rev/s]

    //--- deformation: pseudo viscosity c_eta, Lee 2008 eq. (6) [ticks] ------
    //! The fraction 1/c_eta of the gap to the sphere is closed per 1 ms tick.
    //! Higher with the wheel stopped: firm, hard-to-deform clay (Han 2007).
    double viscositySpinning = 150.0;
    double viscosityStopped  = 450.0;

    //--- resistance c_v, Lee 2008 eq. (9), averaged as Han 2007 eq. (5) [N/m]
    double resistanceSpinning = 400.0;
    double resistanceStopped  = 800.0;

    //--- resistance k_V of the penetration-volume force [N/m^3] -------------
    //! F = k_V sum_i p_i A_i f_i. Chosen to match the mean-penetration force
    //! at 2 mm depth with a 6 mm HIP; the force then stiffens with depth as a
    //! sphere pressed into a surface does.
    double volumeResistanceSpinning = 6.0e6;
    double volumeResistanceStopped  = 1.2e7;

    //--- rotational friction c_r, Lee 2008 eq. (10) [N/m] -------------------
    double friction = 100.0;

    //--- friction coefficient for the volume force: F_r = mu |F_n| t --------
    //! Proportional to the normal force, hence as continuous as it is.
    double frictionCoeff = 0.35;

    //--- viscous resistance of the HIP c2, Han 2007 eq. (6) [N.s/m] ---------
    double damping = 1.0;

    //! Method 2 scales the viscous resistance by the contact area,
    //! min(1, A / A_ref), so that it grows from zero as contact begins instead
    //! of switching on at full strength. A_ref: the patch of a 6 mm tool
    //! pressed 2 mm into the clay [m^2].
    double dampingReferenceArea = 6.0e-5;

    //--- rigid surfaces: wheel head, solid top, rim, floor [N/m] ------------
    double rigidStiffness = 500.0;

    //--- Butterworth low-pass cut-off on the total force, Lee 2008 [Hz] -----
    double lowPassHz = 10.0;            //!< 0 disables the filter

    //--- response force propagation, Lee 2009 -------------------------------
    bool   propagation          = true;
    double propagationCoeff     = 1500.0;   //!< c_l [1/m]
    double propagationThreshold = 2.0e-8;   //!< stop below this change [m]
    int    propagationMaxHulls  = 10;       //!< bounds the cost per tick
};

//==============================================================================

class VirtualPottery
{
public:

    //! Builds a solid cylinder on a grid of the given size.
    VirtualPottery(int a_rows, int a_cols);

    //--------------------------------------------------------------------------
    // Shape
    //--------------------------------------------------------------------------

    //! Restores the initial solid cylinder (keeps hollow/solid).
    void reset(double a_radius = kClayInitialRadius);

    //! Changes the grid resolution, resampling the current shape onto it.
    void resize(int a_rows, int a_cols);

    //! Opens the body (hollow, worked from inside too) or closes it.
    void setHollow(bool a_hollow);

    //--------------------------------------------------------------------------
    // Elements
    //--------------------------------------------------------------------------

    int rows() const { return m_rows; }
    int cols() const { return m_cols; }
    int numElements() const { return m_rows * m_cols; }
    int index(int a_row, int a_col) const { return a_row * m_cols + a_col; }

    double dh() const     { return m_dh; }            //!< height spacing [m]
    double dTheta() const { return m_dTheta; }        //!< angular spacing [rad]

    double rowHeight(int a_row) const { return m_geometry.baseZ + a_row * m_dh; }
    double topHeight() const          { return rowHeight(m_rows - 1); }
    double cosTheta(int a_col) const  { return m_cos[a_col]; }
    double sinTheta(int a_col) const  { return m_sin[a_col]; }

    CircularSectorElement element(int a_row, int a_col) const;

    float radius(int a_row, int a_col) const { return m_radius[index(a_row, a_col)]; }
    float radiusAt(int a_index) const        { return m_radius[a_index]; }

    //! Sets S_r, clamped to the allowed range, and marks the row changed.
    void setRadiusAt(int a_index, double a_radius);

    //! S_r at an arbitrary height and angle, bilinear.
    double sampleRadius(double a_z, double a_theta) const;

    //! Clamps an outer radius to the physically allowed range.
    double clampRadius(double a_radius) const;

    const float* data() const { return m_radius.data(); }
    const PotteryGeometry& geometry() const { return m_geometry; }

    //--------------------------------------------------------------------------
    // Physical constants
    //--------------------------------------------------------------------------

    PotteryParameters&       parameters()       { return m_params; }
    const PotteryParameters& parameters() const { return m_params; }

    //--------------------------------------------------------------------------
    // Potter's wheel
    //--------------------------------------------------------------------------

    //! Turns the wheel by one time step.
    void advanceWheel(double a_dt, bool a_spinning);

    double wheelAngle() const      { return m_wheelAngle; }      //!< [rad]
    double angularVelocity() const { return m_omega; }           //!< [rad/s]
    bool   isSpinning() const      { return m_omega != 0.0; }

    //! World frame <-> clay frame (rotation about z by the wheel angle).
    chai3d::cVector3d worldToLocal(const chai3d::cVector3d& a_v) const;
    chai3d::cVector3d localToWorld(const chai3d::cVector3d& a_v) const;

    //--------------------------------------------------------------------------
    // Changed rows, for the graphics synchronisation
    //--------------------------------------------------------------------------

    bool isDirty() const  { return m_dirtyMin <= m_dirtyMax; }
    int  dirtyMin() const { return m_dirtyMin; }
    int  dirtyMax() const { return m_dirtyMax; }
    void clearDirty()     { m_dirtyMin = m_rows; m_dirtyMax = -1; }

private:

    void buildTables();
    void markAllDirty() { m_dirtyMin = 0; m_dirtyMax = m_rows - 1; }

    int    m_rows;
    int    m_cols;
    double m_dh;
    double m_dTheta;

    PotteryGeometry   m_geometry;
    PotteryParameters m_params;

    //! S_r of every element, row-major (row = height, column = angle).
    std::vector<float>  m_radius;

    //! cos/sin of S_theta per column.
    std::vector<double> m_cos;
    std::vector<double> m_sin;

    double m_wheelAngle;
    double m_omega;

    int m_dirtyMin;
    int m_dirtyMax;
};

} // namespace vp

#endif // VP_VIRTUAL_POTTERY_H
