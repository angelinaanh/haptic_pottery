//==============================================================================
//  Graphic thread: data synchronisation between the haptic and graphic loops.
//
//  The haptic thread rewrites part of the radius grid every millisecond; the
//  graphic thread draws it some sixty times a second. Sharing the grid would
//  let the two loops step on each other, so the data crosses over here:
//
//    haptic thread    VirtualPottery --(changed rows, tryAcquire)--> staging
//    graphic thread   staging --(changed rows)--> 2D alpha texture on the GPU
//
//  The radius array maps one to one onto a 2D texture - columns are S_theta,
//  rows are S_h, the alpha channel is S_r - and only the rows changed since
//  the last frame are uploaded (Lee 2009, Sec. II-C). The haptic side never
//  waits: if the graphic thread holds the lock, the publish is skipped and the
//  changed rows carry over to the next attempt.
//
//  The same class carries the haptic loop's state (HIP, force, wheel angle,
//  rates) to the graphic thread for display.
//==============================================================================
#ifndef VP_GRAPHIC_SYNCHRONIZER_H
#define VP_GRAPHIC_SYNCHRONIZER_H

#include "MethodConfig.h"
#include "VirtualPottery.h"
#include "system/CMutex.h"

#include <cstdint>
#include <vector>

namespace vp {

//------------------------------------------------------------------------------

//! Which thread wrote a part of the frame.
enum FramePart
{
    FRAME_SERVO   = 0,      //!< 1 kHz loop: HIP, force sent, servo rate
    FRAME_PHYSICS = 1       //!< full model: contacts, depth, wheel, grid
};

//! State of the haptic side, as displayed by the graphic thread.
struct HapticFrame
{
    //--- servo part ---------------------------------------------------------
    chai3d::cVector3d hipPosition;          //!< world [m]
    chai3d::cVector3d force;                //!< sent to the device, world [N]
    bool       armed          = false;
    double     hapticRate     = 0.0;        //!< [Hz]

    //--- physics part -------------------------------------------------------
    PotteryMethod method      = METHOD_BASELINE;
    double     hipRadius      = 0.0;
    double     wheelAngle     = 0.0;        //!< [rad]
    int        contacts       = 0;
    double     meanDepth      = 0.0;        //!< area-weighted [m]
    bool       rigidContact   = false;
    double     physicsRate    = 0.0;        //!< [Hz]
    int        rows           = 0;
    int        cols           = 0;
};

//==============================================================================

class GraphicSynchronizer
{
public:

    GraphicSynchronizer();

    //--------------------------------------------------------------------------
    // Haptic thread
    //--------------------------------------------------------------------------

    //! Copies the pottery's changed rows to the staging buffer unless the
    //! graphic thread is reading it. Clears the pottery's change range on
    //! success.
    bool publishShape(VirtualPottery& a_pottery);

    //! Replaces the whole shape after a change of resolution or geometry.
    void publishAll(VirtualPottery& a_pottery);

    //! Copies one thread's part of \p a_frame; the other part is kept.
    void publishFrame(const HapticFrame& a_frame, FramePart a_part);

    //--------------------------------------------------------------------------
    // Graphic thread (GL context current)
    //--------------------------------------------------------------------------

    //! Brings the radius texture up to date. Returns false if nothing changed.
    bool synchronize();

    HapticFrame frame();

    unsigned int texture() const { return m_texture; }
    int rows() const { return m_rows; }
    int cols() const { return m_cols; }
    const PotteryGeometry& geometry() const { return m_geometry; }

    //! Changes whenever the texture is reallocated (size or geometry).
    uint64_t topology() const { return m_topology; }

private:

    //--- shared, under m_shapeLock ------------------------------------------
    chai3d::cMutex     m_shapeLock;
    int                m_stageRows;
    int                m_stageCols;
    PotteryGeometry    m_stageGeometry;
    uint64_t           m_stageTopology;
    std::vector<float> m_stage;
    int                m_stageDirtyMin;
    int                m_stageDirtyMax;

    //--- shared, under m_frameLock ------------------------------------------
    chai3d::cMutex     m_frameLock;
    HapticFrame        m_frame;

    //--- graphic thread only ------------------------------------------------
    int                m_rows;
    int                m_cols;
    PotteryGeometry    m_geometry;
    uint64_t           m_topology;
    std::vector<float> m_local;
    unsigned int       m_texture;
};

} // namespace vp

#endif // VP_GRAPHIC_SYNCHRONIZER_H
