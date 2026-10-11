//==============================================================================
//  Graphic thread: data synchronisation. See GraphicSynchronizer.h.
//==============================================================================
#include "GraphicSynchronizer.h"
#include "chai3d.h"

#include <algorithm>
#include <cstring>

using namespace chai3d;

namespace vp {

//------------------------------------------------------------------------------

GraphicSynchronizer::GraphicSynchronizer()
{
    m_stageRows     = 0;
    m_stageCols     = 0;
    m_stageTopology = 0;
    m_stageDirtyMin = 0;
    m_stageDirtyMax = -1;

    m_rows       = 0;
    m_cols       = 0;
    m_topology   = 0;
    m_texture    = 0;
}

//------------------------------------------------------------------------------
// Haptic thread
//------------------------------------------------------------------------------

bool GraphicSynchronizer::publishShape(VirtualPottery& a_pottery)
{
    if (!a_pottery.isDirty()) { return true; }
    if (!m_shapeLock.tryAcquire()) { return false; }

    // A grid of another size must go through publishAll first.
    if ((a_pottery.rows() != m_stageRows) || (a_pottery.cols() != m_stageCols))
    {
        m_shapeLock.release();
        return false;
    }

    int r0 = a_pottery.dirtyMin();
    int r1 = a_pottery.dirtyMax();
    std::memcpy(&m_stage[(size_t)r0 * m_stageCols],
                a_pottery.data() + (size_t)r0 * m_stageCols,
                sizeof(float) * (size_t)(r1 - r0 + 1) * m_stageCols);

    m_stageDirtyMin = std::min(m_stageDirtyMin, r0);
    m_stageDirtyMax = std::max(m_stageDirtyMax, r1);

    m_shapeLock.release();

    a_pottery.clearDirty();
    return true;
}

//------------------------------------------------------------------------------

void GraphicSynchronizer::publishAll(VirtualPottery& a_pottery)
{
    m_shapeLock.acquire();

    m_stageRows     = a_pottery.rows();
    m_stageCols     = a_pottery.cols();
    m_stageGeometry = a_pottery.geometry();
    m_stage.assign(a_pottery.data(), a_pottery.data() + a_pottery.numElements());
    m_stageDirtyMin = 0;
    m_stageDirtyMax = m_stageRows - 1;
    m_stageTopology++;

    m_shapeLock.release();

    a_pottery.clearDirty();
}

//------------------------------------------------------------------------------

void GraphicSynchronizer::publishFrame(const HapticFrame& a_frame, FramePart a_part)
{
    if (!m_frameLock.tryAcquire()) { return; }

    HapticFrame& f = m_frame;
    if (a_part == FRAME_SERVO)
    {
        f.hipPosition = a_frame.hipPosition;
        f.force       = a_frame.force;
        f.armed       = a_frame.armed;
        f.hapticRate  = a_frame.hapticRate;
    }
    else
    {
        HapticFrame servo = f;
        f = a_frame;
        f.hipPosition = servo.hipPosition;
        f.force       = servo.force;
        f.armed       = servo.armed;
        f.hapticRate  = servo.hapticRate;
    }

    m_frameLock.release();
}

//------------------------------------------------------------------------------
// Graphic thread
//------------------------------------------------------------------------------

HapticFrame GraphicSynchronizer::frame()
{
    m_frameLock.acquire();
    HapticFrame f = m_frame;
    m_frameLock.release();
    return f;
}

//------------------------------------------------------------------------------

bool GraphicSynchronizer::synchronize()
{
    if (m_texture == 0) { glGenTextures(1, &m_texture); }

    //--------------------------------------------------------------------------
    // Pull the changed rows. The lock is held for a memcpy only; the upload
    // happens after it is released.
    //--------------------------------------------------------------------------
    m_shapeLock.acquire();

    if (m_stageTopology == 0)
    {
        m_shapeLock.release();
        return false;
    }

    bool full = (m_stageTopology != m_topology);
    if (!full && (m_stageDirtyMin > m_stageDirtyMax))
    {
        m_shapeLock.release();
        return false;
    }

    int row0, row1;
    if (full)
    {
        m_rows     = m_stageRows;
        m_cols     = m_stageCols;
        m_geometry = m_stageGeometry;
        m_topology = m_stageTopology;
        m_local    = m_stage;
        row0 = 0;
        row1 = m_rows - 1;
    }
    else
    {
        row0 = m_stageDirtyMin;
        row1 = m_stageDirtyMax;
        std::memcpy(&m_local[(size_t)row0 * m_cols], &m_stage[(size_t)row0 * m_cols],
                    sizeof(float) * (size_t)(row1 - row0 + 1) * m_cols);
    }

    m_stageDirtyMin = m_stageRows;
    m_stageDirtyMax = -1;

    m_shapeLock.release();

    //--------------------------------------------------------------------------
    // Upload into the 2D alpha texture: width = columns (S_theta), height =
    // rows (S_h), alpha = S_r in metres. Nearest filtering: the vertex shader
    // reads texel centres. Repeat around the circumference, clamp in height.
    //--------------------------------------------------------------------------
    glBindTexture(GL_TEXTURE_2D, m_texture);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 4);

    if (full)
    {
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_ALPHA32F_ARB, m_cols, m_rows, 0,
                     GL_ALPHA, GL_FLOAT, m_local.data());
    }
    else
    {
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, row0, m_cols, row1 - row0 + 1,
                        GL_ALPHA, GL_FLOAT, &m_local[(size_t)row0 * m_cols]);
    }

    glBindTexture(GL_TEXTURE_2D, 0);
    return true;
}

} // namespace vp
