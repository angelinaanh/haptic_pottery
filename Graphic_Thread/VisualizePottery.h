//==============================================================================
//  Graphic thread: visualisation of the pottery.
//
//  Sets up the OpenGL scene through CHAI3D - camera, lights, the wheel, the
//  HIP sphere, a minimal read-out and the method buttons - and draws the clay.
//  The clay is a static vertex buffer holding the (S_theta, S_h) grid, drawn
//  with the pottery shaders, which read S_r from the GraphicSynchronizer's
//  alpha texture: the CPU never transforms a vertex.
//==============================================================================
#ifndef VP_VISUALIZE_POTTERY_H
#define VP_VISUALIZE_POTTERY_H

#include "chai3d.h"
#include "GraphicSynchronizer.h"

#include <string>

namespace vp {

class PotteryMesh;

//------------------------------------------------------------------------------

//! What the read-out shows besides the haptic frame.
struct HudInfo
{
    std::string benchmark;      //!< progress text while a benchmark runs
};

//==============================================================================

class VisualizePottery
{
public:

    VisualizePottery();
    ~VisualizePottery();

    //! Builds the scene. Needs a current GL context. False if the pottery
    //! shaders cannot be loaded or compiled from \p a_shaderDir.
    bool init(GraphicSynchronizer* a_sync, const std::string& a_shaderDir,
              float a_contentScaleX, float a_contentScaleY);

    //! Releases the scene. Call while the GL context still exists.
    void shutdown();

    //! Draws one frame into the current framebuffer.
    void render(int a_width, int a_height, const HapticFrame& a_frame, const HudInfo& a_hud);

    //! Point under the mouse cursor on the vertical plane x = \p a_planeX.
    bool cursorToPlane(double a_mouseX, double a_mouseY, int a_windowW, int a_windowH,
                       double a_planeX, chai3d::cVector3d& a_point) const;

    //! Method button under a point given in window coordinates (GLFW: origin
    //! top-left), or -1.
    int methodButtonAt(double a_mouseX, double a_mouseY, int a_windowW, int a_windowH) const;

    void setContentScale(float a_x, float a_y);
    void toggleWireframe();
    void toggleHelp() { m_showHelp = !m_showHelp; }
    void setStatus(const std::string& a_text);

private:

    void buildScene();
    void buildOverlay();
    void updateOverlay(int a_width, int a_height, const HapticFrame& a_frame, const HudInfo& a_hud);

    GraphicSynchronizer* m_sync;

    chai3d::cWorld*            m_world;
    chai3d::cCamera*           m_camera;
    chai3d::cViewport*         m_viewport;
    chai3d::cDirectionalLight* m_keyLight;
    chai3d::cDirectionalLight* m_fillLight;

    PotteryMesh*            m_mesh;
    chai3d::cShapeCylinder* m_wheel;
    chai3d::cShapeSphere*   m_hip;
    chai3d::cShapeLine*     m_forceLine;

    chai3d::cFontPtr m_font;
    chai3d::cLabel*  m_title;
    chai3d::cLabel*  m_line1;
    chai3d::cLabel*  m_line2;
    chai3d::cLabel*  m_warning;
    chai3d::cLabel*  m_status;
    chai3d::cLabel*  m_hint;
    chai3d::cPanel*  m_helpPanel;
    chai3d::cLabel*  m_helpText;
    bool             m_showHelp;

    //! The two method buttons (Baseline, Method 2), top right.
    struct Button
    {
        chai3d::cPanel* panel;
        chai3d::cLabel* label;
        PotteryMethod   method;
    };
    Button m_buttons[2];

    int m_displayW;             //!< overlay size, for the button hit test
    int m_displayH;

    std::string               m_statusText;
    double                    m_statusUntil;
    chai3d::cPrecisionClock   m_clock;
};

} // namespace vp

#endif // VP_VISUALIZE_POTTERY_H
