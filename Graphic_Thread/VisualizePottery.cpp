//==============================================================================
//  Graphic thread: visualisation of the pottery. See VisualizePottery.h.
//==============================================================================
#include "VisualizePottery.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

using namespace chai3d;
using namespace std;

namespace vp {

namespace {

// Palette for the white interface.
const cColorf kText      (0.15f, 0.16f, 0.18f);
const cColorf kTextMuted (0.45f, 0.47f, 0.50f);
const cColorf kAccent    (0.74f, 0.42f, 0.27f);     // terracotta
const cColorf kButtonIdle(0.90f, 0.91f, 0.92f);
const cColorf kWarning   (0.80f, 0.20f, 0.15f);

} // namespace

//==============================================================================
//  PotteryMesh: the clay as a CHAI3D scene object.
//
//  Vertex layout (7 floats): theta, radius mode, normal mode, 1 | u, v, z,
//  with (u, v) the element (column, row) in the grid. See pottery.vert.
//  The grid holds the outer wall, and for a hollow body the inner wall, the
//  rim and the inner floor; a solid body gets a flat top instead. It is
//  rebuilt only when the synchroniser reallocates the texture; every other
//  change of shape reaches the screen through the texture alone.
//==============================================================================

class PotteryMesh : public cGenericObject
{
public:

    explicit PotteryMesh(GraphicSynchronizer* a_sync)
        : m_sync(a_sync), m_program(0), m_vbo(0), m_ibo(0), m_indexCount(0),
          m_topology(0), m_wireframe(false)
    {
        m_color = kAccent;
        m_boundaryBoxMin.set(-kClayMaxRadius, -kClayMaxRadius, kWheelTopZ);
        m_boundaryBoxMax.set( kClayMaxRadius,  kClayMaxRadius, kWheelTopZ + kClayHeight);
        m_boundaryBoxEmpty = false;
    }

    bool loadShaders(const string& a_dir);

    void setWireframe(bool a_on) { m_wireframe = a_on; }
    bool getWireframe() const    { return m_wireframe; }

    virtual void render(cRenderOptions& a_options);

private:

    void rebuildGrid();

    GraphicSynchronizer* m_sync;
    GLuint   m_program;
    GLuint   m_vbo;
    GLuint   m_ibo;
    GLsizei  m_indexCount;
    uint64_t m_topology;
    bool     m_wireframe;
    cColorf  m_color;
};

//------------------------------------------------------------------------------

namespace {

bool readFile(const string& a_path, string& a_text)
{
    ifstream in(a_path.c_str(), ios::in | ios::binary);
    if (!in) { return false; }
    ostringstream ss;
    ss << in.rdbuf();
    a_text = ss.str();
    return true;
}

GLuint compileShader(GLenum a_type, const string& a_source, const string& a_name)
{
    GLuint shader = glCreateShader(a_type);
    const char* src = a_source.c_str();
    glShaderSource(shader, 1, &src, NULL);
    glCompileShader(shader);

    GLint ok = GL_FALSE;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
    if (!ok)
    {
        char log[4096];
        glGetShaderInfoLog(shader, sizeof(log), NULL, log);
        cout << "[render] " << a_name << " does not compile:" << endl << log << endl;
        glDeleteShader(shader);
        return 0;
    }
    return shader;
}

} // namespace

//------------------------------------------------------------------------------

bool PotteryMesh::loadShaders(const string& a_dir)
{
    string vertSrc, fragSrc;
    if (!readFile(a_dir + "/pottery.vert", vertSrc) || !readFile(a_dir + "/pottery.frag", fragSrc))
    {
        cout << "[render] cannot read the shaders in " << a_dir << endl;
        return false;
    }

    GLint vertexUnits = 0;
    glGetIntegerv(GL_MAX_VERTEX_TEXTURE_IMAGE_UNITS, &vertexUnits);
    if (vertexUnits < 1)
    {
        cout << "[render] this GPU cannot read textures in a vertex shader" << endl;
        return false;
    }

    GLuint vs = compileShader(GL_VERTEX_SHADER, vertSrc, "pottery.vert");
    GLuint fs = compileShader(GL_FRAGMENT_SHADER, fragSrc, "pottery.frag");
    if ((vs == 0) || (fs == 0)) { return false; }

    m_program = glCreateProgram();
    glAttachShader(m_program, vs);
    glAttachShader(m_program, fs);
    glLinkProgram(m_program);
    glDeleteShader(vs);
    glDeleteShader(fs);

    GLint ok = GL_FALSE;
    glGetProgramiv(m_program, GL_LINK_STATUS, &ok);
    if (!ok)
    {
        char log[4096];
        glGetProgramInfoLog(m_program, sizeof(log), NULL, log);
        cout << "[render] pottery shaders do not link:" << endl << log << endl;
        return false;
    }

    glGenBuffers(1, &m_vbo);
    glGenBuffers(1, &m_ibo);
    return true;
}

//------------------------------------------------------------------------------

void PotteryMesh::rebuildGrid()
{
    const int rows = m_sync->rows();
    const int cols = m_sync->cols();
    const PotteryGeometry& g = m_sync->geometry();
    const double dh = g.height / (double)(rows - 1);

    const int R = rows;
    const int C = cols;
    const int W = C + 1;                        // seam column duplicated

    vector<float>    v;
    vector<unsigned> idx;
    v.reserve((size_t)(2 * R + 4) * W * 7);

    auto vertex = [&](int a_row, int a_c, float a_rMode, float a_nMode) -> unsigned
    {
        v.push_back((float)(2.0 * C_PI * a_c / C));
        v.push_back(a_rMode);
        v.push_back(a_nMode);
        v.push_back(1.0f);
        v.push_back((float)a_c);
        v.push_back((float)a_row);
        v.push_back((float)(g.baseZ + a_row * dh));
        return (unsigned)(v.size() / 7 - 1);
    };

    auto wall = [&](int a_row0, float a_rMode, float a_nMode, bool a_flip)
    {
        unsigned base = (unsigned)(v.size() / 7);
        for (int i = a_row0; i < R; i++)
            for (int c = 0; c <= C; c++)
                vertex(i, c, a_rMode, a_nMode);

        for (int i = 0; i < R - 1 - a_row0; i++)
        {
            for (int c = 0; c < C; c++)
            {
                unsigned a = base + i * W + c, b = a + 1, d = a + W, e = d + 1;
                if (!a_flip) { idx.insert(idx.end(), { a, b, e, a, e, d }); }
                else         { idx.insert(idx.end(), { a, e, b, a, d, e }); }
            }
        }
    };

    auto disc = [&](int a_row, float a_rMode)
    {
        unsigned centre = vertex(a_row, 0, 2.0f, 2.0f);
        unsigned ring   = (unsigned)(v.size() / 7);
        for (int c = 0; c <= C; c++) vertex(a_row, c, a_rMode, 2.0f);
        for (int c = 0; c < C; c++) idx.insert(idx.end(), { centre, ring + c, ring + c + 1 });
    };

    const int top = R - 1;

    wall(0, 0.0f, 0.0f, false);                 // outer wall, normals outward

    if (g.hollow)
    {
        int floorRow = (int)std::floor((g.floorZ - g.baseZ) / dh) + 1;
        floorRow = std::max(0, std::min(R - 2, floorRow));

        wall(floorRow, 1.0f, 1.0f, true);       // inner wall, normals inward

        unsigned outer = (unsigned)(v.size() / 7);
        for (int c = 0; c <= C; c++) vertex(top, c, 0.0f, 2.0f);
        unsigned inner = (unsigned)(v.size() / 7);
        for (int c = 0; c <= C; c++) vertex(top, c, 1.0f, 2.0f);
        for (int c = 0; c < C; c++)
        {
            unsigned o0 = outer + c, o1 = o0 + 1, n0 = inner + c, n1 = n0 + 1;
            idx.insert(idx.end(), { o0, o1, n1, o0, n1, n0 });
        }

        disc(floorRow, 1.0f);                   // inner floor
    }
    else
    {
        disc(top, 0.0f);                        // flat top
    }

    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    glBufferData(GL_ARRAY_BUFFER, v.size() * sizeof(float), v.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, idx.size() * sizeof(unsigned), idx.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    m_indexCount = (GLsizei)idx.size();
    m_topology   = m_sync->topology();
}

//------------------------------------------------------------------------------

void PotteryMesh::render(cRenderOptions& a_options)
{
    if (!SECTION_RENDER_OPAQUE_PARTS_ONLY(a_options) || a_options.m_creating_shadow_map) { return; }
    if ((m_program == 0) || (m_sync->topology() == 0)) { return; }

    if (m_topology != m_sync->topology()) { rebuildGrid(); }

    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glPushClientAttrib(GL_CLIENT_ALL_ATTRIB_BITS);

    glDisable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
    glPolygonMode(GL_FRONT_AND_BACK, m_wireframe ? GL_LINE : GL_FILL);

    glUseProgram(m_program);
    glUniform1i(glGetUniformLocation(m_program, "uRadius"), 0);
    glUniform2f(glGetUniformLocation(m_program, "uGrid"), (float)m_sync->cols(), (float)m_sync->rows());
    glUniform1f(glGetUniformLocation(m_program, "uDh"),
                (float)(m_sync->geometry().height / (m_sync->rows() - 1)));
    glUniform1f(glGetUniformLocation(m_program, "uDTheta"), (float)(2.0 * C_PI / m_sync->cols()));
    glUniform1f(glGetUniformLocation(m_program, "uWall"), (float)m_sync->geometry().wall);
    glUniform3f(glGetUniformLocation(m_program, "uColor"), m_color.getR(), m_color.getG(), m_color.getB());

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, m_sync->texture());

    const GLsizei stride = 7 * sizeof(float);
    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(4, GL_FLOAT, stride, (void*)0);
    glClientActiveTexture(GL_TEXTURE0);
    glEnableClientState(GL_TEXTURE_COORD_ARRAY);
    glTexCoordPointer(3, GL_FLOAT, stride, (void*)(4 * sizeof(float)));

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ibo);
    glDrawElements(GL_TRIANGLES, m_indexCount, GL_UNSIGNED_INT, (void*)0);

    glUseProgram(0);
    glBindTexture(GL_TEXTURE_2D, 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    glPopClientAttrib();
    glPopAttrib();
}

//==============================================================================
//  VisualizePottery
//==============================================================================

VisualizePottery::VisualizePottery()
{
    m_sync      = nullptr;
    m_world     = nullptr;
    m_camera    = nullptr;
    m_viewport  = nullptr;
    m_keyLight  = nullptr;
    m_fillLight = nullptr;
    m_mesh      = nullptr;
    m_wheel     = nullptr;
    m_hip       = nullptr;
    m_forceLine = nullptr;
    m_title = m_line1 = m_line2 = m_warning = m_status = m_hint = nullptr;
    m_helpPanel = nullptr;
    m_helpText  = nullptr;
    m_showHelp  = false;
    m_displayW  = 0;
    m_displayH  = 0;
    m_statusUntil = 0.0;
    for (int b = 0; b < 2; b++) { m_buttons[b].panel = nullptr; m_buttons[b].label = nullptr; }
}

VisualizePottery::~VisualizePottery()
{
    shutdown();
}

void VisualizePottery::shutdown()
{
    // The viewport is deliberately not deleted: CHAI3D 3.3's ~cViewport()
    // deletes its view panel twice (once through its internal camera's front
    // layer, once directly) and crashes. It is released with the process.
    m_viewport = nullptr;

    delete m_world;     // deletes every object in the scene graph
    m_world = nullptr;
}

//------------------------------------------------------------------------------

bool VisualizePottery::init(GraphicSynchronizer* a_sync, const string& a_shaderDir,
                            float a_contentScaleX, float a_contentScaleY)
{
    m_sync = a_sync;
    m_clock.start(true);

    buildScene();
    buildOverlay();

    m_viewport = new cViewport(m_camera, a_contentScaleX, a_contentScaleY);

    return m_mesh->loadShaders(a_shaderDir);
}

//------------------------------------------------------------------------------

void VisualizePottery::buildScene()
{
    m_world = new cWorld();
    m_world->m_backgroundColor.set(1.0f, 1.0f, 1.0f);

    // Raised front view of the wheel axis. The device's x axis points at the
    // viewer, so hand motion and screen motion agree.
    m_camera = new cCamera(m_world);
    m_world->addChild(m_camera);
    m_camera->set(cVector3d(0.30, 0.0, 0.085),
                  cVector3d(0.00, 0.0, -0.008),
                  cVector3d(0.00, 0.0, 1.000));
    m_camera->setClippingPlanes(0.01, 3.0);

    // The pottery shader reads the first two lights.
    m_keyLight = new cDirectionalLight(m_world);
    m_world->addChild(m_keyLight);
    m_keyLight->setEnabled(true);
    m_keyLight->setDir(-0.7, -0.45, -0.55);
    m_keyLight->m_ambient.set(0.35f, 0.35f, 0.36f);
    m_keyLight->m_diffuse.set(0.85f, 0.83f, 0.80f);
    m_keyLight->m_specular.set(0.80f, 0.80f, 0.80f);

    m_fillLight = new cDirectionalLight(m_world);
    m_world->addChild(m_fillLight);
    m_fillLight->setEnabled(true);
    m_fillLight->setDir(-0.4, 0.8, 0.2);
    m_fillLight->m_ambient.set(0.10f, 0.10f, 0.10f);
    m_fillLight->m_diffuse.set(0.35f, 0.36f, 0.40f);
    m_fillLight->m_specular.set(0.20f, 0.20f, 0.22f);

    // Stand (static) and wheel head (turns with the clay).
    cShapeCylinder* stand = new cShapeCylinder(0.020, 0.030, 0.06);
    m_world->addChild(stand);
    stand->setLocalPos(0.0, 0.0, kWheelTopZ - kWheelThickness - 0.06);
    stand->m_material->setColorf(0.62f, 0.64f, 0.67f);

    m_wheel = new cShapeCylinder(kWheelRadius, kWheelRadius, kWheelThickness);
    m_world->addChild(m_wheel);
    m_wheel->setLocalPos(0.0, 0.0, kWheelTopZ - kWheelThickness);
    m_wheel->m_material->setColorf(0.80f, 0.82f, 0.84f);
    m_wheel->m_material->setShininess(30);

    // A marker on the rim makes the rotation visible.
    cShapeBox* marker = new cShapeBox(0.012, 0.006, 0.002);
    m_wheel->addChild(marker);
    marker->setLocalPos(kWheelRadius - 0.010, 0.0, kWheelThickness + 0.001);
    marker->m_material->setColorf(0.35f, 0.37f, 0.40f);

    // The clay.
    m_mesh = new PotteryMesh(m_sync);
    m_world->addChild(m_mesh);

    // The HIP: a red sphere, as in the papers.
    m_hip = new cShapeSphere(0.006);
    m_world->addChild(m_hip);
    m_hip->m_material->setRedCrimson();
    m_hip->m_material->setShininess(60);

    // The force sent to the device, drawn from the HIP (25 mm per newton).
    m_forceLine = new cShapeLine(cVector3d(0, 0, 0), cVector3d(0, 0, 0));
    m_world->addChild(m_forceLine);
    m_forceLine->setLineWidth(3.0);
    m_forceLine->m_colorPointA.set(0.85f, 0.45f, 0.05f);
    m_forceLine->m_colorPointB.set(0.85f, 0.45f, 0.05f);
}

//------------------------------------------------------------------------------

void VisualizePottery::buildOverlay()
{
    m_font = NEW_CFONT_CALIBRI_20();

    auto label = [&](const cColorf& a_color, double a_scale) -> cLabel*
    {
        cLabel* l = new cLabel(m_font);
        m_camera->m_frontLayer->addChild(l);
        l->m_fontColor = a_color;
        l->setFontScale(a_scale);
        return l;
    };

    m_title   = label(kText, 1.15);
    m_line1   = label(kTextMuted, 0.85);
    m_line2   = label(kText, 0.95);
    m_warning = label(kWarning, 0.85);
    m_status  = label(kTextMuted, 0.85);
    m_hint    = label(kTextMuted, 0.75);
    m_title->setText("Virtual Pottery");
    m_hint->setText("F1  keys");

    // Method buttons.
    const PotteryMethod methods[2] = { METHOD_BASELINE, METHOD_2 };
    for (int b = 0; b < 2; b++)
    {
        Button& btn = m_buttons[b];
        btn.method = methods[b];
        btn.panel  = new cPanel();
        m_camera->m_frontLayer->addChild(btn.panel);
        btn.panel->setSize(118, 36);
        btn.panel->setCornerRadius(8, 8, 8, 8);
        btn.label = new cLabel(m_font);
        btn.panel->addChild(btn.label);
        btn.label->setText(methodConfig(methods[b]).name);
        btn.label->setFontScale(0.9);
    }

    // Key list, shown with F1.
    m_helpPanel = new cPanel();
    m_camera->m_frontLayer->addChild(m_helpPanel);
    m_helpPanel->setColor(cColorf(0.96f, 0.96f, 0.97f));
    m_helpPanel->setCornerRadius(8, 8, 8, 8);

    m_helpText = new cLabel(m_font);
    m_helpPanel->addChild(m_helpText);
    m_helpText->m_fontColor = kText;
    m_helpText->setFontScale(0.8);
    m_helpText->setText(
        "0 / 2      Baseline (original method) / Method 2\n"
        "SPACE      spin the wheel            UP / DOWN  wheel speed\n"
        "[  ]       tool size                 ,  .  resolution\n"
        "R          reset clay                O  open / close (hollow)\n"
        "K V D C    stiffness, viscosity, damping, friction (SHIFT: lower)\n"
        "P          propagation on/off        W  wireframe\n"
        "Mouse      move the tool, wheel = depth, right button = spin");
}

//------------------------------------------------------------------------------

void VisualizePottery::setContentScale(float a_x, float a_y)
{
    if (m_viewport) { m_viewport->setContentScale(a_x, a_y); }
}

void VisualizePottery::toggleWireframe()
{
    m_mesh->setWireframe(!m_mesh->getWireframe());
}

void VisualizePottery::setStatus(const string& a_text)
{
    m_statusText  = a_text;
    m_statusUntil = m_clock.getCurrentTimeSeconds() + 3.0;
    cout << "[ui] " << a_text << endl;
}

//------------------------------------------------------------------------------

bool VisualizePottery::cursorToPlane(double a_mouseX, double a_mouseY, int a_windowW, int a_windowH,
                                     double a_planeX, cVector3d& a_point) const
{
    if ((a_windowW <= 0) || (a_windowH <= 0)) { return false; }

    // Ray through the cursor (GLFW: origin top-left).
    double nx = (2.0 * a_mouseX / a_windowW) - 1.0;
    double ny = 1.0 - (2.0 * a_mouseY / a_windowH);
    double aspect  = (double)a_windowW / (double)a_windowH;
    double tanHalf = std::tan(0.5 * cDegToRad(m_camera->getFieldViewAngleDeg()));

    cVector3d eye = m_camera->getLocalPos();
    cVector3d dir = m_camera->getLookVector()
                  + (nx * aspect * tanHalf) * m_camera->getRightVector()
                  + (ny * tanHalf) * m_camera->getUpVector();

    if (std::fabs(dir.x()) < 1e-9) { return false; }
    double t = (a_planeX - eye.x()) / dir.x();
    if (t <= 0.0) { return false; }

    a_point = eye + t * dir;
    return true;
}

//------------------------------------------------------------------------------

int VisualizePottery::methodButtonAt(double a_mouseX, double a_mouseY, int a_windowW, int a_windowH) const
{
    if ((a_windowW <= 0) || (a_windowH <= 0)) { return -1; }

    // Window pixels (top-left origin) -> overlay pixels (bottom-left origin).
    double x = a_mouseX / a_windowW * m_displayW;
    double y = (1.0 - a_mouseY / a_windowH) * m_displayH;

    for (int b = 0; b < 2; b++)
    {
        const cPanel* p = m_buttons[b].panel;
        cVector3d pos = p->getLocalPos();
        if ((x >= pos.x()) && (x <= pos.x() + p->getWidth()) &&
            (y >= pos.y()) && (y <= pos.y() + p->getHeight()))
        {
            return (int)m_buttons[b].method;
        }
    }
    return -1;
}

//------------------------------------------------------------------------------

void VisualizePottery::render(int a_width, int a_height, const HapticFrame& a_frame,
                              const HudInfo& a_hud)
{
    // Clay and wheel head turn together.
    cMatrix3d rot;
    rot.setAxisAngleRotationRad(cVector3d(0.0, 0.0, 1.0), a_frame.wheelAngle);
    m_mesh->setLocalRot(rot);
    m_wheel->setLocalRot(rot);

    // HIP and force.
    m_hip->setLocalPos(a_frame.hipPosition);
    m_hip->setRadius(std::max(1e-4, a_frame.hipRadius));
    if ((a_frame.contacts > 0) || a_frame.rigidContact) { m_hip->m_material->setColorf(1.0f, 0.35f, 0.25f); }
    else                                                { m_hip->m_material->setRedCrimson(); }

    m_forceLine->m_pointA = a_frame.hipPosition;
    m_forceLine->m_pointB = a_frame.hipPosition + 0.025 * a_frame.force;

    updateOverlay(m_viewport->getDisplayWidth(), m_viewport->getDisplayHeight(), a_frame, a_hud);

    m_world->updateShadowMaps(false, false);
    m_viewport->renderView(a_width, a_height);
}

//------------------------------------------------------------------------------

void VisualizePottery::updateOverlay(int a_width, int a_height, const HapticFrame& a_frame,
                                     const HudInfo& a_hud)
{
    m_displayW = a_width;
    m_displayH = a_height;

    // Line 1: what is running.
    ostringstream l1;
    l1 << fixed << setprecision(0) << methodConfig(a_frame.method).name << "    "
       << a_frame.rows << " x " << a_frame.cols
       << "    haptic " << a_frame.hapticRate << " Hz    physics " << a_frame.physicsRate << " Hz";
    m_line1->setText(l1.str());

    // Line 2: the three quantities that matter while working the clay.
    ostringstream l2;
    l2 << fixed << setprecision(2) << "Force " << a_frame.force.length() << " N      Depth "
       << a_frame.meanDepth * 1000.0 << " mm      Contacts " << a_frame.contacts;
    m_line2->setText(l2.str());

    m_warning->setText(!a_hud.benchmark.empty() ? a_hud.benchmark
                     : (a_frame.armed ? string("") : string("Force off - move the tool away from the clay")));

    bool showStatus = m_clock.getCurrentTimeSeconds() < m_statusUntil;
    m_status->setText(showStatus ? m_statusText : string(""));

    // Layout.
    m_title->setLocalPos(18, a_height - 36);
    m_line1->setLocalPos(18, a_height - 62);
    m_line2->setLocalPos(18, a_height - 88);
    m_warning->setLocalPos(18, a_height - 112);
    m_status->setLocalPos(18, 14);
    m_hint->setLocalPos(a_width - m_hint->getWidth() - 18, 14);

    // Method buttons, top right; the running method is filled.
    for (int b = 0; b < 2; b++)
    {
        Button& btn = m_buttons[b];
        bool active = (a_frame.method == btn.method);
        btn.panel->setColor(active ? kAccent : kButtonIdle);
        btn.label->m_fontColor = active ? cColorf(1.0f, 1.0f, 1.0f) : kText;
        btn.panel->setLocalPos(a_width - 18 - (2 - b) * 128 + 10, a_height - 54);
        btn.label->setLocalPos(0.5 * (btn.panel->getWidth() - btn.label->getWidth()),
                               0.5 * (btn.panel->getHeight() - btn.label->getHeight()));
    }

    m_helpPanel->setShowEnabled(m_showHelp);
    if (m_showHelp)
    {
        double w = m_helpText->getWidth() + 28;
        double h = m_helpText->getHeight() + 24;
        m_helpPanel->setSize(w, h);
        m_helpPanel->setLocalPos(a_width - w - 18, 40);
        m_helpText->setLocalPos(14, 12);
    }
}

} // namespace vp
