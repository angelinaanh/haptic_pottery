#version 120
//==============================================================================
//  Pottery vertex shader (Lee 2009, Sec. II-C).
//
//  The vertex buffer is a static grid in cylindrical coordinates; S_r comes
//  from the 2D alpha texture the GraphicSynchronizer keeps up to date. This
//  shader turns (S_r, S_theta, S_h) into a Cartesian position and derives the
//  surface normal, work that would otherwise be redone on the CPU for every
//  changed element every frame.
//
//  Per-vertex input
//    gl_Vertex.x          S_theta [rad]
//    gl_Vertex.y          radius mode: 0 outer wall, 1 inner wall (S_r - wall),
//                         2 on the axis (centre of a cap)
//    gl_Vertex.z          normal mode: 0 outward, 1 inward, 2 straight up
//    gl_MultiTexCoord0.xy element in the grid (column, row)
//    gl_MultiTexCoord0.z  S_h [m]
//==============================================================================

uniform sampler2D uRadius;      // alpha = S_r [m]
uniform vec2      uGrid;        // cols, rows of the grid
uniform float     uDh;          // height spacing of the grid [m]
uniform float     uDTheta;      // angular spacing of the grid [rad]
uniform float     uWall;        // wall thickness of a hollow body [m]

varying vec3 vNormal;           // eye space
varying vec3 vEye;              // eye space position
varying vec2 vCyl;              // (theta, height) for the surface pattern

float radiusAt(float col, float row)
{
    // Texel centres; wrap around the circumference (GL_REPEAT), clamp in
    // height so the end rows see themselves as neighbours.
    row = clamp(row, 0.0, uGrid.y - 1.0);
    return texture2DLod(uRadius, vec2((col + 0.5) / uGrid.x, (row + 0.5) / uGrid.y), 0.0).a;
}

void main()
{
    float theta = gl_Vertex.x;
    float rMode = gl_Vertex.y;
    float nMode = gl_Vertex.z;
    vec2  g     = gl_MultiTexCoord0.xy;
    float z     = gl_MultiTexCoord0.z;

    // S_r of the element, and its central differences around and along.
    float r  = radiusAt(g.x, g.y);
    float rT = 0.5 * (radiusAt(g.x + 1.0, g.y) - radiusAt(g.x - 1.0, g.y)) / uDTheta;   // dS_r / dtheta
    float rH = 0.5 * (radiusAt(g.x, g.y + 1.0) - radiusAt(g.x, g.y - 1.0)) / uDh;       // dS_r / dh

    float rad = (rMode < 0.5) ? r : ((rMode < 1.5) ? r - uWall : 0.0);

    // Cylindrical -> Cartesian.
    float c = cos(theta);
    float s = sin(theta);
    vec3  P = vec3(rad * c, rad * s, z);

    // Normal = dP/dtheta x dP/dh, for P(theta, h) = (S_r cos, S_r sin, h).
    vec3 N = vec3(0.0, 0.0, 1.0);
    if (nMode < 1.5)
    {
        vec3 Tt = vec3(rT * c - rad * s, rT * s + rad * c, 0.0);
        vec3 Th = vec3(rH * c, rH * s, 1.0);
        N = cross(Tt, Th);
        if (nMode > 0.5) { N = -N; }
    }

    vec4 eye    = gl_ModelViewMatrix * vec4(P, 1.0);
    vEye        = eye.xyz;
    vNormal     = gl_NormalMatrix * N;
    vCyl        = vec2(theta, z);
    gl_Position = gl_ProjectionMatrix * eye;
}
