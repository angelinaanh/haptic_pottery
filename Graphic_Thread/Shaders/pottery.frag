#version 120
//==============================================================================
//  Pottery fragment shader: wet clay.
//
//  Blinn-Phong under the scene's first two lights (set by CHAI3D through the
//  fixed-function state, read back here), with faint throwing rings so the
//  rotation of the wheel and the relief of the surface read clearly.
//==============================================================================

uniform vec3 uColor;            // clay colour

varying vec3 vNormal;
varying vec3 vEye;
varying vec2 vCyl;

void main()
{
    vec3 n = normalize(vNormal);
    if (!gl_FrontFacing) { n = -n; }       // inner wall, seen from inside
    vec3 v = normalize(-vEye);

    float ring = 1.0 + 0.035 * sin(vCyl.y * 2200.0 + 1.7 * sin(vCyl.x * 2.0))
                     + 0.020 * sin(vCyl.y * 640.0);
    vec3 base  = uColor * ring;
    vec3 color = base * 0.30;              // ambient

    for (int i = 0; i < 2; i++)
    {
        vec3  l  = normalize(gl_LightSource[i].position.xyz);
        float nd = max(dot(n, l), 0.0);
        vec3  h  = normalize(l + v);
        float sp = (nd > 0.0) ? pow(max(dot(n, h), 0.0), 48.0) : 0.0;

        color += base * gl_LightSource[i].diffuse.rgb * nd
               + 0.22 * gl_LightSource[i].specular.rgb * sp;
    }

    gl_FragColor = vec4(color, 1.0);
}
