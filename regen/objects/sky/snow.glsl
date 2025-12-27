
-- update.vs
#include regen.objects.sky.precipitation.update.vs

-- draw.vs
#include regen.objects.sky.precipitation.draw.vs
-- draw.gs
#include regen.objects.sky.precipitation.draw.gs
-- draw.fs
#include regen.objects.mesh.defines

layout(location = 0) out vec4 out_color;
in float in_brightness;
in vec3 in_posWorld;
in vec3 in_norWorld;
in vec2 in_spriteTexco;

#include regen.camera.camera.input

#include regen.shading.direct.diffuse
#include regen.camera.camera.linearizeDepth
#include regen.camera.camera.transformEyeToWorld
#include regen.objects.particles.sprite.softParticleScale
#include regen.textures.noise.random2D
#ifdef HAS_fogDistance
#include regen.objects.sky.fog.applyFogToColor
#endif

uniform sampler2D in_snowTexture;

void main() {
    vec3 P = in_posWorld.xyz;
#ifdef NORMAL_CORRECTION
    vec2 spriteTexco = in_spriteTexco*2.0 - vec2(1.0);
    vec3 norEye = vec3(spriteTexco, sqrt(1.0 - min(dot(spriteTexco,spriteTexco), 1.0)));
    vec3 N = normalize(transformEyeToWorld(vec4(norEye,0.0),in_layer).xyz);
#else
    vec3 N = normalize(in_posWorld.xyz - in_cameraPosition.xyz);
#endif
    float brightness = in_brightness;
    float density = texture(in_snowTexture, in_spriteTexco).x;
#ifdef HAS_softParticleScale
    // fade out particles intersecting the world
    density *= softParticleScale();
#endif
    vec3 C = getDiffuseLight(P, N, gl_FragCoord.z);
    C = brightness*(C + in_ambientLight);
    C /= max(C.x, max(C.y, C.z)); // normalize
#ifdef HAS_fogDistance
    C = applyFogToColor(C, gl_FragCoord.z, P);
#endif
    out_color = density*vec4(C,  1.0);
}
