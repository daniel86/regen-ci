
-- selectViewIdx.maxDot
#ifndef REGEN_selectViewIdx_defined_
#define2 REGEN_selectViewIdx_defined_
uint selectViewIdx(vec3 viewDirLocal) {
    float maxDot = 0, d;
    uint bestIndex = 0;
    for (uint i = 0; i < NUM_IMPOSTOR_VIEWS; ++i) {
        d = dot(viewDirLocal, in_snapshotDirs[i].xyz);
        if (d > maxDot) {
            maxDot = d;
            bestIndex = i;
        }
    }
    return bestIndex;
}
#endif

-- selectViewIdx.bin
#ifndef REGEN_selectViewIdx_defined_
#define2 REGEN_selectViewIdx_defined_
uint selectViewIdx(vec3 dir) {
    const float TWO_PI = 6.2831853;

    // Longitude in [0, 2π)
    float lonAngle = atan(-dir.z, -dir.x); // returns [-π, π]
    lonAngle = mod(lonAngle + TWO_PI, TWO_PI);
    // Compute the longitude segment index
    int lonIdx = int((lonAngle + in_longitudeHalfStep) / in_longitudeStep);
    lonIdx = (max(lonIdx, 0) % in_numLongitudeSteps);

    // Latitude in [-π/2, π/2]
    float angle_y = atan(dir.y, length(dir.xz));
#ifdef IS_HEMISPHERICAL
    int latIdx = int((angle_y + in_latitudeHalfStep) / in_latitudeStep);
    latIdx = clamp(latIdx, 0, in_numLatitudeSteps - 1);
#else
    float f_latIdx = ((angle_y + in_latitudeHalfStep) / in_latitudeStep);
    int neg = int(step(f_latIdx, 0.0)); // 1 if latIdx < 0
    int mapped = (in_numLatitudeSteps - 1) + min(int(-f_latIdx)+1, in_numLatitudeSteps - 1);
    int latIdx = int(f_latIdx) * (1 - neg) + mapped * neg;
#endif

    return uint(latIdx * in_numLongitudeSteps + lonIdx);
}
#endif

-- getViewIdx
#ifndef REGEN_getViewIdx_defined_
#define2 REGEN_getViewIdx_defined_
#include regen.models.impostor.selectViewIdx.bin
uint getViewIdx(int layer, vec3 centerWorld) {
    // Find the best impostor view index based on the view direction.
#ifdef HAS_modelMatrix
    vec3 viewDirWorld = normalize(REGEN_CAM_POS_(layer) - centerWorld.xyz);
    vec3 viewDirLocal = transpose(mat3(in_modelMatrix)) * viewDirWorld;
#else
    vec3 viewDirLocal = normalize(REGEN_CAM_POS_(layer) - centerWorld.xyz);
#endif
    return selectViewIdx(viewDirLocal);
}
#endif

-- getSpriteSize
#ifndef REGEN_getSpriteSize_defined_
#define2 REGEN_getSpriteSize_defined_
vec2 getSpriteSize(inout vec4 centerEye, vec3 zAxis, uint viewIdx, float scale) {
    vec4 orthoBounds = in_snapshotOrthoBounds[viewIdx];
    // Compute size of the quad in world space, based on the ortho bounds of the selected view.
    vec2 spriteSize = vec2(orthoBounds.y - orthoBounds.x, orthoBounds.w - orthoBounds.z) * scale;
#ifndef DEPTH_CORRECT
    #if RENDER_TARGET_MODE != CASCADE
    vec2 depthRange = in_snapshotDepthRanges[viewIdx];
    float zCenter = max(abs(centerEye.z), 1e-4);
        #if OUTPUT_TYPE == DEPTH && DEPTH_FACE == BACK
    centerEye.xyz += zAxis * 0.5 * (depthRange.y - depthRange.x) * scale;
        #else
    centerEye.xyz -= zAxis * 0.5 * (depthRange.y - depthRange.x) * scale;
        #endif
    spriteSize *= abs(centerEye.z / zCenter);
    #endif
#endif
    return spriteSize;
}
#endif

/**
 * This shader renders different views of a billboard impostor using 2D texture arrays.
 * Only one draw call is needed to render all views of the impostor thanks to layered rendering
 * and geometry shaders.
 * The geometry shader takes as input a triangle, and emits it into the different layers of
 * the array textures.
 **/
-- update.defines
// the model should be centered at origin, so we need
// to ignore the model matrix.
#define IGNORE_modelMatrix
#define IGNORE_modelOffset
// material parameters are handled by billboard to
// allow per-instance variations.
#define IGNORE_MATERIAL

-- update.vs
#include regen.models.impostor.update.defines
#include regen.models.mesh.defines
#include regen.models.mesh.vs

-- update.gs
#include regen.models.impostor.update.defines
#include regen.models.mesh.gs

-- update.fs
#include regen.models.impostor.update.defines
#include regen.models.mesh.fs

-- extrude.vs
#include regen.models.mesh.defines

in vec3 in_pos;
#ifdef HAS_INSTANCES
flat out int out_instanceID;
#endif
#if RENDER_LAYER > 1
flat out int out_layer;
#define in_layer regen_RenderLayer()
#endif

#define HANDLE_IO(i)

void main() {
    int layer = regen_RenderLayer();
    vec4 pos = vec4(in_pos,1.0);
#ifdef HAS_modelOrigin
    // Translate the center of the quad to the center of the original mesh,
    // for the case where the mesh is not centered at the origin.
    pos.xyz += in_modelOrigin;
#endif
#ifdef HAS_modelMatrix
    pos = in_modelMatrix * pos;
#endif
#ifdef HAS_modelOffset
    pos.xyz += in_modelOffset.xyz;
#endif
    gl_Position = pos;
#ifdef HAS_INSTANCES
    out_instanceID = gl_InstanceID + gl_BaseInstance;
#endif // HAS_INSTANCES
#if RENDER_LAYER > 1
    out_layer = layer;
#endif
    HANDLE_IO(gl_VertexID);
}

-- extrude.gs
#include regen.models.mesh.defines

layout(points) in;
layout(triangle_strip, max_vertices=6) out;

#include regen.states.camera.input

flat out uint out_impostorIdx;
out vec3 out_texco0;
out vec3 out_posEye;
out vec3 out_posWorld;
//out vec3 out_norWorld;

buffer vec4 in_snapshotDirs[];
buffer vec4 in_snapshotOrthoBounds[];
buffer vec2 in_snapshotDepthRanges[];

const float in_depthOffset = 0.5f;

#include regen.states.camera.transformEyeToScreen
#include regen.states.camera.transformEyeToWorld
#include regen.states.camera.transformWorldToEye
#include regen.math.computeSpritePoints

#ifdef HAS_windFlow
#include regen.models.sprite.applyForce
#include regen.weather.wind.windAtPosition
#endif

#define HANDLE_IO(i)

#include regen.models.impostor.getSpriteSize
#include regen.models.impostor.getViewIdx

#if RENDER_TARGET_MODE == CASCADE
void emitVertex(vec3 posWorld, vec3 texco, int layer) {
    vec4 posEye = transformWorldToEye(vec4(posWorld,1.0),layer);
    out_texco0 = texco;
    out_posWorld = posWorld;
    out_posEye = posEye.xyz;
    //out_norWorld = vec3(0.0, 1.0, 0.0);
    gl_Position = transformEyeToScreen(posEye,layer);
    HANDLE_IO(0);
    EmitVertex();
}
#else
void emitVertex(vec4 posEye, vec3 texco, int layer) {
    out_texco0 = texco;
    out_posEye = posEye.xyz;
    out_posWorld = transformEyeToWorld(posEye,layer).xyz;
    //out_norWorld = vec3(0.0, 1.0, 0.0);
    gl_Position = transformEyeToScreen(posEye,layer);
    HANDLE_IO(0);
    EmitVertex();
}
#endif

#if RENDER_TARGET_MODE == CASCADE
const float EPS_CROSS = 1e-6;
void emitLayer(int layer, float scale) {
    // world-space center
    vec3 centerWorld = gl_in[0].gl_Position.xyz;
    // choose snapshot view index
    uint viewIdx = getViewIdx(layer, centerWorld.xyz);
    float viewCoord = float(viewIdx);
    // transform center to light-eye-space for getSpriteSize
    mat4 V = REGEN_VIEW_(layer);
    vec4 centerEye = V * vec4(centerWorld, 1.0);
    // approximate eye-space forward for getSpriteSize
    vec3 zAxisEye = vec3(0.0, 0.0, -1.0);
    vec2 spriteSize = getSpriteSize(centerEye, zAxisEye, viewIdx, scale);

    // compute world-space billboard axes from view matrix
    vec3 lightForward = normalize(-V[2].xyz); // points from sprite -> camera
    vec3 lightRight   = normalize( V[0].xyz);
    vec3 lightUp      = normalize( V[1].xyz);
    // fallback for degenerate forward
    if(dot(lightForward,lightForward) < EPS_CROSS) lightForward = vec3(0.0,0.0,1.0);

    // compute quad corners in world space
    vec3 halfX = lightRight * 0.5 * spriteSize.x;
    vec3 halfY = lightUp    * 0.5 * spriteSize.y;
    vec3 p0 = centerWorld - halfX - halfY; // bottom-left
    vec3 p1 = centerWorld - halfX + halfY; // top-left
    vec3 p2 = centerWorld + halfX - halfY; // bottom-right
    vec3 p3 = centerWorld + halfX + halfY; // top-right

    // emit two triangles
    out_impostorIdx = viewIdx;
    // bottom-left, top-left, bottom-right
    emitVertex(p2, vec3(1.0, 0.0, viewCoord), layer);
    emitVertex(p1, vec3(0.0, 1.0, viewCoord), layer);
    emitVertex(p0, vec3(0.0, 0.0, viewCoord), layer);
    EndPrimitive();
    // bottom-right, top-left, top-right
    emitVertex(p3, vec3(1.0, 1.0, viewCoord), layer);
    emitVertex(p1, vec3(0.0, 1.0, viewCoord), layer);
    emitVertex(p2, vec3(1.0, 0.0, viewCoord), layer);
    EndPrimitive();
}
#else
void emitLayer(int layer, float scale) {
    vec4 centerWorld = gl_in[0].gl_Position;
    vec4 centerEye = transformWorldToEye(centerWorld, layer);
    // Find the best impostor view index based on the view direction.
    uint viewIdx = getViewIdx(layer, centerWorld.xyz);
    float viewCoord = float(viewIdx);
    // Compute the size of the quad in eye space, based on the ortho bounds of the selected view.
    vec3 zAxis = normalize(centerEye.xyz);
    vec2 spriteSize = getSpriteSize(centerEye, zAxis, viewIdx, scale);
    // Finally, compute the four corners of the quad in eye space.
    vec3 up = mix(vec3(1.0, 0.0, 0.0), vec3(0.0, 1.0, 0.0), float(abs(zAxis.y) < 0.99));
    vec3 quadPos[4] = computeSpritePoints(centerEye.xyz, spriteSize, zAxis, up);

    #ifdef HAS_windFlow
    vec3 bottomCenter = 0.5*(quadPos[0] + quadPos[2]);
    vec2 wind = windAtPosition(bottomCenter);
    // cancel out wind along zAxis.xz to avoid artifacts.
    wind -= dot(wind, zAxis.xz) * zAxis.xz;
    // apply the wind force to the quad
    applyForce(quadPos, wind);
    #endif

    // Emit the quad as two triangles.
    out_impostorIdx = viewIdx;
    // bottom-left, top-left, bottom-right
    emitVertex(vec4(quadPos[2],1.0), vec3(1.0,0.0,viewCoord), layer);
    emitVertex(vec4(quadPos[1],1.0), vec3(0.0,1.0,viewCoord), layer);
    emitVertex(vec4(quadPos[0],1.0), vec3(0.0,0.0,viewCoord), layer);
    EndPrimitive();
    // bottom-right, top-left, top-right
    emitVertex(vec4(quadPos[3],1.0), vec3(1.0,1.0,viewCoord), layer);
    emitVertex(vec4(quadPos[1],1.0), vec3(0.0,1.0,viewCoord), layer);
    emitVertex(vec4(quadPos[2],1.0), vec3(1.0,0.0,viewCoord), layer);
    EndPrimitive();
}
#endif

void main() {
#ifdef HAS_modelMatrix
    float scale = length(in_modelMatrix[0].xyz);
#else
    float scale = 1.0;
#endif
#if RENDER_LAYER > 1
    int layer = in_layer[0];
    gl_Layer = layer;
#else
    int layer = 0;
#endif
    emitLayer(layer, scale);
}

/**
 * This shader renders a billboard impostor using 2D texture arrays.
 * The arrays are used for mapping to diffuse, normal, specular components,
 * and can be combined with material properties in the fragment shader.
 **/
-- vs
#include regen.models.impostor.extrude.vs
-- gs
#include regen.models.impostor.extrude.gs
-- fs
#include regen.models.mesh.fs
