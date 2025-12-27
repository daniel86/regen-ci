
-- regen_InstanceID
#ifndef regen_InstanceID_defined_
#define2 regen_InstanceID_defined_

#if SHADER_STAGE==fs
    #ifdef HAS_instanceIDMap
#define regen_InstanceID in_instanceIDMap[in_instanceID]
    #else // HAS_instanceIDMap
#define regen_InstanceID in_instanceID
    #endif // HAS_instanceIDMap
#elif SHADER_STAGE==gs
    #ifdef HAS_instanceIDMap
#define regen_InstanceID in_instanceIDMap[in_instanceID[0]]
    #else // HAS_instanceIDMap
#define regen_InstanceID in_instanceID[0]
    #endif // HAS_instanceIDMap
#else
    #ifdef HAS_instanceIDMap
#define regen_InstanceID in_instanceIDMap[gl_InstanceID + gl_BaseInstance]
    #else
#define regen_InstanceID (gl_InstanceID + gl_BaseInstance)
    #endif
#endif
#endif // regen_InstanceID_defined_

-- regen_RenderLayer
#ifndef regen_RenderLayer_defined_
#define2 regen_RenderLayer_defined_
#ifdef HAS_layer
#define regen_RenderLayer() in_layer
#else // HAS_layer
    #if RENDER_LAYER > 1
    // Compute render layer from gl_DrawID.
    // Currently the mesh may have n indirect draw calls for n LOD levels,
    // with layered rendering we repeat each LOD level for each layer.
        #ifdef USE_GS_LAYERED_RENDERING
#define regen_RenderLayer() 0
        #else
#define regen_RenderLayer() (gl_DrawID % ${RENDER_LAYER})
        #endif
    #else
#define regen_RenderLayer() 0
    #endif
#endif // regen_InstanceID_defined_
#endif

-- VS_SelectLayer
#ifndef regen_VS_SelectLayer_defined_
#define2 regen_VS_SelectLayer_defined_
#ifdef VS_LAYER_SELECTION
void VS_SelectLayer(int layer) {
    out_layer = layer;
#ifdef ARB_shader_viewport_layer_array
    gl_Layer = layer;
#endif // ARB_shader_viewport_layer_array
}
#else // !defined(VS_LAYER_SELECTION)
#define VS_SelectLayer(x)
#endif // VS_LAYER_SELECTION
#endif

-- layer-defines
#ifndef regen_layered_defines_defined_
#define regen_layered_defines_defined_
#ifndef HAS_layer
    #if RENDER_LAYER > 1
flat in int in_layer;
    #else
#define in_layer 0
    #endif
#endif
#endif // regen_layered_defines_defined_

-- all-defines
// enable GL_ARB_shader_viewport_layer_array if we do layered rendering.
// this allows us to select the render layer in the vertex shader.
#if SHADER_STAGE == vs
    #ifdef HAS_layer
        #define2 _REQUIRE_VS_LAYER_SELECTION
    #elif RENDER_LAYER > 1
        #define2 _REQUIRE_VS_LAYER_SELECTION
    #endif
#endif
#ifdef _REQUIRE_VS_LAYER_SELECTION
    #ifdef ARB_shader_viewport_layer_array
        #ifndef USE_GS_LAYERED_RENDERING
            #ifndef USE_GEOMETRY_SHADER
#extension GL_ARB_shader_viewport_layer_array : require
            #endif
        #endif
    #endif
#endif
#ifdef HAS_layer
#define VS_LAYER_SELECTION
#elif RENDER_LAYER > 1
    #ifndef USE_GS_LAYERED_RENDERING
#define VS_LAYER_SELECTION
    #endif
#endif
#ifdef HAS_nor && HAS_tan
#define HAS_TANGENT_SPACE
#endif
#if SHADER_STAGE == tes
#define SAMPLE(T,C) texture(T,INTERPOLATE_VALUE(C))
#else
#define SAMPLE(T,C) texture(T,C)
#endif
#if OUTPUT_TYPE == DEPTH
    #ifndef DEPTH_FACE
        #define DEPTH_FACE BACK
    #endif
#endif

#ifndef PI
#define PI 3.14159265
#endif
