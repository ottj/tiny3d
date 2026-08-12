/**
* @copyright 2024 - Max Bebök
* @license MIT
* @file t3dskeleton.h
*/
#ifndef TINY3D_T3DSKELETON_H
#define TINY3D_T3DSKELETON_H

#include "t3dmodel.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * Bone instance, part of a skeleton.
 * 'matrix' will get updated by the skeleton when calling t3d_skeleton_update,
 * if 'hasChanged' is set to true.
 */
typedef struct {
  T3DMat4 matrix;
  T3DVec3 scale;
  T3DQuat rotation;
  T3DVec3 position;
  int32_t hasChanged;
} T3DBone;

/**
 * Skeleton instance, can be constructed from a model's skeleton definition.
 * This is used to draw skinned models.
 */
typedef struct {
  T3DBone* bones;
  T3DMat4FP* boneMatricesFP; // fixed point matrix, used for rendering — SKIN matrix (current * IBM)
  T3DMat4* inverseBindPoses; // one per bone, computed once at create from the bind SRT chain
  uint8_t bufferCount; // number of matrices buffers
  uint8_t currentBufferIdx;
  const T3DChunkSkeleton* skeletonRef; // reference to the model, defines skeleton structure
} T3DSkeleton;

/**
 * Creates a skeleton instance from a model's skeleton definition.
 * It will internally reserve multiple matrix stacks to allow for buffering.
 * This can be used to update a skeleton while the last frame is still being rendered.
 * Note that only the fixed-point matrices are buffered, the bone data itself is not.
 *
 * @param model The model to create the skeleton from
 * @param bufferCount number of buffers, should match the frame-buffer count
 * @return The created skeleton
 */
T3DSkeleton t3d_skeleton_create_buffered(const T3DModel *model, int bufferCount);

/// Allocator callbacks for t3d_skeleton_create_alloc. `alloc` returns a cached
/// buffer, `alloc_uncached` an uncached (KSEG1) one for the RSP-read matrix
/// stack; both may return NULL. `free_fn` accepts either kind.
/// Added for sr64: its animation memory lives in an arena-backed TLSF pool, and
/// the plain constructor's internal malloc/malloc_uncached cannot be redirected.
/// Same shape as T3DModelAllocFn (t3dmodel.h), which exists for the same reason.
typedef void* (*T3DSkelAllocFn)(size_t size, void *ctx);
typedef void  (*T3DSkelFreeFn)(void *ptr, void *ctx);

/**
 * Like t3d_skeleton_create_buffered, but every allocation comes from the
 * supplied callbacks instead of malloc/malloc_uncached.
 *
 * Unlike the plain constructor this NULL-CHECKS: on a failed allocation it
 * releases whatever it took and returns a zeroed T3DSkeleton (`bones == NULL`),
 * so a caller whose pool is full degrades instead of dereferencing NULL inside
 * t3d_mat4_invert_affine. Check `bones` before use.
 *
 * Free with t3d_skeleton_destroy_free and the matching free callback — NEVER
 * t3d_skeleton_destroy, which calls libc free() on pointers your allocator owns.
 */
T3DSkeleton t3d_skeleton_create_alloc(const T3DModel *model, int bufferCount,
                                      T3DSkelAllocFn alloc,
                                      T3DSkelAllocFn alloc_uncached,
                                      T3DSkelFreeFn free_fn, void *ctx);

/// Counterpart of t3d_skeleton_create_alloc. NULL-safe.
void t3d_skeleton_destroy_free(T3DSkeleton *skeleton, T3DSkelFreeFn free_fn, void *ctx);


/**
 * Creates a skeleton instance from a model's skeleton definition
 * @param model The model to create the skeleton from
 * @return The created skeleton
 */
static inline T3DSkeleton t3d_skeleton_create(const T3DModel *model) {
  return t3d_skeleton_create_buffered(model, 1);
}

/**
 * Sets the given skeleton up for the next draw call.
 * This has to be used for buffered skeletons, but is a no-op for single-buffer skeletons.
 * @param skel skeleton to use in the next draw
 */
static inline void t3d_skeleton_use(const T3DSkeleton *skel) {
  if(skel->bufferCount > 1) {
    void* mat = skel->boneMatricesFP + skel->currentBufferIdx * skel->skeletonRef->boneCount;
    t3d_segment_set(T3D_SEGMENT_SKELETON, mat);
  }
}

/**
 * Clones a skeleton instance.
 * This can also be used to create optimized skeletons for blending animations.
 * @param skel Skeleton to clone
 * @param useMatrices If false, no matrices will be allocated, this is useful for blending animations
 * @return Cloned skeleton
 */
T3DSkeleton t3d_skeleton_clone(const T3DSkeleton *skel, bool useMatrices);

/**
 * Resets a skeleton to its initial state (resting pose).
 * This can be useful when switching between animations.
 * Note: To recalculate the bone matrices too, call 't3d_skeleton_update' afterwards.
 * @param skeleton The skeleton to reset
 */
void t3d_skeleton_reset(T3DSkeleton *skeleton);

/**
 * Blends two skeletons together.
 * Note: it is safe to use the same skeleton as an input and output parameter.
 * @param skelRes Resulting skeleton
 * @param skelA First skeleton
 * @param skelB Second skeleton
 * @param factor Blend factor (0.0-1.0), you are allowed to go beyond these values to "overdrive" animations
 */
void t3d_skeleton_blend(const T3DSkeleton *skelRes, const T3DSkeleton *skelA, const T3DSkeleton *skelB, float factor);

/**
 * Updates the skeleton's bone matrices if data has changed.
 * Call this after making changes to the bones individual properties (pos/rot/scale).
 * To make this work, the `hasChanged` flag in the bone must also be set
 * @param skeleton The skeleton to update
 */
void t3d_skeleton_update(T3DSkeleton *skeleton);

/**
 * Frees data allocated in the skeleton struct.
 * Note: it's safe to call this multiple times, pointers are set to NULL.
 * @param skeleton The skeleton to destroy
 */
void t3d_skeleton_destroy(T3DSkeleton* skeleton);

/**
 * Tries to finds a bone with the given name in the skeleton.
 * Returns the index of the bone or -1 if not found.
 * @param skeleton The skeleton to search in
 * @param name Name of the bone to find
 * @return Index of the bone or -1 if not found
 */
int t3d_skeleton_find_bone(T3DSkeleton* skeleton, const char* name);

/**
 * Gets the position in model space of the bone with the given index from its matrix.
 * This assumes the bone matrix has been updated beforehand with t3d_skeleton_update.
 * @param skeleton The skeleton containing the bone
 * @param boneIdx Index of the bone
 * @return The T3DVec3 position of the bone in model space
 */
static inline T3DVec3 t3d_skeleton_get_bone_pos_model_space(const T3DSkeleton *skeleton, int boneIdx){
  assertf(boneIdx >= 0 && boneIdx < skeleton->skeletonRef->boneCount, "Bone index is out of bounds, idx: %i, boneCount: %i", boneIdx, skeleton->skeletonRef->boneCount);
  // Access the bone at the given index
  const T3DBone *bone = &skeleton->bones[boneIdx];

  // Extract the position from the matrix.
  return (T3DVec3) {{
    bone->matrix.m[3][0],
    bone->matrix.m[3][1],
    bone->matrix.m[3][2]
  }};
}

/**
 * Draws a skinned model with default settings.
 * Alternatively, use 't3d_model_draw_custom' and set 'matrices' in the config.
 * @param model
 */
static inline void t3d_model_draw_skinned(const T3DModel* model, const T3DSkeleton* skeleton) {
  t3d_model_draw_custom(model, (T3DModelDrawConf){
    .userData = NULL,
    .tileCb = NULL,
    .filterCb = NULL,
    .matrices = skeleton->bufferCount == 1
      ? skeleton->boneMatricesFP
      : (const T3DMat4FP*)t3d_segment_placeholder(T3D_SEGMENT_SKELETON)
  });
}

#ifdef __cplusplus
}
#endif

#endif //TINY3D_T3DSKELETON_H
