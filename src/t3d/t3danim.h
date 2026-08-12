/**
* @copyright 2024 - Max Bebök
* @license MIT
* @file t3danim.h
*/
#ifndef TINY3D_T3DANIM_H
#define TINY3D_T3DANIM_H

#include "t3dmodel.h"
#include "t3dskeleton.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define T3D_ANIM_TARGET_TRANSLATION 0
#define T3D_ANIM_TARGET_SCALE_XYZ   1
#define T3D_ANIM_TARGET_SCALE_S     2
#define T3D_ANIM_TARGET_ROTATION    3

typedef struct {
  float timeStart;
  float timeEnd;
  int32_t* changedFlag; // flag to increment when target is changed
} T3DAnimTargetBase;

typedef struct {
  T3DAnimTargetBase base;
  T3DQuat* targetQuat; // target to modify
  T3DQuat kfCurr; // current keyframe value
  T3DQuat kfNext; // next keyframe value
} T3DAnimTargetQuat;

typedef struct {
  T3DAnimTargetBase base;
  float* targetScalar;
  float kfCurr;
  float kfNext;
} T3DAnimTargetScalar;

typedef struct {
  T3DChunkAnim *animRef;
  T3DAnimTargetQuat *targetsQuat;
  T3DAnimTargetScalar *targetsScalar;

  float speed;
  float time;

  FILE *file;
  // sr64: optional MEMORY keyframe source. When dataPtr is non-NULL the
  // reader takes keyframes from it at dataOffset instead of fread()ing from
  // `file`, and rewind resets the offset rather than the stream.
  //
  // Why: the .sdata file position IS the playback cursor, so a per-FILE*
  // design forces one open handle per playing instance. Every biped playing
  // the same clip then holds its own newlib FILE buffer and DragonFS handle
  // for the whole animation — on N64 that is both the largest per-actor cost
  // and a load-time failure point when the heap is fragmented. The clips are
  // tiny (a grunt's are 0.4-6 KB), so the caller can load one ONCE, share it
  // across every instance, and keep only this 8-byte cursor per anim.
  //
  // The buffer is BORROWED: t3d_anim_destroy* never frees it.
  const uint8_t *dataPtr;
  uint32_t dataOffset;
  uint32_t dataSize;
  int nextKfSize;
  uint8_t isPlaying;
  uint8_t isLooping;
} T3DAnim;

/**
 * Creates an animation instance from a model's animation definition
 * @param model The model to create the animation from
 * @param name The name of the animation to create
 * @return The created animation
 */
T3DAnim t3d_anim_create(const T3DModel *model, const char* name);

/**
 * Attaches an animation to a skeleton.
 * @param anim The animation to attach
 * @param skeleton The skeleton to attach the animation to
 */
void t3d_anim_attach(T3DAnim* anim, const T3DSkeleton* skeleton);

/**
 * Like t3d_anim_attach, but the channel-target block comes from `alloc`
 * instead of calloc. Returns false (leaving targetsQuat NULL) if it refuses.
 * Added for sr64's arena-backed animation pool; see t3dskeleton.h.
 *
 * Pair with t3d_anim_destroy_free, NOT t3d_anim_destroy — the latter free()s
 * the target block, which your allocator owns.
 */
bool t3d_anim_attach_alloc(T3DAnim* anim, const T3DSkeleton* skeleton,
                           void* (*alloc)(size_t size, void *ctx),
                           void (*free_fn)(void *ptr, void *ctx), void *ctx);

/// Counterpart of t3d_anim_attach_alloc: releases the target block through
/// `free_fn` and closes the keyframe stream. NULL-safe.
/**
 * Point an animation at an in-memory keyframe buffer instead of a file.
 * `data` is BORROWED — the caller keeps ownership and must outlive the anim.
 * Pass NULL to return to file-backed reading. Resets the cursor.
 */
void t3d_anim_set_source_memory(T3DAnim *anim, const void *data, uint32_t size);

void t3d_anim_destroy_free(T3DAnim *anim,
                           void (*free_fn)(void *ptr, void *ctx), void *ctx);


/**
 * Attaches a single position target to a single channel target.
 * This can be used to override an earlier attachment from 't3d_anim_attach'.
 *
 * @param anim animation to attach to
 * @param targetIdx index of the target bone
 * @param target position to update
 * @param updateFlag set to '1' if changed, '2' if animation rolled over
 */
void t3d_anim_attach_pos(T3DAnim* anim, uint32_t targetIdx, T3DVec3* target, int32_t *updateFlag);

/**
 * Attaches a single rotation target to a single channel target.
 * This can be used to override an earlier attachment from 't3d_anim_attach'.
 *
 * @param anim animation to attach to
 * @param targetIdx index of the target bone
 * @param target rotation to update
 * @param updateFlag set to '1' if changed, '2' if animation rolled over
 */
void t3d_anim_attach_rot(T3DAnim* anim, uint32_t targetIdx, T3DQuat* target, int32_t *updateFlag);

/**
 * Attaches a single scale target to a single channel target.
 * This can be used to override an earlier attachment from 't3d_anim_attach'.
 *
 * @param anim animation to attach to
 * @param targetIdx index of the target bone
 * @param target scale to update
 * @param updateFlag set to '1' if changed, '2' if animation rolled over
 */
void t3d_anim_attach_scale(T3DAnim* anim, uint32_t targetIdx, T3DVec3* target, int32_t *updateFlag);

/**
 * Updates an animation, this will actually play it and apply the changes to the skeleton.
 * @param anim animation to update
 * @param deltaTime time since last update (seconds)
 */
void t3d_anim_update(T3DAnim* anim, float deltaTime);

/**
 * Sets the animation to a specific time.
 * Note: this may cause some work internally due to potential DMAs.
 * @param anim animation to set time for
 * @param time time in seconds
 */
void t3d_anim_set_time(T3DAnim* anim, float time);

/**
 * Returns the current time of an animation in seconds.
 * Alternatively, you can also read the 'time' member directly.
 * @param anim animation to get time for
 * @return current time in seconds
 */
inline static float t3d_anim_get_time(const T3DAnim* anim) {
  return anim->time;
}

/**
 * Returns the length of an animation in seconds.
 * @param anim animation to get length for
 * @return length in seconds
 */
inline static float t3d_anim_get_length(const T3DAnim* anim) {
  return anim->animRef->duration;
}

/**
 * Sets the speed of the animation.
 * Note: reverse playback (speed < 0) is currently not supported.
 * @param anim animation to set speed for
 * @param speed speed as a factor, default: 1.0
 */
inline static void t3d_anim_set_speed(T3DAnim* anim, float speed) {
  anim->speed = speed < 0.0f ? 0.0f : speed;
}

/**
 * Set the animation to playing or paused.
 * Note: this works independently of setting the speed to 0.
 * @param anim
 * @param isPlaying true to play, false to pause
 */
inline static void t3d_anim_set_playing(T3DAnim* anim, bool isPlaying) {
  anim->isPlaying = isPlaying;
}

/**
 * Checks if a given animation is still playing.
 * For looping animations this will always be true once started.
 * For non-looping animations this will be false once the end is reached.
 *
 * @param anim animation to check
 * @return true if playing, false if paused or ended
 */
inline static bool t3d_anim_is_playing(const T3DAnim* anim) {
  return anim->isPlaying != 0;
}


/**
 * Sets the animation to loop or not.
 * If the animation is not set to loop, you will have to call 't3d_anim_set_playing' to play it again.
 * @param anim animation to set loop for
 * @param loop true to loop, false to stop at the end
 */
inline static void t3d_anim_set_looping(T3DAnim* anim, bool loop) {
  anim->isLooping = loop;
}

/**
 * Frees data allocated in the animation struct.
 * @param anim
 */
void t3d_anim_destroy(T3DAnim *anim);

#ifdef __cplusplus
}
#endif

#endif // TINY3D_T3DANIM_H