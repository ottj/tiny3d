/**
* @copyright 2024 - Max Bebök
* @license MIT
*/

#include "t3d/t3danim.h"
#include <string.h>
#include <malloc.h>

#define SQRT_2_INV 0.70710678118f
#define KF_TIME_TICK (1.0f / 60.0f)

// Maps the input data streamed from the animation data file
typedef struct {
  uint16_t nextTime;
  uint16_t channelIdx;
  uint16_t data[2]; // can be either 1 or 2 16-bit values (scalar / quat)
} T3DAnimKF;

T3DAnim t3d_anim_create(const T3DModel *model, const char *name) {
  T3DChunkAnim* animDef = t3d_model_get_animation(model, name);
  assertf(animDef, "Animation '%s' not found in model", name);

  return (T3DAnim){
    .animRef = animDef,
    .targetsScalar = NULL,
    .targetsQuat = NULL,
    .time = 0.0f,
    .speed = 1.0f,
    .nextKfSize = sizeof(T3DAnimKF),
    .file = asset_fopen(animDef->filePath, NULL),
    .isPlaying = 1,
    .isLooping = 1
  };
}

static void rewind_anim(T3DAnim *anim)
{
  for(int c=0; c<anim->animRef->channelsScalar; c++) {
    anim->targetsScalar[c].base.timeEnd = 0;
  }
  for(int c=0; c<anim->animRef->channelsQuat; c++) {
    anim->targetsQuat[c].base.timeEnd = 0;
  }
  anim->nextKfSize = sizeof(T3DAnimKF);
  if(anim->dataPtr) anim->dataOffset = 0;   // memory source: cursor is the offset
  else if(anim->file) rewind(anim->file);
}

void t3d_anim_set_source_memory(T3DAnim *anim, const void *data, uint32_t size) {
  if(anim == NULL)return;
  anim->dataPtr = (const uint8_t*)data;
  anim->dataOffset = 0;
  anim->dataSize = size;
}

// Wire each channel's target pointer at the matching bone field. Factored out
// of t3d_anim_attach so the allocator-hooked variant shares one body — a second
// copy of this loop would be a silent divergence waiting to happen.
static void attach_targets(T3DAnim *anim, const T3DSkeleton *skeleton) {
  uint32_t channelCount = anim->animRef->channelsScalar + anim->animRef->channelsQuat;

  uint32_t idxQuat = 0;
  uint32_t idxScalar = 0;
  for(uint32_t i = 0; i < channelCount; i++)
  {
    T3DAnimChannelMapping *channelMap = &anim->animRef->channelMappings[i];
    T3DBone *bone = &skeleton->bones[channelMap->targetIdx];

    switch(channelMap->targetType) {
      case T3D_ANIM_TARGET_TRANSLATION:
        anim->targetsScalar[idxScalar].targetScalar = &bone->position.v[channelMap->attributeIdx];
        anim->targetsScalar[idxScalar++].base.changedFlag = &bone->hasChanged;
        break;
      case T3D_ANIM_TARGET_SCALE_XYZ:
        anim->targetsScalar[idxScalar].targetScalar = &bone->scale.v[channelMap->attributeIdx];
        anim->targetsScalar[idxScalar++].base.changedFlag = &bone->hasChanged;
        break;
      case T3D_ANIM_TARGET_ROTATION:
        anim->targetsQuat[idxQuat].targetQuat = &bone->rotation;
        anim->targetsQuat[idxQuat++].base.changedFlag = &bone->hasChanged;
      break;
      default: {assertf(false, "Unknown animation target %d", channelMap->targetType);}
    }
  }
}

void t3d_anim_attach(T3DAnim *anim, const T3DSkeleton *skeleton) {
  if(anim->targetsQuat)free(anim->targetsQuat);

  size_t allocQuat = sizeof(T3DAnimTargetQuat) * anim->animRef->channelsQuat;
  size_t allocScalar = sizeof(T3DAnimTargetScalar) * anim->animRef->channelsScalar;
  anim->targetsQuat = calloc(allocQuat + allocScalar, 1); // only allocate a single block
  anim->targetsScalar = (T3DAnimTargetScalar*)((uint8_t*)anim->targetsQuat + allocQuat);
  rewind_anim(anim);
  attach_targets(anim, skeleton);
}

inline static void attach_scalar(T3DAnim* anim, uint32_t targetIdx, T3DVec3* target, int32_t *updateFlag, uint8_t targetType) {
  for(int i = 0; i < anim->animRef->channelsScalar; i++) {
    T3DAnimChannelMapping *channelMap = &anim->animRef->channelMappings[i+anim->animRef->channelsQuat];
    if(channelMap->targetIdx == targetIdx && channelMap->targetType == targetType) {
      anim->targetsScalar[i].targetScalar = &target->v[channelMap->attributeIdx];
      anim->targetsScalar[i].base.changedFlag = updateFlag;
    }
  }
}

void t3d_anim_attach_pos(T3DAnim* anim, uint32_t targetIdx, T3DVec3* target, int32_t *updateFlag) {
  attach_scalar(anim, targetIdx, target, updateFlag, T3D_ANIM_TARGET_TRANSLATION);
}

void t3d_anim_attach_scale(T3DAnim *anim, uint32_t targetIdx, T3DVec3 *target, int32_t *updateFlag) {
  attach_scalar(anim, targetIdx, target, updateFlag, T3D_ANIM_TARGET_SCALE_XYZ);
}

void t3d_anim_attach_rot(T3DAnim *anim, uint32_t targetIdx, T3DQuat *target, int32_t *updateFlag) {
  for(int i = 0; i < anim->animRef->channelsQuat; i++) {
    T3DAnimChannelMapping *channelMap = &anim->animRef->channelMappings[i];
    if(channelMap->targetIdx == targetIdx && channelMap->targetType == T3D_ANIM_TARGET_ROTATION) {
      anim->targetsQuat[i].targetQuat = target;
      anim->targetsQuat[i].base.changedFlag = updateFlag;
    }
  }
}

static inline float s10ToFloat(uint32_t value, float offset, float scale) {
  return (float)value / 1023.0f * scale + offset;
}

static inline void unpack_quat(uint16_t dataHi, uint16_t dataLo, T3DQuat *out) {
  int largestIdx = dataHi >> 14;
  int idx0 = (largestIdx + 1) & 0b11;
  int idx1 = (largestIdx + 2) & 0b11;
  int idx2 = (largestIdx + 3) & 0b11;

  uint16_t dataMid = (dataHi << 6) | (dataLo >> 10);
  float q0 = s10ToFloat((dataHi >> 4) & 0x3FF, -SQRT_2_INV, SQRT_2_INV+SQRT_2_INV);
  float q1 = s10ToFloat((dataMid    ) & 0x3FF, -SQRT_2_INV, SQRT_2_INV+SQRT_2_INV);
  float q2 = s10ToFloat((dataLo     ) & 0x3FF, -SQRT_2_INV, SQRT_2_INV+SQRT_2_INV);

  out->v[idx0] = q0;
  out->v[idx1] = q1;
  out->v[idx2] = q2;
  out->v[largestIdx] = sqrtf(1.0f - q0*q0 - q1*q1 - q2*q2);
}

static inline T3DAnimTargetBase* get_base_target(T3DAnim *anim, uint64_t channelIdx, bool isRot) {
  return isRot ?
    (T3DAnimTargetBase*)&anim->targetsQuat[channelIdx] :
    (T3DAnimTargetBase*)&anim->targetsScalar[channelIdx - anim->animRef->channelsQuat];
}

static inline bool load_keyframe(T3DAnim *anim) {
  T3DAnimKF kf;
  if(anim->dataPtr) {
    // Memory source. Same bytes the file path would have read; running off the
    // end is the normal end-of-animation signal, exactly as fread returning 0.
    if(anim->dataOffset + (uint32_t)anim->nextKfSize > anim->dataSize)return false;
    memcpy(&kf, anim->dataPtr + anim->dataOffset, anim->nextKfSize);
    anim->dataOffset += (uint32_t)anim->nextKfSize;
  } else {
    // No source at all: neither a memory buffer nor an open stream. Treat it
    // as "no more keyframes" rather than dereferencing NULL inside fread.
    // This is reachable when t3d_anim_create's asset_fopen fails (a missing or
    // unreadable keyframe file, or an out-of-memory open on a fragmented
    // heap), and the honest outcome is a static pose, not a crash — the
    // caller already handles an animation that yields nothing.
    if(anim->file == NULL)return false;
    size_t readBytes = fread(&kf, anim->nextKfSize, 1, anim->file);
    if(readBytes == 0)return false;
  }

  bool isLarge = kf.nextTime & 0x8000;
  anim->nextKfSize = isLarge ? sizeof(T3DAnimKF) : (sizeof(T3DAnimKF)-2);
  kf.nextTime &= 0x7FFF;

  T3DAnimChannelMapping *channelMap = &anim->animRef->channelMappings[kf.channelIdx];

  bool isRot = kf.channelIdx < anim->animRef->channelsQuat;
  T3DAnimTargetBase *targetBase = get_base_target(anim, kf.channelIdx, isRot);

  targetBase->timeStart = targetBase->timeEnd;
  targetBase->timeEnd += (float)kf.nextTime * KF_TIME_TICK;
  if(kf.nextTime == 0)targetBase->timeStart -= 0.00001f; // avoid zero-div for overlapping keyframes

  if(channelMap->targetType == T3D_ANIM_TARGET_ROTATION) {
    T3DAnimTargetQuat *target = (T3DAnimTargetQuat*)targetBase;
    target->kfCurr = target->kfNext;
    unpack_quat(kf.data[0], kf.data[1], &target->kfNext);
  } else {
    T3DAnimTargetScalar *target = (T3DAnimTargetScalar*)targetBase;
    target->kfCurr = target->kfNext;
    target->kfNext = (float)kf.data[0] * channelMap->quantScale + channelMap->quantOffset;
  }

  return true;
}

void t3d_anim_update(T3DAnim *anim, float deltaTime) {
  if(!anim->isPlaying)return;
  int32_t updateFlag = 1;
  anim->time += deltaTime * anim->speed;

  if(anim->time >= anim->animRef->duration) {
    anim->time -= anim->animRef->duration;
    rewind_anim(anim);
    updateFlag = 2;

    if(!anim->isLooping) {
      anim->isPlaying = 0;
      return;
    }
  }

  uint32_t channelCount = anim->animRef->channelsScalar + anim->animRef->channelsQuat;
  for(uint32_t c=0; c<channelCount; c++)
  {
    bool isRot = c < anim->animRef->channelsQuat;
    T3DAnimTargetBase *target = get_base_target(anim, c, isRot);

    while(anim->time >= target->timeEnd) {
      if(!load_keyframe(anim))break;
    }

    // timeDiff is ZERO when this channel loaded no keyframes at all — the
    // window never opened, so timeStart == timeEnd == 0. That happens for an
    // animation with no usable source (a failed asset_fopen, an empty or
    // truncated keyframe file), and dividing by it raises a DIV0 FPU exception
    // and takes the ROM down.
    //
    // The existing `if(kf.nextTime == 0) timeStart -= 0.00001f` above covers
    // ZERO-DURATION keyframes, which is a different case: there at least one
    // keyframe was read. This covers "none were".
    //
    // Holding interp at 0 pins the channel to kfCurr, i.e. a static pose —
    // the same outcome as an animation that has finished, and what every
    // caller already copes with.
    // SKIP the channel outright when its window never opened
    // (timeStart == timeEnd == 0, i.e. load_keyframe returned false before any
    // keyframe was read — an animation with no usable source). Two divides lie
    // below and BOTH blow up in that state:
    //   1. interp = (time - timeStart) / timeDiff        -> DIV0
    //   2. t3d_quat_nlerp -> t3d_quat_normalize divides by the quaternion
    //      length, and kfCurr/kfNext are still the calloc'd (0,0,0,0), so the
    //      length is 0 -> DIV0 one line later.
    // Guarding only the first moves the crash rather than fixing it; skipping
    // leaves the bone at its bind value, which is the static pose we want.
    //
    // NB the existing `if(kf.nextTime == 0) timeStart -= 0.00001f` above covers
    // ZERO-DURATION keyframes — a different case, where a keyframe WAS read.
    float timeDiff = target->timeEnd - target->timeStart;
    if(timeDiff <= 0.0f) {
      *target->changedFlag = updateFlag;
      continue;
    }
    float interp = (anim->time - target->timeStart) / timeDiff;
    *target->changedFlag = updateFlag;

    if(isRot) {
      T3DAnimTargetQuat *t = (T3DAnimTargetQuat*)target;
      t3d_quat_nlerp(t->targetQuat, &t->kfCurr, &t->kfNext, interp);
      //t3d_quat_slerp(t->targetQuat, &t->kfCurr, &t->kfNext, interp);
    } else {
      T3DAnimTargetScalar *t = (T3DAnimTargetScalar*)target;
      *t->targetScalar = t3d_lerp(t->kfCurr, t->kfNext, interp);
    }
  }
}

void t3d_anim_destroy(T3DAnim *anim) {
  if(anim->targetsQuat)free(anim->targetsQuat); // 'targetsScalar' is part of this memory-block
  if(anim->file)fclose(anim->file);
  anim->targetsQuat = NULL;
  anim->targetsScalar = NULL;
  anim->file = NULL;
}

void t3d_anim_set_time(T3DAnim *anim, float time) {
  if(time > anim->animRef->duration)time = anim->animRef->duration;
  if(time < anim->time)rewind_anim(anim);
  anim->time = time;
}

// --- allocator-hooked variants (sr64) --------------------------------------
// See t3danim.h. The channel-target block is the only allocation t3d_anim_attach
// makes, so routing it is enough to keep an animation entirely out of the
// shared heap.

bool t3d_anim_attach_alloc(T3DAnim *anim, const T3DSkeleton *skeleton,
                           void* (*alloc)(size_t size, void *ctx),
                           void (*free_fn)(void *ptr, void *ctx), void *ctx)
{
  if(anim == NULL || skeleton == NULL || alloc == NULL)return false;
  if(anim->targetsQuat && free_fn)free_fn(anim->targetsQuat, ctx);

  size_t allocQuat = sizeof(T3DAnimTargetQuat) * anim->animRef->channelsQuat;
  size_t allocScalar = sizeof(T3DAnimTargetScalar) * anim->animRef->channelsScalar;
  anim->targetsQuat = alloc(allocQuat + allocScalar, ctx);
  if(anim->targetsQuat == NULL)return false;
  memset(anim->targetsQuat, 0, allocQuat + allocScalar);
  anim->targetsScalar = (T3DAnimTargetScalar*)((uint8_t*)anim->targetsQuat + allocQuat);

  rewind_anim(anim);
  attach_targets(anim, skeleton);
  return true;
}

void t3d_anim_destroy_free(T3DAnim *anim,
                           void (*free_fn)(void *ptr, void *ctx), void *ctx)
{
  if(anim == NULL)return;
  if(anim->targetsQuat && free_fn)free_fn(anim->targetsQuat, ctx);
  anim->targetsQuat = NULL;
  anim->targetsScalar = NULL;
  if(anim->file){ fclose(anim->file); anim->file = NULL; }
}
