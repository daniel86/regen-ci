#ifndef REGEN_SKELETON_MOTION_CLIP_H_
#define REGEN_SKELETON_MOTION_CLIP_H_
#include "motion-type.h"
#include "regen/animation/animation-range.h"

namespace regen {
	struct MotionClip {
		/** The type of motion for this clip. */
		MotionType motion = MotionType::IDLE;
		/** The animation range for this clip. */
		const AnimationRange *loop = nullptr;
		/** The range to run to start the animation. Optional. */
		const AnimationRange *begin = nullptr;
		/** The range to run to end the animation. Optional. */
		const AnimationRange *end = nullptr;
	};
} // namespace

#endif /* REGEN_SKELETON_MOTION_CLIP_H_ */
