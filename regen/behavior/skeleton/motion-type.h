#ifndef REGEN_MODEL_MOTION_TYPE_H_
#define REGEN_MODEL_MOTION_TYPE_H_
#include <iostream>

namespace regen {
	enum class MotionType {
		// fast movement, usually to flee or chase
		RUN = 0,
		// normal movement, usually to walk around
		WALK,
		// jumping up
		JUMP,
		// swimming, e.g. in water
		SWIM,
		// no movement, just standing around
		IDLE,
		// sitting, e.g. on a chair or on the ground
		SIT,
		// agreeing, e.g. nodding
		AGREE,
		// disagreeing, e.g. shaking head
		DISAGREE,
		// vocalizing, e.g. talking or barking
		VOCALIZE,
		// attacking, e.g. with a weapon
		ATTACK,
		// blocking, e.g. with a shield
		BLOCK,
		// crouching, e.g. to hide or sneak
		CROUCH,
		// looking around, e.g. turning head or body or in case
		// of animals sniffing the ground or air
		INSPECT,
		// threatening or trying to scare away another character, e.g. by roaring if animal
		INTIMIDATE,
		// stretching, e.g. after waking up
		STRETCH,
		// grooming, e.g. cleaning oneself
		GROOMING,
		// praying or crouching to meditate or to worship
		PRAY,
		// sleeping, usually while lying down and being still
		SLEEP,
		// usually falling to the ground
		DIE,
		// standing up after being dead
		REVIVE,
		MOTION_LAST
	};

	/**
	 * True for motions where the animation can be interrupted and overruled by another motion.
	 * This means no matter at which stage of the animation we are, interruptible motions will
	 * fade out right away when no longer desired.
	 * This is eg. useful for motions like WALK or RUN where we want to be able to stop
	 * walking or running at any time.
	 * @param type The motion type.
	 * @return true if the given motion type can be interrupted.
	 */
	bool isInterruptibleMotion(MotionType type);

	std::ostream &operator<<(std::ostream &out, const MotionType &v);

	std::istream &operator>>(std::istream &in, MotionType &v);

} // namespace

#endif /* REGEN_MODEL_MOTION_TYPE_H_ */
