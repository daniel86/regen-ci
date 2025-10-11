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
		// no movement, just standing around
		IDLE,
		// sitting, e.g. on a chair or on the ground
		SIT,
		// agreeing, e.g. nodding
		AGREE,
		// disagreeing, e.g. shaking head
		DISAGREE,
		// attacking, e.g. with a weapon
		ATTACK,
		// blocking, e.g. with a shield
		BLOCK,
		// crouching, e.g. to hide or sneak
		CROUCH,
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

	std::ostream &operator<<(std::ostream &out, const MotionType &v);

	std::istream &operator>>(std::istream &in, MotionType &v);

} // namespace

#endif /* REGEN_MODEL_MOTION_TYPE_H_ */
