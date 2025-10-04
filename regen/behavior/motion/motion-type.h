#ifndef REGEN_MODEL_MOTION_TYPE_H_
#define REGEN_MODEL_MOTION_TYPE_H_

#include <iostream>

namespace regen {
	enum class MotionType {
		// fast movement, usually to flee or chase
		RUN = 0,
		// normal movement, usually to walk around
		WALK,
		// no movement, just standing around
		IDLE,
		YES,
		NO,
		// attacking, e.g. with a weapon
		ATTACK,
		// blocking, e.g. with a shield
		BLOCK,
		// crouching, e.g. to hide or sneak
		CROUCH,
		// praying or crouching to meditate or to worship
		PRAY,
		// standing up, e.g. after sleeping
		STAND_UP,
		// sleeping, usually while lying down and being still
		SLEEP,
		MOTION_LAST
	};

} // namespace

#endif /* REGEN_MODEL_MOTION_TYPE_H_ */
