#ifndef REGEN_MODEL_ACTION_TYPE_H_
#define REGEN_MODEL_ACTION_TYPE_H_

#include <iostream>

namespace regen {
	enum class ActionType {
		IDLE = 0,
		CONVERSING,
		STROLLING,
		OBSERVING,
		PATROLLING,
		PRAYING,
		SLEEPING,
		ATTACKING,
		BLOCKING,
		FLEEING,
		TRAVELING,
		SITTING
	};

} // namespace

#endif /* REGEN_MODEL_ACTION_TYPE_H_ */
