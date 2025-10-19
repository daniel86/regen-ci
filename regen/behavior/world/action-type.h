#ifndef REGEN_MODEL_ACTION_TYPE_H_
#define REGEN_MODEL_ACTION_TYPE_H_

#include <iostream>

namespace regen {
	/**
	 * The types of actions a character can perform.
	 */
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
		NAVIGATING,
		// enforce walk animation without path based navigation.
		// e.g. flocking could inlclude walking, but not all the time.
		WALKING,
		SITTING,
		FLOCKING,
		LAST_ACTION
	};

	std::ostream &operator<<(std::ostream &out, const ActionType &v);

	std::istream &operator>>(std::istream &in, ActionType &v);
} // namespace

#endif /* REGEN_MODEL_ACTION_TYPE_H_ */
