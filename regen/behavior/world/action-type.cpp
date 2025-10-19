#include "action-type.h"

#include "regen/utility/logging.h"

using namespace regen;

std::ostream &regen::operator<<(std::ostream &out, const ActionType &v) {
	switch (v) {
		case ActionType::IDLE:
			return out << "IDLE";
		case ActionType::CONVERSING:
			return out << "CONVERSE";
		case ActionType::STROLLING:
			return out << "STROLL";
		case ActionType::OBSERVING:
			return out << "OBSERVE";
		case ActionType::PATROLLING:
			return out << "PATROL";
		case ActionType::PRAYING:
			return out << "PRAY";
		case ActionType::SLEEPING:
			return out << "SLEEP";
		case ActionType::ATTACKING:
			return out << "ATTACK";
		case ActionType::BLOCKING:
			return out << "BLOCK";
		case ActionType::FLEEING:
			return out << "FLEE";
		case ActionType::NAVIGATING:
			return out << "NAVIGATING";
		case ActionType::WALKING:
			return out << "WALK";
		case ActionType::SITTING:
			return out << "SIT";
		case ActionType::FLOCKING:
			return out << "FLOCK";
		case ActionType::LAST_ACTION:
			return out << "LAST_ACTION";
	}
	return out;
}

std::istream &regen::operator>>(std::istream &in, ActionType &v) {
	std::string val;
	in >> val;
	boost::to_upper(val);
	if (val == "IDLE") v = ActionType::IDLE;
	else if (val == "CONVERSE" || val == "TALK") v = ActionType::CONVERSING;
	else if (val == "STROLL") v = ActionType::STROLLING;
	else if (val == "PATROL") v = ActionType::PATROLLING;
	else if (val == "FLEE" || val == "RUN") v = ActionType::FLEEING;
	else if (val == "NAVIGATE" || val == "GO") v = ActionType::NAVIGATING;
	else if (val == "WALK") v = ActionType::WALKING;
	else if (val == "SIT" || val == "SITTING") v = ActionType::SITTING;
	else if (val == "BLOCK") v = ActionType::BLOCKING;
	else if (val == "ATTACK" || val == "FIGHT") v = ActionType::ATTACKING;
	else if (val == "OBSERVE" || val == "WATCH") v = ActionType::OBSERVING;
	else if (val == "SLEEP" || val == "SLEEPING") v = ActionType::SLEEPING;
	else if (val == "PRAY" || val == "PRAYING") v = ActionType::PRAYING;
	else if (val == "FLOCK" || val == "FLOCKING") v = ActionType::FLOCKING;
	else {
		REGEN_WARN("Unknown action type '" << val << "'. Using IDLE.");
		v = ActionType::IDLE;
	}
	return in;
}
