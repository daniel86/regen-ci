#include "motion-type.h"

#include "regen/utility/logging.h"

using namespace regen;

std::ostream &regen::operator<<(std::ostream &out, const MotionType &v) {
	switch (v) {
		case MotionType::IDLE:
			return out << "IDLE";
		case MotionType::WALK:
			return out << "WALK";
		case MotionType::RUN:
			return out << "RUN";
		case MotionType::JUMP:
			return out << "JUMP";
		case MotionType::SIT:
			return out << "SIT";
		case MotionType::AGREE:
			return out << "AGREE";
		case MotionType::DISAGREE:
			return out << "DISAGREE";
		case MotionType::ATTACK:
			return out << "ATTACK";
		case MotionType::BLOCK:
			return out << "BLOCK";
		case MotionType::CROUCH:
			return out << "CROUCH";
		case MotionType::PRAY:
			return out << "PRAY";
		case MotionType::SLEEP:
			return out << "SLEEP";
		case MotionType::DIE:
			return out << "DIE";
		case MotionType::REVIVE:
			return out << "REVIVE";
		case MotionType::MOTION_LAST:
			return out << "MOTION_LAST";
	}
	return out;
}

std::istream &regen::operator>>(std::istream &in, MotionType &v) {
	std::string val;
	in >> val;
	boost::to_upper(val);
	if (val == "IDLE") v = MotionType::IDLE;
	else if (val == "WALK") v = MotionType::WALK;
	else if (val == "RUN") v = MotionType::RUN;
	else if (val == "JUMP") v = MotionType::JUMP;
	else if (val == "SIT") v = MotionType::SIT;
	else if (val == "AGREE" || val == "YES" || val == "NOD") v = MotionType::AGREE;
	else if (val == "DISAGREE" || val == "NO" || val == "SHAKE") v = MotionType::DISAGREE;
	else if (val == "ATTACK" || val == "FIGHT") v = MotionType::ATTACK;
	else if (val == "BLOCK" || val == "SHIELD") v = MotionType::BLOCK;
	else if (val == "CROUCH" || val == "SNEAK") v = MotionType::CROUCH;
	else if (val == "PRAY" || val == "PRAYING" || val == "WORSHIP") v = MotionType::PRAY;
	else if (val == "SLEEP" || val == "SLEEPING") v = MotionType::SLEEP;
	else if (val == "DIE" || val == "FALL") v = MotionType::DIE;
	else if (val == "REVIVE" || val == "STAND_UP" || val == "GET_UP") v = MotionType::REVIVE;
	else if (val == "MOTION_LAST") v = MotionType::MOTION_LAST;
	else {
		REGEN_WARN("Unknown action type '" << val << "'. Using IDLE.");
		v = MotionType::IDLE;
	}
	return in;
}
