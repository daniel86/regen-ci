#include "world-model.h"

using namespace regen;

std::ostream &regen::operator<<(std::ostream &out, const PlaceType &v) {
	switch (v) {
		case PLACE_GATHERING:
			return out << "GATHERING";
		case PLACE_HOME:
			return out << "HOME";
		case PLACE_PATROL_POINT:
			return out << "PATROL_POINT";
		case PLACE_SPIRITUAL:
			return out << "SPIRITUAL";
	}
	return out;
}

std::istream &regen::operator>>(std::istream &in, PlaceType &v) {
	std::string val;
	in >> val;
	boost::to_upper(val);
	if (val == "GATHERING") v = PLACE_GATHERING;
	else if (val == "HOME") v = PLACE_HOME;
	else if (val == "PATROL_POINT" || val == "PATROL") v = PLACE_PATROL_POINT;
	else if (val == "SPIRITUAL") v = PLACE_SPIRITUAL;
	else {
		REGEN_WARN("Unknown place type '" << val << "'. Using HOME.");
		v = PLACE_HOME;
	}
	return in;
}

std::ostream &regen::operator<<(std::ostream &out, const AffordanceType &v) {
	switch (v) {
		case AffordanceType::NONE:
			return out << "NONE";
		case AffordanceType::SIT:
			return out << "SIT";
		case AffordanceType::SLEEP:
			return out << "SLEEP";
		case AffordanceType::PRAY:
			return out << "PRAY";
		case AffordanceType::CONVERSE:
			return out << "CONVERSE";
		case AffordanceType::OBSERVE:
			return out << "OBSERVE";
	}
	return out;
}

std::istream &regen::operator>>(std::istream &in, AffordanceType &v) {
	std::string val;
	in >> val;
	boost::to_upper(val);
	if (val == "SIT") v = AffordanceType::SIT;
	else if (val == "SLEEP") v = AffordanceType::SLEEP;
	else if (val == "PRAY") v = AffordanceType::PRAY;
	else if (val == "CONVERSE" || val == "TALK") v = AffordanceType::CONVERSE;
	else if (val == "OBSERVE" || val == "WATCH") v = AffordanceType::OBSERVE;
	else {
		REGEN_WARN("Unknown affordance type '" << val << "'. Using NONE.");
		v = AffordanceType::NONE;
	}
	return in;
}

std::ostream &regen::operator<<(std::ostream &out, const PathWayType &v) {
	switch (v) {
		case PathWayType::STROLL:
			return out << "STROLL";
		case PathWayType::PATROL:
			return out << "PATROL";
	}
	return out;
}

std::istream &regen::operator>>(std::istream &in, PathWayType &v) {
	std::string val;
	in >> val;
	boost::to_upper(val);
	if (val == "STROLL") v = PathWayType::STROLL;
	else if (val == "PATROL") v = PathWayType::PATROL;
	else {
		REGEN_WARN("Unknown path way type '" << val << "'. Using STROLL.");
		v = PathWayType::STROLL;
	}
	return in;
}
