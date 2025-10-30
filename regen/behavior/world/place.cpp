#include "place.h"

#include "regen/utility/logging.h"

using namespace regen;

std::ostream &regen::operator<<(std::ostream &out, const PlaceType &v) {
	switch (v) {
		case PlaceType::LAST:
		case PlaceType::GATHERING:
			return out << "GATHERING";
		case PlaceType::HOME:
			return out << "HOME";
		case PlaceType::PATROL_POINT:
			return out << "PATROL_POINT";
		case PlaceType::SPIRITUAL:
			return out << "SPIRITUAL";
	}
	return out;
}

std::istream &regen::operator>>(std::istream &in, PlaceType &v) {
	std::string val;
	in >> val;
	boost::to_upper(val);
	if (val == "GATHERING") v = PlaceType::GATHERING;
	else if (val == "HOME") v = PlaceType::HOME;
	else if (val == "PATROL_POINT" || val == "PATROL") v = PlaceType::PATROL_POINT;
	else if (val == "SPIRITUAL") v = PlaceType::SPIRITUAL;
	else {
		REGEN_WARN("Unknown place type '" << val << "'. Using HOME.");
		v = PlaceType::HOME;
	}
	return in;
}
