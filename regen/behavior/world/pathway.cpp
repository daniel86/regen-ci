#include "pathway.h"

using namespace regen;

std::ostream &regen::operator<<(std::ostream &out, const PathwayType &v) {
	switch (v) {
		case PathwayType::STROLL:
			return out << "STROLL";
		case PathwayType::PATROL:
			return out << "PATROL";
	}
	return out;
}

std::istream &regen::operator>>(std::istream &in, PathwayType &v) {
	std::string val;
	in >> val;
	boost::to_upper(val);
	if (val == "STROLL") v = PathwayType::STROLL;
	else if (val == "PATROL") v = PathwayType::PATROL;
	else {
		REGEN_WARN("Unknown path way type '" << val << "'. Using STROLL.");
		v = PathwayType::STROLL;
	}
	return in;
}
