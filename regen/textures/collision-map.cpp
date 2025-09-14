#include "collision-map.h"
#include "regen/utility/logging.h"
#include <boost/algorithm/string.hpp>

using namespace regen;

std::ostream &regen::operator<<(std::ostream &out, const CollisionMapType &v) {
	switch (v) {
		case COLLISION_SCALAR:
			return out << "SCALAR";
		case COLLISION_VECTOR_FIELD:
			return out << "VECTOR_FIELD";
	}
	return out;
}

std::istream &regen::operator>>(std::istream &in, CollisionMapType &v) {
	std::string val;
	in >> val;
	boost::to_upper(val);
	if (val == "SCALAR") v = COLLISION_SCALAR;
	else if (val == "VECTOR_FIELD" || val == "VECTORFIELD" || val == "VECTOR") v = COLLISION_VECTOR_FIELD;
	else {
		REGEN_WARN("Unknown collision map type '" << val << "'. Using SCALAR.");
		v = COLLISION_SCALAR;
	}
	return in;
}
