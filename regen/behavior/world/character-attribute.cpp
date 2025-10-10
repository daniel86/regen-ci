#include "character-attribute.h"
#include "regen/utility/logging.h"
#include <boost/algorithm/string.hpp>
#include <string>

using namespace regen;

std::ostream &regen::operator<<(std::ostream &out, const CharacterAttribute &v) {
	switch (v) {
		case CharacterAttribute::LAST:
		case CharacterAttribute::HEALTH:
			return out << "HEALTH";
		case CharacterAttribute::HUNGER:
			return out << "HUNGER";
		case CharacterAttribute::STAMINA:
			return out << "STAMINA";
		case CharacterAttribute::FEAR:
			return out << "FEAR";
	}
	return out;
}

std::istream &regen::operator>>(std::istream &in, CharacterAttribute &v) {
	std::string val;
	in >> val;
	boost::to_upper(val);
	if (val == "HEALTH") v = CharacterAttribute::HEALTH;
	else if (val == "HUNGER") v = CharacterAttribute::HUNGER;
	else if (val == "STAMINA") v = CharacterAttribute::STAMINA;
	else if (val == "FEAR") v = CharacterAttribute::FEAR;
	else {
		REGEN_WARN("Unknown character attribute '" << val << "'. Using HEALTH.");
		v = CharacterAttribute::HEALTH;
	}
	return in;
}
