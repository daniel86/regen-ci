#include "character-trait.h"
#include "regen/utility/logging.h"
#include <boost/algorithm/string.hpp>
#include <string>

using namespace regen;

std::ostream &regen::operator<<(std::ostream &out, const Trait &v) {
	switch (v) {
		case Trait::LAST_TRAIT:
		case Trait::LAZINESS:
			return out << "LAZINESS";
		case Trait::SPIRITUALITY:
			return out << "SPIRITUALITY";
		case Trait::ALERTNESS:
			return out << "ALERTNESS";
		case Trait::BRAVERY:
			return out << "BRAVERY";
		case Trait::SOCIALABILITY:
			return out << "SOCIALABILITY";
	}
	return out;
}

std::istream &regen::operator>>(std::istream &in, Trait &v) {
	std::string val;
	in >> val;
	boost::to_upper(val);
	if (val == "LAZINESS") v = Trait::LAZINESS;
	else if (val == "SPIRITUALITY") v = Trait::SPIRITUALITY;
	else if (val == "ALERTNESS") v = Trait::ALERTNESS;
	else if (val == "BRAVERY") v = Trait::BRAVERY;
	else if (val == "SOCIALABILITY" || val == "SOCIALITY" || val == "SOCIAL") v = Trait::SOCIALABILITY;
	else {
		REGEN_WARN("Unknown character trait '" << val << "'. Using LAZINESS.");
		v = Trait::LAZINESS;
	}
	return in;
}
