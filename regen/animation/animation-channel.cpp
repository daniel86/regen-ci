#include "animation-channel.h"
#include "regen/utility/logging.h"

namespace regen {
	std::ostream &operator<<(std::ostream &out, const AnimationChannelBehavior &mode) {
		switch (mode) {
			case AnimationChannelBehavior::DEFAULT:
				return out << "DEFAULT";
			case AnimationChannelBehavior::CONSTANT:
				return out << "CONSTANT";
			case AnimationChannelBehavior::LINEAR:
				return out << "LINEAR";
			case AnimationChannelBehavior::REPEAT:
				return out << "REPEAT";
		}
		return out;
	}

	std::istream &operator>>(std::istream &in, AnimationChannelBehavior &mode) {
		std::string val;
		in >> val;
		boost::to_upper(val);
		if (val == "DEFAULT") mode = AnimationChannelBehavior::DEFAULT;
		else if (val == "CONSTANT") mode = AnimationChannelBehavior::CONSTANT;
		else if (val == "LINEAR") mode = AnimationChannelBehavior::LINEAR;
		else if (val == "REPEAT") mode = AnimationChannelBehavior::REPEAT;
		else {
			REGEN_WARN("Unknown animation behavior '" << val << "'. Using default DEFAULT behavior.");
			mode = AnimationChannelBehavior::DEFAULT;
		}
		return in;
	}
}
