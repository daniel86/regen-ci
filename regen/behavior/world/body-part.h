#ifndef REGEN_MODEL_BODY_PART_H_
#define REGEN_MODEL_BODY_PART_H_

#include <iostream>

namespace regen {
	enum class BodyPart {
		HEAD = 0,
		NECK,
		TORSO,
		ARM,
		LEG,
		LAST
	};

	std::ostream &operator<<(std::ostream &out, const BodyPart &v);

	std::istream &operator>>(std::istream &in, BodyPart &v);
} // namespace

#endif /* REGEN_MODEL_ACTION_TYPE_H_ */
