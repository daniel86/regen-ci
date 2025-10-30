#ifndef REGEN_MODEL_PATHWAY_H_
#define REGEN_MODEL_PATHWAY_H_

#include <iostream>
#include "world-object.h"

namespace regen {
	enum class PathwayType {
		STROLL = 0,
		PATROL,
	};

	class WayPoint : public WorldObject {
	public:
		explicit WayPoint(std::string_view name) : WorldObject(name) {}

		WayPoint(std::string_view name, const Vec3f &position) : WorldObject(name) {
			setPosition(position);
		}

		explicit WayPoint(const ref_ptr<WorldObject> &obj) : WorldObject(obj->name()) {
			setRadius(obj->radius());
			setPosition(obj->position3D());
		}
	};

	std::ostream &operator<<(std::ostream &out, const PathwayType &v);

	std::istream &operator>>(std::istream &in, PathwayType &v);
} // namespace

#endif /* REGEN_MODEL_PATHWAY_H_ */
