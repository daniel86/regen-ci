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
		WayPoint(std::string_view name, const Vec3f &position) :
			WorldObject(name, position) {}

		WayPoint(std::string_view name, const ref_ptr<ShaderInput3f> &position) :
			WorldObject(name, position) {}

		explicit WayPoint(const ref_ptr<WorldObject> &obj) :
			WorldObject(obj->name(), obj->position3D()) {
			setRadius(obj->radius());
		}
	};

	std::ostream &operator<<(std::ostream &out, const PathwayType &v);

	std::istream &operator>>(std::istream &in, PathwayType &v);
} // namespace

#endif /* REGEN_MODEL_PATHWAY_H_ */
