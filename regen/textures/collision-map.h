#ifndef REGEN_COLLISION_MAP_H_
#define REGEN_COLLISION_MAP_H_

#include <string>
#include <iostream>

namespace regen {
	/**
	 * \brief Type of collision map.
	 * A collision map can either be a scalar field (e.g. a distance field)
	 * or a vector field (e.g. a flow field).
	 */
	enum CollisionMapType {
		COLLISION_SCALAR,
		COLLISION_VECTOR_FIELD
	};

	std::ostream &operator<<(std::ostream &out, const CollisionMapType &v);

	std::istream &operator>>(std::istream &in, CollisionMapType &v);
} // namespace

#endif /* REGEN_COLLISION_MAP_H_ */
