#ifndef REGEN_PERCEPTION_DATA_H_
#define REGEN_PERCEPTION_DATA_H_
#include "regen/shapes/bounding-shape.h"

namespace regen {
	/**
	 * Data structure for perception events.
	 */
	struct PerceptionData {
		const BoundingShape *self = nullptr;
		// The perceived shape.
		const BoundingShape *other = nullptr;
		// The distance between the perceived shape and the perceiver.
		float distance = 0.0f;
		// The vector from the perceived shape to the perceiver.
		Vec3f delta = Vec3f::zero();
		// The normalized direction from the perceived shape to the perceiver.
		Vec3f dir = Vec3f::zero();
		// Whether the perceived shape is behind the perceiver.
		bool isBehind = false;
	};
} // namespace

#endif /* REGEN_PERCEPTION_DATA_H_ */

