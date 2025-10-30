#include "world-model.h"

using namespace regen;

WayPoint* WorldModel::getClosestTo(const WayPoint &other, const std::vector<const WayPoint*> &excludeWayPoints) {
	WayPoint* closest = nullptr;
	float minDistSq = std::numeric_limits<float>::max();
	Vec3f otherPos = other.position3D();
	for (const auto &wp: wayPoints) {
		if (wp.get() == &other) continue;
		if (std::find(excludeWayPoints.begin(), excludeWayPoints.end(), wp.get()) != excludeWayPoints.end()) {
			continue;
		}
		Vec3f wpPos = wp->position3D();
		float distSq = (wpPos - otherPos).lengthSquared();
		if (distSq < minDistSq) {
			minDistSq = distSq;
			closest = wp.get();
		}
	}
	return closest;
}
