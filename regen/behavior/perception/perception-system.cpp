#include "perception-system.h"

#include "regen/shapes/obb.h"

using namespace regen;

PerceptionSystem::PerceptionSystem(
			const ref_ptr<SpatialIndex> &spatialIndex,
			const ref_ptr<BoundingShape> &indexedShape) :
	spatialIndex_(spatialIndex),
	indexedShape_(indexedShape)
{
	auto &mesh = indexedShape_->baseMesh();
	auto &tf = indexedShape_->transform();
	perceptionData_.self = indexedShape_.get();

	// create the initial collision shape
	// TODO: Reconsider form of shape for perception.
	//     - best: frustum, but we might get away with a box or sphere.
	float lookAheadDistance = 15.0f;
	Bounds<Vec3f> collisionBounds(mesh->minPosition(), mesh->maxPosition());
	collisionBounds.min.z -= lookAheadDistance;
	collisionBounds.min.x -= 0.0f;
	collisionBounds.max.x += 0.0f;
	collisionBounds.min.y -= 1.0f;
	collisionBounds.max.y += 1.0f;
	collisionShape_ = ref_ptr<OBB>::alloc(collisionBounds);
	collisionShape_->setTransform(tf, indexedShape->instanceID());
	collisionShape_->updateTransform(true);
	spatialIndex_->addDebugShape(collisionShape_);
}

void PerceptionSystem::addMonitor(PerceptionMonitor *monitor) {
	monitors_.push_back(monitor);
}

void PerceptionSystem::removeMonitor(PerceptionMonitor *monitor) {
	auto it = std::find(monitors_.begin(), monitors_.end(), monitor);
	if (it != monitors_.end()) {
		monitors_.erase(it);
	}
}

static void handleIntersectionStatic(const BoundingShape &other, void *data) {
	static_cast<PerceptionSystem*>(data)->handleIntersection(other, data);
}

void PerceptionSystem::handleIntersection(const BoundingShape &other, void *userData) {
	if (indexedShape_.get() == &other) return; // skip self
	const Vec3f &thisCenter = indexedShape_->tfOrigin();
	const Vec3f &otherCenter = other.tfOrigin();

	// Compute perception data.
	perceptionData_.other = &other;
	perceptionData_.delta = thisCenter - otherCenter;
	perceptionData_.distance = perceptionData_.delta.length();
	perceptionData_.dir = perceptionData_.delta / (perceptionData_.distance + 1e-6f);
	perceptionData_.isBehind = (Vec3f::front().dot(perceptionData_.delta) < 0.0f);

	// TODO: Also make a quick distance check for perception range to avoid false positives.
	if (perceptionData_.distance > 0.01f) {
		// Notify all monitors.
		for (auto &monitor : monitors_) {
			monitor->callOnPerception(perceptionData_);
		}
	}
}

void PerceptionSystem::update(double dt_s) {
	// Initialize monitors
	for (auto &monitor : monitors_) {
		monitor->callOnPerceptionInit();
	}
	// Update the collision shape transform
	collisionShape_->updateTransform(false);
	spatialIndex_->foreachIntersection(
		*collisionShape_.get(),
		handleIntersectionStatic,
		this,
		collisionMask_);
	// Cleanup monitors
	for (auto &monitor : monitors_) {
		monitor->callOnPerceptionCleanup();
	}
}
