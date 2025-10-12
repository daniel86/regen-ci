#include <random>
#include "navigation-controller.h"
#include "regen/shapes/obb.h"

#define USE_HEIGHT_SMOOTHING
#define USE_DYNAMIC_LOOKAHEAD
#define USE_PUSH_THROUGH_OBSTACLES

using namespace regen;

NavigationController::NavigationController(
	const ref_ptr<Mesh> &mesh,
	const Indexed<ref_ptr<ModelTransformation>> &tfIndexed,
	const ref_ptr<WorldModel> &world)
	: TransformAnimation(tfIndexed.value, tfIndexed.index),
	  worldModel_(world) {
	// create the initial collision shape
	Bounds<Vec3f> collisionBounds(mesh->minPosition(), mesh->maxPosition());
	collisionBounds.min.z -= lookAheadDistance_;
	collisionBounds.min.x -= 0.0f;
	collisionBounds.max.x += 0.0f;
	collisionBounds.min.y -= 1.0f;
	collisionBounds.max.y += 1.0f;
	collisionShape_ = ref_ptr<OBB>::alloc(collisionBounds);
	collisionShape_->setTransform(tfIndexed.value, tfIndexed.index);
	collisionShape_->updateTransform(true);
}

void NavigationController::setSpatialIndex(const ref_ptr<SpatialIndex> &spatialIndex, std::string_view shapeName) {
	spatialIndex_ = spatialIndex;
	indexedShape_ = spatialIndex_->getShape(shapeName, tfIdx_);
	if (!indexedShape_.get()) {
		REGEN_WARN("Unable to find indexed shape '" << shapeName << "' in spatial index.");
	}
	spatialIndex_->addDebugShape(collisionShape_);
}

void NavigationController::setHeightMap(const ref_ptr<HeightMap> &heightMap) {
	heightMap_ = heightMap;
	heightMap_->ensureTextureData();
}

float NavigationController::getHeight(const Vec2f &pos) {
	if (heightMap_.get()) {
		return heightMap_->sampleHeight(pos) - 0.25f; // offset a bit to avoid floating
	} else {
		return floorHeight_;
	}
}

static inline float wrapPi(float a) {
	// normalize to (-PI, PI]
	while (a <= -M_PI) a += 2.0f * M_PI;
	while (a > M_PI) a -= 2.0f * M_PI;
	return a;
}

void NavigationController::updatePathCurve(const Vec2f &source, const Vec2f &target) {
	// Direction from source to target
	Vec2f dir = target - source;
	float len = dir.length();
	if (len < 1e-6f) {
		dir = Vec2f(cos(currentDir_.x + baseOrientation_), sin(currentDir_.x + baseOrientation_));
	} else {
		dir /= len;
	}

	// Mix with orientation of next waypoint, if any.
	if (currentPathIndex_ + 2 <= currentPath_.size()) {
		Vec2f nextPos = currentPath_[currentPathIndex_ + 1]->position2D();
		Vec2f nextDir = nextPos - target;
		float nextLen = nextDir.length();
		if (nextLen > 1e-6f) {
			nextDir /= nextLen;
			// blend directions
			nextDir = math::lerp(nextDir, nextDir, 0.25f);
			float nextDirLen = nextDir.length();
			if (nextDirLen > 1e-6f) {
				dir = nextDir / nextDirLen;
			}
		}
	}

	float angle = atan2(dir.y, dir.x) - baseOrientation_;
	updatePathCurve(source, target, Vec3f(angle, 0.0f, 0.0f));
}

void NavigationController::updateTransformFrame(const Vec2f &target, const Vec3f &desiredDir) {
	float bezierLength = bezierLUT_.totalLength;
	// compute dt based on distance and speed
	float dt = bezierLength / (isWalking_ ? walkSpeed_ : runSpeed_);
	// set the target position
	frames_.clear();
	TransformAnimation::push_back(
		Vec3f(target.x, getHeight(target), target.y),
		desiredDir,
		dt);
	if (!isRunning()) { startAnimation(); }
}

void NavigationController::updatePathCurve(
		const Vec2f &source,
		const Vec2f &target,
		const Vec3f &desiredDir) {
	bezierPath_.p0 = source;
	bezierPath_.p3 = target;
	// compute control points using Euler angles
	float directDistance = (bezierPath_.p0 - bezierPath_.p3).length();
	// add base orientation
	float angle_rad1 = currentDir_.x + baseOrientation_;
	float angle_rad2 = desiredDir.x + baseOrientation_;
	float angleDiff = wrapPi(angle_rad2 - angle_rad1);
	// shorter handles for sharper turns
	float handleScale = directDistance * 0.5f * (1.0f - 0.5f * std::abs(angleDiff) / M_PI);
	//float handleScale = directDistance * 0.25f * (1.0f - 0.5f * std::abs(angleDiff) / M_PI);
	// p1: in direction of start orientation
	// - Allow sharper turns by using a shorter handle. e.g. in case NPC makes a 180deg turn.
	bezierPath_.p1 = bezierPath_.p0 + Vec2f(cos(angle_rad1), sin(angle_rad1)) * handleScale * 0.25f;
	// p2: in direction of target orientation
	bezierPath_.p2 = bezierPath_.p3 - Vec2f(cos(angle_rad2), sin(angle_rad2)) * handleScale;
	bezierLUT_ = bezierPath_.buildArcLengthLUT(numLUTEntries_);
	updateTransformFrame(target, desiredDir);
}

void NavigationController::handleCharacterCollision(const BoundingShape &other, NPCNeighborData *data) {
	const Vec3f &thisCenter3D = indexedShape_->tfOrigin();
	const Vec3f &otherCenter3D = other.tfOrigin();

	Vec2f thisCenter(thisCenter3D.x, thisCenter3D.z);
	Vec2f otherCenter = Vec2f(otherCenter3D.x, otherCenter3D.z);
	Vec2f delta = thisCenter - otherCenter;
	const float dist = std::max(0.0f, delta.length() - personalSpace_);
	if (dist < 0.01f) return;
	if (dist < personalSpace_ * 2.0f) {
		// inverse-square falloff
		delta /= dist;
		delta /= (dist * dist);
		data->avoidance.x += delta.x;
		data->avoidance.z += delta.y;
		data->neighborCount++;
	}
}

void NavigationController::handleWallCollision(const BoundingShape &other, NPCNeighborData *data) {
	const Vec3f &thisCenter3D = indexedShape_->tfOrigin();
	const Vec3f &otherCenter3D = other.tfOrigin();

	Vec3f closest = other.closestPointOnSurface(thisCenter3D);
	Vec3f delta = thisCenter3D - closest;
	delta.y = 0.0f; // ignore height differences

	const float dist = delta.length();
	if (dist < 0.01f) return;
#ifdef USE_DYNAMIC_LOOKAHEAD
	const float influenceRadius = data->lookAhead;
#else
	const float influenceRadius = lookAheadDistance_;
#endif
	if (dist > influenceRadius) return;
	delta /= dist;

	Vec3f delta2 = closest - otherCenter3D;
	delta2.y = 0.0f;
	delta2.normalize();
	// Compute tangent blend factor based on angle between delta and delta2. Here we set
	// tangent max influence if the approach direction coincides with dir between the closest
	// surface point and shape origin.
	float tanBlend = std::clamp(wallTangentWeight_ + 0.5f * std::abs(delta.dot(delta2)), 0.0f, 1.0f);
	Vec3f tangent = Vec3f(-delta.z, 0.0f, delta.x);
	if (tangent.dot(currentVel_) < 0.0f) {
		// Pick the tangent handedness that goes in the direction of current velocity
		tangent = -tangent;
	}
	Vec3f combined = delta * (1.0f - tanBlend) + tangent * tanBlend;
	combined.normalize();

	//const float falloff = std::max(0.0f, 1.0f - dist / influenceRadius) * 10.0f;
	//float falloff = expf(-dist * dist / (2.0f * sigma * sigma));
	float falloff = (influenceRadius / (0.1f + dist));
	// Cap falloff
	// falloff = std::clamp(influenceRadius / (0.3f + dist), 0.0f, maxAvoidance_);

	data->avoidance += combined * falloff;
	data->neighborCount++;
}

void NavigationController::handleNeighbourStatic(const BoundingShape &other, void *userData) {
	NPCNeighborData *data = static_cast<NPCNeighborData *>(userData);
	data->npc->handleNeighbour(other, data);
}

void NavigationController::handleNeighbour(const BoundingShape &other, void *userData) {
	if (indexedShape_.get() == &other) return; // skip self
	if (patientShape_.get() == &other) return; // skip patient
	NPCNeighborData *data = static_cast<NPCNeighborData *>(userData);

	// TODO: improve this, currently only other instances of same mesh are recognized as NPCs
	const bool isCollisionWithNPC = (other.baseMesh().get() == indexedShape_->baseMesh().get());

	if (isCollisionWithNPC) {
		data->npc->handleCharacterCollision(other, data);
	} else { // static object
		data->npc->handleWallCollision(other, data);
	}
}

Vec3f NavigationController::computeNeighborAvoidance() {
#ifdef USE_PUSH_THROUGH_OBSTACLES
	if (distanceToTarget_ < pushThroughDistance_) {
		// close to goal, no avoidance just push through.
		return Vec3f::zero();
	}
#endif
	// TODO: We could accumulate progress in distance to goal, and react in case
	//         we are stuck (i.e. no progress for some time).

	NPCNeighborData data;
	data.npc = this;
#ifdef USE_DYNAMIC_LOOKAHEAD
	// Dynamically decrease the lookahead distance when close to the goal.
	// This allows to better approach the goal without being pushed away by
	// things that are located at the goal.
	data.lookAhead = std::max(personalSpace_, lookAheadDistance_ *
		std::clamp(distanceToTarget_ / lookAheadThreshold_, 0.0f, 1.0f));
#endif

	collisionShape_->updateTransform(false);
	if (spatialIndex_.get()) {
		spatialIndex_->foreachIntersection(
			*collisionShape_.get(),
			handleNeighbourStatic,
			&data,
			collisionMask_);
	}
	if (data.neighborCount > 0) {
		data.avoidance /= static_cast<float>(data.neighborCount);
	}

	return data.avoidance;
}

void NavigationController::updateControllerVelocity(double bezierTime) {
	const float desiredSpeed = (isWalking_ ? walkSpeed_ : runSpeed_);

	auto bezierSample = bezierPath_.sample(bezierTime);
	// Desired velocity toward Bezier point
	Vec3f desiredDir(
		bezierSample.x - currentPos_.x,
		0.0f,
		bezierSample.y - currentPos_.z);
	float tmpLen = desiredDir.length();
	if (tmpLen < 1e-4f) { return; }
	desiredDir /= tmpLen;

	// TODO: If we got off rails a lot, maybe it could be worth re-planning? Could
	//      check the distance here....
	/**
	const float replanThresholdSq = 100.0f;
	float distanceToBezier = (Vec2f(currentPos_.x, currentPos_.z) - bezierSample).lengthSquared();
	if (distanceToBezier > replanThresholdSq) {
		// We are too far away from the bezier curve, so re-plan.
		if (currentPathIndex_ < currentPath_.size()) {
			REGEN_WARN("[" << tfIdx_ << "] Re-planning path, distance to bezier: " << sqrtf(distanceToBezier) << "m");
			Vec2f target = currentPath_.back()->position2D();
			updatePathCurve(Vec2f(currentPos_.x, currentPos_.z), target);
			// and return, we will update velocity next frame
			return;
		}
	}
	**/

	// Compute avoidance from neighbors
	Vec3f avoidance = computeNeighborAvoidance();

	// Compute desired velocity by blending bezier tangent with avoidance.
	Vec3f desiredVel;
	float avoidanceStrength = avoidance.length();
	if (avoidanceStrength > 1e-4f) {
		// Blend desired with avoidance vector to avoid collisions.
		desiredVel = (desiredDir + avoidance * avoidanceWeight_);
		tmpLen = desiredVel.length();
		if (tmpLen < 1e-4f) {
			desiredVel = desiredDir;
		} else {
			desiredVel /= tmpLen;
		}
		desiredVel *= desiredSpeed * std::clamp(1.0f - avoidanceStrength*0.05f, 0.5f, 1.0f);
	} else {
		desiredVel = desiredDir * desiredSpeed;
	}

	// First assume current vel, then blend toward desired vel
	Vec3f velDir = currentVel_;
	tmpLen = velDir.length();
	if (tmpLen > 1e-4f) velDir /= tmpLen;

	// Update position by following the desired velocity
	float angleDiff = acos(std::clamp(desiredVel.dot(velDir), -1.0f, 1.0f));
	float blendFactor = std::clamp(0.1f + (angleDiff / M_PIf) * 0.4f, 0.1f, 0.5f);
	// Note: avoid rapid changes in velocity by blending with current vel
	currentVel_ = math::lerp(currentVel_, desiredVel, blendFactor);

	// Finally, advance the position
	currentPos_ += currentVel_ * lastDT_;
	float desiredHeight = getHeight(Vec2f(currentPos_.x, currentPos_.z));
#ifdef USE_HEIGHT_SMOOTHING
	// Smooth height changes over time.
	currentPos_.y = math::lerp(currentPos_.y, desiredHeight,
		std::clamp(lastDT_ * heightSmoothFactor_, 0.0, 1.0));
#else
	currentPos_.y = desiredHeight;
#endif
}

void NavigationController::updateControllerOrientation(double bezierTime) {
	const float desiredSpeed = (isWalking_ ? walkSpeed_ : runSpeed_);
	// compute path tangent (as 3D: x,z -> x,z)
	Vec2f bezierTan2 = bezierPath_.tangent(bezierTime);
	float tmpLen = bezierTan2.length();
	if (tmpLen > 1e-6f) bezierTan2 /= tmpLen;

	Vec3f tangentDir(bezierTan2.x, 0.0f, bezierTan2.y); // tangent.x -> world X, tangent.y -> world Z
	tmpLen = tangentDir.length();
	if (tmpLen > 1e-6f) tangentDir /= tmpLen;

	// compute velocity direction (if moving)
	Vec3f velDir(currentVel_.x, 0.0f, currentVel_.z);
	tmpLen = velDir.length();
	if (tmpLen > 1e-4f) velDir /= tmpLen;

	// Blend velocity direction and path tangent to form a stable target direction.
	// Idea: when moving fast, prefer velocity; when slow, prefer path tangent.
	float vWeight = std::clamp(tmpLen / (desiredSpeed + 1e-6f), 0.0f, 1.0f); // 0..1
	float velBlend = vWeight * velOrientationWeight_; // in [0,1]
	// clamp rotation speed (radians per second)
	float maxDeltaThisFrame = maxTurn_ * lastDT_;

#if 0
	// Reduce influence of velocity when close to target, and
	// when we are approaching the last waypoint of the current path.
	// TODO: this does not appear to work well, I think the rotation change makes the NPC overshoot its position
	//       often for some reason?
	bool isAtLastWP = (currentPathIndex_ + 1 == currentPath_.size());
	if (isAtLastWP) {
		float distToGoal = (bezierPath_.p3 - Vec2f(currentPos_.x, currentPos_.z)).lengthSquared();
		float approachThresholdSq = 5.0f * 5.0f;
		float goalProximity = std::clamp(1.0f - distToGoal / approachThresholdSq, 0.0f, 1.0f);
		velBlend *= (1.0f - goalProximity);
		// allow faster turning when close to goal
		maxDeltaThisFrame += goalProximity * maxTurn_ * 2.0f * static_cast<float>(lastDT_);
	}
#endif

	// compute blended direction
	Vec3f blendedDir;
	if (velDir.lengthSquared() > 1e-5f) {
		blendedDir = tangentDir * (1.0f - velBlend) + velDir * velBlend;
	} else {
		blendedDir = tangentDir;
	}
	// safety: fallback to tangent if blended is too small
	tmpLen = blendedDir.length();
	if (tmpLen < 1e-4f) {
		blendedDir = tangentDir;
	} else {
		blendedDir /= tmpLen;
	}

	// compute target yaw (world yaw)
	float targetYaw = atan2(blendedDir.z, blendedDir.x);
	// retrieve current stored yaw (undo baseOrientation_ offset that is stored in currentDir_.x)
	float prevYaw = currentDir_.x + baseOrientation_;
	// shortest angular difference
	float deltaYaw = wrapPi(targetYaw - prevYaw);
	if (deltaYaw > maxDeltaThisFrame) deltaYaw = maxDeltaThisFrame;
	else if (deltaYaw < -maxDeltaThisFrame) deltaYaw = -maxDeltaThisFrame;
	// apply the clamped rotation
	currentDir_.x = prevYaw + deltaYaw - baseOrientation_;
}

void NavigationController::updatePose(const TransformKeyFrame &currentFrame, double frameTime) {
	float bezierTime = math::Bezier<Vec2f>::lookupParameter(bezierLUT_, frameTime);
	if (currentFrame.pos.has_value()) {
		updateControllerVelocity(bezierTime);
	}
	updateControllerOrientation(bezierTime);
}
