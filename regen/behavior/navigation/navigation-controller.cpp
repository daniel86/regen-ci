#include <random>
#include "navigation-controller.h"

#define USE_HEIGHT_SMOOTHING
#define USE_DYNAMIC_LOOKAHEAD
#define USE_PUSH_THROUGH_OBSTACLES

using namespace regen;

static void initCollisionFrame_s(void *userData) {
	// set avoidance force to zero
	((NavigationController*)userData)->initCollisionFrame();
}

static void finalizeCollisionFrame_s(void *userData) {
	// finalize avoidance after all perception events have been handled
	((NavigationController*)userData)->finalizeCollisionFrame();
}

static void handlePerception_s(const CollisionEvent &evt, void *userData) {
	// accumulate avoidance forces
	((NavigationController*)userData)->handleCollisionEvent(evt);
}

NavigationController::NavigationController(
	const Indexed<ref_ptr<ModelTransformation>> &tfIndexed,
	const ref_ptr<WorldModel> &world)
	: TransformAnimation(tfIndexed.value, tfIndexed.index),
	  CollisionMonitor(handlePerception_s, initCollisionFrame_s, finalizeCollisionFrame_s, this),
	  worldModel_(world) {
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

void NavigationController::initCollisionFrame() {
#ifdef USE_DYNAMIC_LOOKAHEAD
	// Dynamically decrease the lookahead distance when close to the goal.
	// This allows to better approach the goal without being pushed away by
	// things that are located at the goal.
	dynLookAheadDistance_ = std::max(personalSpace_, lookAheadDistance_ *
		std::clamp(distanceToTarget_ / lookAheadThreshold_, 0.0f, 1.0f));
#endif
	navAvoidance_ = Vec3f::zero();
	navCollisionCount_ = 0;
	hasNewPerception_ = true;
}

void NavigationController::finalizeCollisionFrame() {
	if (navCollisionCount_ > 0) {
		navAvoidance_ /= static_cast<float>(navCollisionCount_);
		avoidanceStrength_ = navAvoidance_.length();
	} else {
		avoidanceStrength_ = 0.0f;
	}
}

void NavigationController::handleCollisionEvent(const CollisionEvent &evt) {
	if (patientShape_.get() == evt.data.other) return; // skip patient
	auto *otherRes = evt.data.other->worldObject();
	bool isStaticObject = true;
	if (otherRes) {
		auto otherWO = static_cast<const WorldObject*>(otherRes);
		isStaticObject = otherWO->isStatic();
	}
	if (isStaticObject) {
		handleStaticCollision(evt.data);
	} else {
		handleCharacterCollision(evt.data);
	}
}

void NavigationController::handleCharacterCollision(const CollisionData &percept) {
	// Note: percept.distance measures center-to-center distance.
	if (percept.distance < personalSpace_ * 3.0f) {
		// The other character is close by, steer away.
		// Use inverse square law for strength falloff.
		float distSqInv = 1.0f / (percept.distance * percept.distance);
		// scale to reasonable values
		distSqInv *= personalSpace_ * 10.0f;
		navAvoidance_.x += percept.dir.x * distSqInv;
		navAvoidance_.z += percept.dir.z * distSqInv;
		navCollisionCount_++;
	}
}

void NavigationController::handleStaticCollision(const CollisionData &percept) {
#ifdef USE_DYNAMIC_LOOKAHEAD
	float influenceRadius = dynLookAheadDistance_;
#else
	float influenceRadius = lookAheadDistance_;
#endif
	const Vec3f &otherCenter3D = percept.other->tfOrigin();
	const Vec3f &selfCenter3D = percept.self->tfOrigin();

	// Find the closest point on the surface of the other shape.
	// It could be a huge shape, so we can't just use center-to-center direction.
	// Note: closestPointOnSurface() returns a point in world space.
	// Note: If we are very close to the wall, closestDir might be numerically unstable,
	//   so we need to handle this case.
	const Vec3f closest = percept.other->closestPointOnSurface(selfCenter3D);
	Vec3f closestDir = selfCenter3D - closest;
	const float closestDistance = closestDir.length();
	if (closestDistance > influenceRadius) return;
	if (closestDistance < 1e-4f) {
		// We are extremely close to the wall, use center-to-center direction instead.
		closestDir = percept.dir;
	} else {
		closestDir /= closestDistance;
	}

	// Compute direction from the closest point on the surface to the shape center.
	Vec3f innerWallDir = closest - otherCenter3D;
	innerWallDir.y = 0.0f;
	innerWallDir.normalize();
	float innerAngle = closestDir.dot(innerWallDir);
	if (innerAngle < 0.0f) {
		// We are inside the wall, use center direction instead.
		closestDir = percept.dir;
		innerAngle *= -1.0f;
	}

	// Compute tangent blend factor based on angle between delta and innerWallDelta. Here we set
	// tangent max influence if the approach direction coincides with dir between the closest
	// surface point and shape origin.
	const float tanBlend = std::clamp(
		wallTangentWeight_ + 0.5f*innerAngle, 0.0f, 1.0f);
	Vec3f avoidance = Vec3f(-closestDir.z, 0.0f, closestDir.x);
	if (avoidance.dot(currentVel_) < 0.0f) {
		// Pick the tangent handedness that goes in the direction of current velocity
		avoidance = -avoidance;
	}
	avoidance = closestDir * (1.0f - tanBlend) + avoidance * tanBlend;
	avoidance.normalize();
	// Scale avoidance strength based on distance.
	// Here we use a simple linear falloff.
	avoidance *= influenceRadius / (0.1f + closestDistance);
	// Other falloff options:
	//avoidance *= std::clamp(influenceRadius / (0.3f + dist), 0.0f, maxAvoidance_);
	//avoidance *= std::max(0.0f, 1.0f - dist / influenceRadius) * 10.0f;
	//avoidance *= expf(-dist * dist / (2.0f * sigma * sigma));

	// Accumulate avoidance vector.
	navAvoidance_ += avoidance;
	navCollisionCount_++;
}

void NavigationController::updateControllerVelocity(double bezierTime) {
	float desiredSpeed = (isWalking_ ? walkSpeed_ : runSpeed_);

	auto bezierSample = bezierPath_.sample(bezierTime);
	// Desired velocity toward Bezier point
	Vec3f desiredDir(
		bezierSample.x - currentPos_.x,
		0.0f,
		bezierSample.y - currentPos_.z);
	float tmpLen = desiredDir.length();
	if (tmpLen < 1e-4f) { return; }
	desiredDir /= tmpLen;

#if 0
	// If we got off rails a lot, do an early re-planning of the path.
	// This can happen when we get stuck due to collisions.
	// TODO: At least would need to get a "cooldown" to avoid too frequent re-planning
	//   when we are in a really bad spot.
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
#endif

	// Compute desired velocity by blending bezier tangent with avoidance.
	Vec3f desiredVel;
	if (avoidanceStrength_ > 1e-4f) {
		// Blend desired with avoidance vector to avoid collisions.
		desiredVel = (desiredDir + navAvoidance_ * avoidanceWeight_);
		tmpLen = desiredVel.length();
		if (tmpLen < 1e-4f) {
			desiredVel = desiredDir;
		} else {
			desiredVel /= tmpLen;
		}
		// Slow down when avoidance is strong.
		// TODO: We could instead try to only reduce the velocity component that
		//       points into the avoidance direction.
		desiredVel *= desiredSpeed * std::clamp(1.0f - avoidanceStrength_*0.05f, 0.5f, 1.0f);
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
	bool isAtLastWP = (currentPathIndex_ + 1 == currentPath_.size());
	if (isAtLastWP) {
		float distToGoal = (bezierPath_.p3 - Vec2f(currentPos_.x, currentPos_.z)).lengthSquared();
		float approachThresholdSq = 5.0f * 5.0f;
		float goalProximity = std::clamp(1.0f - distToGoal / approachThresholdSq, 0.0f, 1.0f);
		velBlend *= (1.0f - goalProximity);
		// allow faster turning when close to goal
		maxDeltaThisFrame += 2.0f * goalProximity * maxDeltaThisFrame;
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
#ifdef USE_PUSH_THROUGH_OBSTACLES
	if (distanceToTarget_ < pushThroughDistance_) {
		// close to goal, no avoidance just push through.
		navAvoidance_ = Vec3f::zero();
	}
#endif
	if (hasNewPerception_) {
		hasNewPerception_ = false;
	} else {
		// Blend-out avoidance with velocity over time when no new perception frame arrived.
		// This avoids that avoidance "sticks" too long.
		navAvoidance_ *= std::max(0.0f, 1.0f - avoidanceDecay_ * static_cast<float>(lastDT_));
	}
	if (currentFrame.pos.has_value()) {
		updateControllerVelocity(bezierTime);
	}
	updateControllerOrientation(bezierTime);
}
