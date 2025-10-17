#include <random>
#include "navigation-controller.h"

#define USE_HEIGHT_SMOOTHING
#define USE_DYNAMIC_LOOKAHEAD

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
	: Animation(false, true),
	  CollisionMonitor(handlePerception_s, initCollisionFrame_s, finalizeCollisionFrame_s, this),
	  tf_(tfIndexed.value),
 	  tfIdx_(tfIndexed.index),
	  worldModel_(world) {
	auto currentTransform = tf_->modelMat()->getVertex(tfIdx_);
	setAnimationName(REGEN_STRING("animation-"<<tf_->modelMat()->name()));

	// initialize transform data
	currentPos_ = currentTransform.r.position();
	currentVal_ = currentTransform.r;
	initialScale_ = currentTransform.r.scaling();

	// remove scaling before computing rotation, else we get faulty results
	auto tmp = currentTransform.r;
	tmp.scale(Vec3f(
			1.0f / initialScale_.x,
			1.0f / initialScale_.y,
			1.0f / initialScale_.z));
	currentDir_ = tmp.rotation();
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

void NavigationController::stopNavigation() {
	navModeMask_ = NO_NAVIGATION;
	navGroup_ = {};
	currentPath_.clear();
	currentPathIndex_ = 0;
}

void NavigationController::startFlocking(const ref_ptr<ObjectGroup> &group) {
	navGroup_ = group;
	// set flocking mode flag
	navModeMask_ |= FLOCKING;
}

void NavigationController::stopFlocking() {
	navGroup_ = {};
	// clear flocking mode flag
	navModeMask_ &= ~FLOCKING;
}

void NavigationController::stopApproaching() {
	// clear approaching mode flag
	navModeMask_ &= ~APPROACHING;
}

void NavigationController::startApproaching(const Vec2f &source, const Vec2f &target) {
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
	startApproaching(source, target, Vec3f(angle, 0.0f, 0.0f));
}

void NavigationController::startApproaching(
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

	navModeMask_ |= APPROACHING;
	// reset approach time
	approachTime_ = 0.0;
	// compute expected duration based on distance and speed
	approachDuration_ = bezierLUT_.totalLength / (isWalking_ ? walkSpeed_ : runSpeed_);

	if (!isRunning()) { startAnimation(); }
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
		distSqInv *= personalSpace_ * characterAvoidance_;
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
	navAvoidance_ += avoidance * wallAvoidance_;
	navCollisionCount_++;
}

void NavigationController::updateControllerVelocity() {
	const Vec2f currentPos2D(currentPos_.x, currentPos_.z);
	float desiredSpeed = (isWalking_ ? walkSpeed_ : runSpeed_);
	float tmpLen;

	if (navModeMask_ & APPROACHING) {
		// Approaching target along Bezier curve
		auto bezierSample = bezierPath_.sample(curveTime_);
		desiredDir_.x = bezierSample.x - currentPos_.x;
		desiredDir_.y = 0.0f;
		desiredDir_.z = bezierSample.y - currentPos_.z;
	} else if (navModeMask_ & FLOCKING) {
		auto groupDir = navGroup_->groupCenter2D() - currentPos2D;
		desiredDir_.x = groupDir.x;;
		desiredDir_.y = 0.0f;
		desiredDir_.z = groupDir.y;
	}
	tmpLen = desiredDir_.length();
	if (tmpLen < 1e-4f) { return; }
	desiredDir_ /= tmpLen;
	currentForce_ = Vec3f::zero();

	Vec3f pushVec = navAvoidance_ * avoidanceWeight_;
	float pushStrength = avoidanceStrength_;

	if (navModeMask_ & APPROACHING) {
		desiredVel_ = desiredDir_ * desiredSpeed;
	} else {
		desiredVel_ = Vec3f::zero();
		pushVec *= 0.1f;
		pushStrength *= 0.1f;
	}

	if (navModeMask_ & FLOCKING) {
		// Desired dir toward group center and away from other group members.
		const float minGroupDistance = personalSpace_ * (0.5f + 0.25f * navGroup_->numMembers());
		Vec2f groupDir = navGroup_->groupCenter2D() - currentPos2D;
		tmpLen = groupDir.length();
		if (tmpLen > 1e-4f) groupDir /= tmpLen;

		// Spring-like cohesion force toward group center.
		Vec2f cohesion = groupDir * (tmpLen - minGroupDistance);

		/**
		float restR = desiredRadius;
		float dist = length(p - center);
		float error = dist - restR;
		Vec2f cohesion = (dist > 1e-4) ? ( (center - p) / dist * error * cohGain ) : Vec2f(0,0);
		// add damping: subtract k * (velocity dot radialDir) to prevent oscillation
		float radialVel = dot(vel, radialDir);
		cohesion -= radialVel * radialDamping;
		**/

		/**
		Vec2f locationDir = navGroup_->currentLocation()->position2D() - currentPos2D;
		float locationDist = locationDir.length();
		if (locationDist > 1e-4f) locationDir /= locationDist;
		// Add force to stay within location bounds.
		cohesion += locationDir * (std::max(0.0f,
			locationDist - navGroup_->currentLocation()->radius()) );
			**/

		// Separation from other group members
		// TODO: for some reason this makes collaps all in one point !?!
		Vec2f memberSeparation = Vec2f::zero();
		/**
		for (uint32_t memberIdx=0; memberIdx < navGroup_->numMembers(); memberIdx++) {
			const WorldObject *member = navGroup_->member(memberIdx);
			//if (member == self) continue;
			Vec2f toMember = currentPos2D - member->position2D();
			float distSq = toMember.length();
			if (distSq > 1e-4f && distSq < personalSpace_) {
				memberSeparation += toMember * (1.0f / distSq);
			}
		}
		memberSeparation /= static_cast<float>(navGroup_->numMembers());
		**/

		// Separation from other groups.
		Vec2f groupSeparation = Vec2f::zero();
		/**
		auto &navLocation = navGroup_->currentLocation();
		if (navLocation.get()) {
			for (int32_t groupIdx=0; groupIdx < navLocation->numGroups(); groupIdx++) {
				ObjectGroup *otherGroup = navLocation->group(groupIdx);
				if (otherGroup == navGroup_.get()) continue;
				Vec2f toGroup = currentPos2D - otherGroup->groupCenter2D();
				float distSq = toGroup.lengthSquared();
				if (distSq > 1e-4f && distSq < personalSpace_ * personalSpace_ * 4.0f) {
					groupSeparation += toGroup * (1.0f / distSq);
				}
			}
		}
		**/

		// Blend forces and compute desired direction.
		Vec2f groupForce = cohesion * cohesionWeight_ +
			memberSeparation * memberSeparationWeight_ +
			groupSeparation * groupSeparationWeight_;
		float groupStrength = groupForce.length();
		pushStrength += groupStrength;
		pushVec.x += groupForce.x;
		pushVec.z += groupForce.y;
	}

	// Compute desired velocity by blending desiredDir with avoidance.
	if (pushStrength > 1e-4f) {
		// Blend desired with avoidance vector to avoid collisions.
		desiredVel_ = (desiredDir_ + pushVec);
		tmpLen = desiredVel_.length();
		if (tmpLen < 1e-4f) {
			desiredVel_ = desiredDir_;
		} else {
			desiredVel_ /= tmpLen;
		}
		// Slow down when avoidance is strong.
		desiredVel_ *= desiredSpeed * std::clamp(1.0f - pushStrength*0.05f, 0.5f, 1.0f);
	} else {
		desiredVel_ = desiredDir_ * desiredSpeed;
	}

	// First assume current vel, then blend toward desired vel
	Vec3f velDir = currentVel_;
	tmpLen = velDir.length();
	if (tmpLen > 1e-4f) velDir /= tmpLen;

	// Update position by following the desired velocity
	float angleDiff = acos(std::clamp(desiredVel_.dot(velDir), -1.0f, 1.0f));
	float blendFactor = std::clamp(0.1f + (angleDiff / M_PIf) * 0.4f, 0.1f, 0.5f);
	// Note: avoid rapid changes in velocity by blending with current vel
	currentVel_ = math::lerp(currentVel_, desiredVel_, blendFactor);

	// Finally, advance the position
	currentPos_ += currentVel_ * lastDT_;
	float desiredHeight = getHeight(currentPos2D);
#ifdef USE_HEIGHT_SMOOTHING
	// Smooth height changes over time.
	currentPos_.y = math::lerp(currentPos_.y, desiredHeight,
		std::clamp(lastDT_ * heightSmoothFactor_, 0.0, 1.0));
#else
	currentPos_.y = desiredHeight;
#endif
}

void NavigationController::updateControllerOrientation() {
	const float desiredSpeed = (isWalking_ ? walkSpeed_ : runSpeed_);
	const Vec2f currentPos2D(currentPos_.x, currentPos_.z);

	if (navModeMask_ & APPROACHING) {
		// Compute curve tangent
		Vec2f bezierTan2 = bezierPath_.tangent(curveTime_);
		desiredDir_.x = bezierTan2.x;
		desiredDir_.y = 0.0f;
		desiredDir_.z = bezierTan2.y;
	}
	else if (navModeMask_ & FLOCKING) {
		// Orient in direction of group center.
		Vec2f groupDir = navGroup_->groupCenter2D() - currentPos2D;
		desiredDir_.x = groupDir.x;
		desiredDir_.y = 0.0f;
		desiredDir_.z = groupDir.y;
	} else {
		// maintain current direction
		return;
	}
	float tmpLen = desiredDir_.length();
	if (tmpLen > 1e-6f) desiredDir_ /= tmpLen;

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

	if (!(navModeMask_ & APPROACHING)) {
		velBlend = 0.0f;
		maxDeltaThisFrame *= 2.0f;
	}

#if 0
	// Reduce influence of velocity when close to target, and
	// when we are approaching the last waypoint of the current path.
	bool isAtLastWP = (currentPathIndex_ + 1 == currentPath_.size());
	if (isAtLastWP) {
		float distToGoal = (bezierPath_.p3 - currentPos2D).lengthSquared();
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
		blendedDir = desiredDir_ * (1.0f - velBlend) + velDir * velBlend;
	} else {
		blendedDir = desiredDir_;
	}
	// safety: fallback to tangent if blended is too small
	tmpLen = blendedDir.length();
	if (tmpLen < 1e-4f) {
		blendedDir = desiredDir_;
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

void NavigationController::animate(GLdouble dt) {
	if (navModeMask_ == NO_NAVIGATION) return;
	lastDT_ = dt / 1000.0;
	approachTime_ += static_cast<float>(lastDT_);
	if (navModeMask_ & APPROACHING) {
		// Approaching target along Bezier curve
		frameTime_ = approachTime_ / approachDuration_;
		curveTime_ = math::Bezier<Vec2f>::lookupParameter(bezierLUT_, frameTime_);
	}

	if (hasNewPerception_) {
		hasNewPerception_ = false;
	} else {
		// Blend-out avoidance with velocity over time when no new perception frame arrived.
		// This avoids that avoidance "sticks" too long.
		navAvoidance_ *= std::max(0.0f, 1.0f - avoidanceDecay_ * static_cast<float>(lastDT_));
	}

	updateControllerVelocity();
	updateControllerOrientation();

#if 0
	const float replanThresholdSq = 100.0f;
	float distanceToBezier = (currentPos2D - bezierSample).lengthSquared();
	if (distanceToBezier > replanThresholdSq) {
		// We are too far away from the bezier curve, so re-plan.
		if (currentPathIndex_ < currentPath_.size()) {
			REGEN_WARN("[" << tfIdx_ << "] Re-planning path, distance to bezier: " << sqrtf(distanceToBezier) << "m");
			Vec2f target = currentPath_.back()->position2D();
			updatePathCurve(currentPos2D, target);
			// and return, we will update velocity next frame
			return;
		}
	}
#endif

	if ((navModeMask_ & APPROACHING) && frameTime_ >= 1.0f) {
		// Approaching duration completed
		navModeMask_ &= ~APPROACHING;
	}

	Quaternion q(0.0, 0.0, 0.0, 1.0);
	q.setEuler(currentDir_.x, currentDir_.y, currentDir_.z);
	currentVal_ = q.calculateMatrix();
	currentVal_.scale(initialScale_);
	currentVal_.translate(currentPos_);
	tf_->setModelMat(tfIdx_, currentVal_);
}
