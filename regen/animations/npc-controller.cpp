#include <random>
#include "npc-controller.h"

using namespace regen;

#define SMOOTH_HEIGHT

NPCController::NPCController(
	const ref_ptr<WorldModel> &world,
	const ref_ptr<ModelTransformation> &tf,
	uint32_t tfIdx,
	const ref_ptr<NodeAnimation> &boneAnimation,
	const std::vector<scene::AnimRange> &ranges)
	: AnimationController(tf, tfIdx, boneAnimation, ranges),
	  worldModel_(world) {
	for (auto &range: animationRanges_) {
		if (range.name.find("walk") == 0) {
			motionRanges_[MOTION_WALK].push_back(&range);
		} else if (range.name.find("run") == 0) {
			motionRanges_[MOTION_RUN].push_back(&range);
		} else if (range.name.find("idle") == 0) {
			motionRanges_[MOTION_IDLE].push_back(&range);
		} else if (range.name.find("yes") == 0) {
			motionRanges_[MOTION_YES].push_back(&range);
		} else if (range.name.find("no") == 0) {
			motionRanges_[MOTION_NO].push_back(&range);
		} else if (range.name.find("crouch") == 0) {
			motionRanges_[MOTION_CROUCH].push_back(&range);
		} else if (range.name.find("pray") == 0) {
			motionRanges_[MOTION_PRAY].push_back(&range);
		} else if (range.name.find("attack") == 0) {
			motionRanges_[MOTION_ATTACK].push_back(&range);
		} else if (range.name.find("up") == 0) {
			motionRanges_[MOTION_STAND_UP].push_back(&range);
		} else if (range.name.find("sleep") == 0) {
			motionRanges_[MOTION_SLEEP].push_back(&range);
		}
	}

	if (!motionRanges_[MOTION_WALK].empty()) {
		auto walkRange = motionRanges_[MOTION_WALK][0]->range;
		auto walkTPS = boneAnimation->ticksPerSecond(motionRanges_[MOTION_WALK][0]->channelIndex);
		walkTime_ = 1000.0f * (walkRange.y - walkRange.x) / walkTPS;
	}
	if (!motionRanges_[MOTION_RUN].empty()) {
		auto runRange = motionRanges_[MOTION_RUN][0]->range;
		auto runTPS = boneAnimation->ticksPerSecond(motionRanges_[MOTION_RUN][0]->channelIndex);
		runTime_ = 1000.0f * (runRange.y - runRange.x) / runTPS;
	}

	pathPlanner_ = ref_ptr<PathPlanner>::alloc();
	// add all places of interest and way points from the world model
	for (const auto &p : worldModel_->places) {
		addPlaceOfInterest(p);
	}
	for (const auto &wp : worldModel_->wayPoints) {
		addWayPoint(wp);
	}
	for (const auto &conn : worldModel_->wayPointConnections) {
		addConnection(conn.first, conn.second);
	}
}


void NPCController::setSpatialIndex(const ref_ptr<SpatialIndex> &spatialIndex, std::string_view shapeName) {
	spatialIndex_ = spatialIndex;
	indexedShape_ = spatialIndex_->getShape(shapeName, tfIdx_);
	if (!indexedShape_.get()) {
		REGEN_WARN("Unable to find indexed shape '" << shapeName << "' in spatial index.");
	}
}

void NPCController::addPlaceOfInterest(const ref_ptr<Place> &place) {
	switch (place->placeType()) {
		case PLACE_HOME:
			homePlaces_.push_back(place);
			break;
		case PLACE_SPIRITUAL:
			spiritualPlaces_.push_back(place);
			break;
		case PLACE_GATHERING:
			gatheringPlaces_.push_back(place);
			break;
		case PLACE_PATROL_POINT:
			patrolPoints_.push_back(place);
			break;
		default:
			REGEN_WARN("Unhandled place type " << place->placeType() << ".");
			break;
	}
	pathPlanner_->addWayPoint(place);
}

void NPCController::addWayPoint(const ref_ptr<WayPoint> &wp) {
	pathPlanner_->addWayPoint(wp);
}

void NPCController::addConnection(
	const ref_ptr<WayPoint> &from,
	const ref_ptr<WayPoint> &to,
	bool bidirectional) {
	pathPlanner_->addConnection(from, to, bidirectional);
}

void NPCController::setHeightMap(const ref_ptr<HeightMap> &heightMap) {
	heightMap_ = heightMap;
	heightMap_->ensureTextureData();
}

float NPCController::getHeight(const Vec2f &pos) {
	if (heightMap_.get()) {
		return heightMap_->sampleHeight(pos) - 0.25f; // offset a bit to avoid floating
	} else {
		return floorHeight_;
	}
}

/**
static void intersectWithBounds(Vec2f& point, const Vec2f& origin, const Bounds<Vec2f>& territoryBounds) {
        if (point.x < territoryBounds.min.x) {
            float t = (territoryBounds.min.x - origin.x) / (point.x - origin.x);
            point.x = territoryBounds.min.x;
            point.y = origin.y + t * (point.y - origin.y);
        } else if (point.x > territoryBounds.max.x) {
            float t = (territoryBounds.max.x - origin.x) / (point.x - origin.x);
            point.x = territoryBounds.max.x;
            point.y = origin.y + t * (point.y - origin.y);
        }
        if (point.y < territoryBounds.min.y) {
            float t = (territoryBounds.min.y - origin.y) / (point.y - origin.y);
            point.y = territoryBounds.min.y;
            point.x = origin.x + t * (point.x - origin.x);
        } else if (point.y > territoryBounds.max.y) {
            float t = (territoryBounds.max.y - origin.y) / (point.y - origin.y);
            point.y = territoryBounds.max.y;
            point.x = origin.x + t * (point.x - origin.x);
        }
}
**/

struct NPCNeighborData {
	NPCController *npc;
	const BoundingShape *shape;
	uint32_t neighborCount = 0;
	Vec3f avoidance = Vec3f::zero();
	float personalSpace = 2.0f;
};

void NPCController::handleNeighbour(const BoundingShape &other, void *userData) {
	NPCNeighborData *data = static_cast<NPCNeighborData *>(userData);
	if (data->shape == &other) return; // skip self

	Vec3f offset;
	float dist;
	const Vec3f &thisCenter = data->shape->tfOrigin();

	if (data->shape->baseMesh().get() == other.baseMesh().get()) {
		const Vec3f &otherCenter = other.tfOrigin();
		offset = thisCenter - otherCenter;
		dist = std::max(0.0f, offset.length() - data->personalSpace);
	} else {
		// TODO: rather enforce 3D test in spatial index query, this is effectively what happens then!
		Vec3f otherClosest = other.closestPointOnSurface(thisCenter);

		offset = thisCenter - otherClosest;
		dist = offset.length();
	}
	if (dist < 0.01f) return;

	if (dist > data->personalSpace) {
		// FIXME: spatial index gives a lot of false positives, not sure why.
		//REGEN_WARN("NOT INTERSECTING!");
		return;
	}

	offset /= dist; // normalize
	data->avoidance += offset / (dist * dist); // inverse-square falloff
	data->neighborCount++;
}

Vec3f NPCController::computeNeighborAvoidance() {
	// TODO: personal space param
	const float personalSpace = 1.0 + 1.0 * (1.0f - bravery_);

	NPCNeighborData data;
	data.npc = this;
	data.shape = indexedShape_.get();
	data.personalSpace = personalSpace;

	if (spatialIndex_.get() && indexedShape_.get()) {
		BoundingSphere querySphere(currentPos_, personalSpace);
		querySphere.updateTransform(true);
		spatialIndex_->foreachIntersection(
			querySphere,
			handleNeighbour,
			&data,
			collisionMask_);
	}

	if (data.neighborCount > 0) {
		data.avoidance /= static_cast<float>(data.neighborCount);
	}

	return data.avoidance;
}

static inline float wrapPi(float a) {
	// normalize to (-PI, PI]
	while (a <= -M_PI) a += 2.0f * M_PI;
	while (a > M_PI) a -= 2.0f * M_PI;
	return a;
}

Vec2f NPCController::pickTargetPosition(const ref_ptr<WayPoint> &wp) {
	// Base target position (waypoint center)
	auto targetPos3D = wp->pos()->getVertex(0);
	Vec2f targetPos(targetPos3D.r.x, targetPos3D.r.z);

	// Minimum distance from affordance (default 0)
	const float minimumDistance = (currentBehaviour_.affordance.get() ?
		currentBehaviour_.affordance->minDistance : 0.0f);

	// Randomize within the waypoint radius
	float radius = std::max(wp->radius(), minimumDistance + 0.05f);
	if (radius > 0.1f) {
		// Pick an angle roughly facing *towards* the waypoint from the NPC's current position
		Vec2f npcPos(currentPos_.x, currentPos_.z);
		Vec2f toWP = targetPos - npcPos;
		float baseAngle = atan2(toWP.y, toWP.x);

		// Allow a small angular variation (±45°)
		float angleVariation = (math::random<float>() - 0.5f) * (M_PI / 2.0f);
		float angle = baseAngle + angleVariation;

		// Radius sampled between minimumDistance and radius
		float r = minimumDistance + (radius - minimumDistance) * math::random<float>();

		// Offset final target
		targetPos += Vec2f(cos(angle), sin(angle)) * r;
	}

	return targetPos;
}

void NPCController::updatePose(const TransformKeyFrame &currentFrame, double frameTime) {
	const float desiredSpeed = (currentBehaviour_.motion == MOTION_RUN ? runSpeed_ : walkSpeed_);
	float bezierTime = math::Bezier<Vec2f>::lookupParameter(bezierLUT_, frameTime);

	if (currentFrame.pos.has_value()) {
		auto sample = bezierPath_.sample(bezierTime);
		// Desired velocity toward Bezier point
		Vec3f vel(sample.x - currentPos_.x, 0.0f, sample.y - currentPos_.z);
		float length = vel.length();
		if (length < 0.01f) { return; }
		vel /= length;

		Vec3f avoidance = computeNeighborAvoidance();
		if (avoidance.lengthSquared() > 0.001f) {
			// Normalize to consistent strength
			avoidance.normalize();
			// Blend with desired direction instead of overwriting
			vel = (vel * (1.0f - avoidanceBlend_) + avoidance * avoidanceBlend_);
			vel.normalize();
			vel *= desiredSpeed;
			//if (vel.dot(desiredDir) < 0) {
			//	// push velocity into forward hemisphere, avoid backwards movement
			//	vel = desiredDir * desiredSpeed * 0.2f; // small forward nudge
			//}
		} else {
			vel *= desiredSpeed;
		}
		// Update position by following the final velocity
		Vec3f velDir = currentVel_;
		float velDirLen = velDir.length();
		if (velDirLen > 1e-4f) {
			velDir /= velDirLen;
		}
		float angleDiff = acos(std::clamp(vel.dot(velDir), -1.0f, 1.0f));
		float blendFactor = std::clamp(0.1f + (angleDiff / M_PIf) * 0.4f, 0.1f, 0.5f);
		// avoid rapid changes in velocity by blending with current vel
		currentVel_ = math::lerp(currentVel_, vel, blendFactor);

		currentPos_ += currentVel_ * lastDT_;
		currentPos_.y = getHeight(Vec2f(currentPos_.x, currentPos_.z));
	}

	{
		// compute path tangent (as 3D: x,z -> x,z)
		Vec2f bezierTan2 = bezierPath_.tangent(bezierTime);
		if (bezierTan2.length() > 1e-6f) bezierTan2.normalize();
		Vec3f tangentDir(bezierTan2.x, 0.0f, bezierTan2.y); // tangent.x -> world X, tangent.y -> world Z
		if (tangentDir.length() > 1e-6f) tangentDir.normalize();

		// compute velocity direction (if moving)
		float velLen = currentVel_.length();
		Vec3f velDir(0.0f, 0.0f, 0.0f);
		if (velLen > 1e-4f) {
			velDir = Vec3f(currentVel_.x, 0.0f, currentVel_.z);
			velDir.normalize();
		}

		// Blend velocity direction and path tangent to form a stable target direction.
		// Idea: when moving fast, prefer velocity; when slow, prefer path tangent.
		float vWeight = std::clamp(velLen / (desiredSpeed + 1e-6f), 0.0f, 1.0f); // 0..1
		const float velOrientationInfluence = 0.9f; // how strongly to trust velocity when vWeight==1
		float velBlend = vWeight * velOrientationInfluence; // in [0,1]

		// compute blended direction
		Vec3f blendedDir;
		if (velDir.lengthSquared() > 1e-5f) {
			blendedDir = tangentDir * (1.0f - velBlend) + velDir * velBlend;
		} else {
			blendedDir = tangentDir;
		}
		// safety: fallback to tangent if blended is too small
		if (blendedDir.lengthSquared() < 1e-5f) {
			blendedDir = tangentDir;
		}
		blendedDir.normalize();

		// compute target yaw (world yaw)
		float targetYaw = atan2(blendedDir.z, blendedDir.x);
		// retrieve current stored yaw (undo baseOrientation_ offset that you store in currentDir_.x)
		float prevYaw = currentDir_.x + baseOrientation_;
		// shortest angular difference
		float deltaYaw = wrapPi(targetYaw - prevYaw);
		// clamp rotation speed (radians per second)
		float maxDeltaThisFrame = maxTurn_ * lastDT_;
		if (deltaYaw > maxDeltaThisFrame) deltaYaw = maxDeltaThisFrame;
		else if (deltaYaw < -maxDeltaThisFrame) deltaYaw = -maxDeltaThisFrame;
		// apply the clamped rotation
		currentDir_.x = prevYaw + deltaYaw - baseOrientation_;
	}
}

void NPCController::updatePathCurve(const Vec2f &source, const Vec2f &target) {
	Vec2f dir;
	if (!currentPath_.empty() && currentPathIndex_+1 == currentPath_.size()) {
		auto pathTarget3D = currentPath_.back()->pos()->getVertex(0);
		dir = Vec2f(pathTarget3D.r.x - target.x, pathTarget3D.r.z - target.y);
	} else {
		dir = target - source;
	}
	if (dir.lengthSquared() < 1e-6f) {
		dir = Vec2f(cos(currentDir_.x + baseOrientation_), sin(currentDir_.x + baseOrientation_));
	} else {
		dir.normalize();
	}
	float angle = atan2(dir.y, dir.x) - baseOrientation_;
	updatePathCurve(source, target, Vec3f(angle, 0.0f, 0.0f));
}

void NPCController::updatePathCurve(
	const Vec2f &source,
	const Vec2f &target,
	const Vec3f &orientation) {
	bezierPath_.p0 = source;
	bezierPath_.p3 = target;
	// compute control points using Euler angles
	float directDistance = (bezierPath_.p0 - bezierPath_.p3).length();
	// add base orientation
	float angle_rad1 = currentDir_.x + baseOrientation_;
	float angle_rad2 = orientation.x + baseOrientation_;
	float angleDiff = wrapPi(angle_rad2 - angle_rad1);
	// shorter handles for sharper turns
	float handleScale = directDistance * 0.5f * (1.0f - 0.5f * std::abs(angleDiff) / M_PI);
	//float handleScale = directDistance * 0.25f * (1.0f - 0.5f * std::abs(angleDiff) / M_PI);
	// p1: in direction of start orientation
	bezierPath_.p1 = bezierPath_.p0 + Vec2f(cos(angle_rad1), sin(angle_rad1)) * handleScale;
	// p2: in direction of target orientation
	bezierPath_.p2 = bezierPath_.p3 - Vec2f(cos(angle_rad2), sin(angle_rad2)) * handleScale;

	float bezierLength = bezierPath_.length1();
	bezierLUT_ = bezierPath_.buildArcLengthLUT(100);

	// compute dt based on distance and speed
	float dt = bezierLength / (currentBehaviour_.motion == MOTION_RUN ? runSpeed_ : walkSpeed_);
	// set the target position
	setTarget(
		Vec3f(target.x, getHeight(target), target.y),
		orientation,
		dt);
}

void NPCController::startNavigate(bool loopPath, bool advancePath) {
	if (!currentPath_.empty()) {
		// Reached target point
		if (advancePath) {
			currentPathIndex_++;
		}
		if (loopPath && currentPathIndex_ >= currentPath_.size()) {
			currentPathIndex_ = 0;
		}
		if (currentPathIndex_ < currentPath_.size()) {
			// Move to next point on path
			auto &nextPoint = currentPath_[currentPathIndex_];
			updatePathCurve(
				Vec2f(currentPos_.x, currentPos_.z),
				pickTargetPosition(nextPoint));
			return;
		} else {
			currentPath_.clear();
		}
	}
	if (!currentBehaviour_.target) {
		REGEN_WARN("No target place for navigation.");
		return;
	}
	float distSqr = (currentBehaviour_.target->pos()->getVertex(0).r - currentPos_).lengthSquared();
	if (distSqr < 0.01f) {
		REGEN_WARN("Already at target place.");
	}

	currentPath_ = pathPlanner_->findPath(
		currentPos_, currentBehaviour_.target);
	currentPathIndex_ = 0;

	if (currentPath_.size() < 2) {
		updatePathCurve(
			Vec2f(currentPos_.x, currentPos_.z),
			pickTargetPosition(currentBehaviour_.target));
	} else {
		auto &firstWP = currentPath_.front();
		updatePathCurve(
			Vec2f(currentPos_.x, currentPos_.z),
			pickTargetPosition(firstWP));
	}
}

void NPCController::startMotion() {
	const Motion movementType = currentBehaviour_.motion;
	auto &ranges = motionRanges_[movementType];
	if (ranges.empty()) {
		REGEN_WARN("No animation ranges for movement type " << movementType << ".");
		return;
	}
	auto &range = ranges[rand() % ranges.size()];
	lastRange_ = range;
	animation_->setAnimationActive(tfIdx_, range->channelName, range->range);
	animation_->startAnimation();
}

void NPCController::updateController(double dt) {
	float dt_s = dt / 1000.0f;
	lastDT_ = dt_s;
	currentBehaviour_.lingerTime -= dt_s;
	currentBehaviour_.activityTime -= dt_s;

	if (footstepTrail_.get() && isLastAnimationMovement_) {
		auto movementTime = (currentBehaviour_.motion == MOTION_WALK ? walkTime_ : runTime_);
		auto elapsed = animation_->elapsedTime(tfIdx_);
		if (!leftFootDown_ && elapsed > leftFootTime_ * movementTime) {
			leftFootDown_ = true;
			footstepTrail_->insertBlanket(currentPos_, currentDir_, leftFootIdx_);
		}
		if (!rightFootDown_ && elapsed > rightFootTime_ * movementTime) {
			rightFootDown_ = true;
			footstepTrail_->insertBlanket(currentPos_, currentDir_, rightFootIdx_);
		}
	}

	if (it_ == frames_.end()) {
		// currently no movement.
		if (animation_->isNodeAnimationActive(tfIdx_)) {
			if (isLastAnimationMovement_) {
				// animation is movement, deactivate
				animation_->stopNodeAnimation(tfIdx_);
			}
			return;
		}
	} else {
		// movement active
		if (!animation_->isNodeAnimationActive(tfIdx_)) {
			// animation not active, activate
			if (lastRange_) {
				animation_->setAnimationActive(tfIdx_, lastRange_->channelName, lastRange_->range);
				animation_->startAnimation();
				if (isLastAnimationMovement_) {
					leftFootDown_ = false;
					rightFootDown_ = false;
				}
			}
		}
		return;
	}

	leftFootDown_ = false;
	rightFootDown_ = false;
	auto lastMovement = currentBehaviour_.motion;
	updateBehavior(dt);
	if (lastMovement == MOTION_SLEEP) {
		if (lastMovement == currentBehaviour_.motion) {
			// remain sleeping
			return;
		}
	}
	switch (currentBehaviour_.motion) {
		case MOTION_RUN:
		case MOTION_WALK:
			isLastAnimationMovement_ = true;
			//startNavigate(false, true);
			break;
		default:
			isLastAnimationMovement_ = false;
			break;
	}
	startMotion();
}

void NPCController::setIdle() {
	currentBehaviour_.state = STATE_IDLE;
	currentBehaviour_.motion = MOTION_IDLE;
	currentBehaviour_.activity = ACTIVITY_IDLE;
	currentBehaviour_.nextActivity = -1;
	currentBehaviour_.lingerTime = 0;
	currentBehaviour_.activityTime = 0;
	currentBehaviour_.target = {};
	currentBehaviour_.source = {};
	currentBehaviour_.patient = {};
}

uint32_t NPCController::findClosestWP_idx(const std::vector<ref_ptr<WayPoint>> &wps) {
	if (wps.empty()) return 0;
	float bestDist = std::numeric_limits<float>::max();
	ref_ptr<WayPoint> bestWP;
	uint32_t bestIdx = 0;
	for (uint32_t i = 0; i < wps.size(); i++) {
		auto &wp = wps[i];
		auto wpPos3D = wp->pos()->getVertex(0);
		Vec2f wpPos(wpPos3D.r.x, wpPos3D.r.z);
		float dist = (wpPos - Vec2f(currentPos_.x, currentPos_.z)).lengthSquared();
		if (dist < bestDist) {
			bestDist = dist;
			bestWP = wp;
			bestIdx = i;
		}
	}
	return bestIdx;
}

void NPCController::setPlaceActivity(const ref_ptr<Place> &place) {
	Behaviour &b = currentBehaviour_;

	std::vector<Activity> possibleActivities;
	if (place->hasPathWay(PathWayType::PATROL)) {
		possibleActivities.push_back(ACTIVITY_PATROLLING);
	}
	if (place->hasPathWay(PathWayType::STROLL)) {
		possibleActivities.push_back(ACTIVITY_STROLLING);
	}
	if (place->hasAffordance(AffordanceType::OBSERVE)) {
		possibleActivities.push_back(ACTIVITY_OBSERVING);
	}
	if (place->hasAffordance(AffordanceType::CONVERSE)) {
		possibleActivities.push_back(ACTIVITY_CONVERSING);
	}
	if (place->hasAffordance(AffordanceType::PRAY)) {
		possibleActivities.push_back(ACTIVITY_PRAYING);
	}
	if (place->hasAffordance(AffordanceType::SLEEP)) {
		possibleActivities.push_back(ACTIVITY_SLEEPING);
	}
	if (possibleActivities.empty()) {
		possibleActivities.push_back(ACTIVITY_IDLE);
	}
	// Pick a random activity from the possible ones
	// TODO: weight by personality traits etc.
	auto nextActivity = possibleActivities[math::randomInt() % possibleActivities.size()];

	switch (nextActivity) {
		case ACTIVITY_IDLE:
			b.activity = ACTIVITY_IDLE;
			b.nextActivity = -1;
			b.motion = MOTION_IDLE;
			b.activityTime = 0.0;
			b.patient = {};
			b.affordance = {};
			break;
		case ACTIVITY_OBSERVING: {
			// Walk to an observable object and observe it there
			auto &observables = place->getAffordanceObjects(AffordanceType::OBSERVE);
			auto observable = observables[math::randomInt() % observables.size()];
			b.motion = MOTION_WALK;
			b.activityTime = 30.0 + (math::random<float>() * 90.0f);
			b.activity = ACTIVITY_TRAVELING;
			b.nextActivity = ACTIVITY_OBSERVING;
			b.patient = observable;
			b.affordance = observable->getAffordance(AffordanceType::OBSERVE);
			break;
		}
		case ACTIVITY_PRAYING: {
			// Walk to a prayable object and pray there
			auto &prayableObjects = place->getAffordanceObjects(AffordanceType::PRAY);
			auto prayable = prayableObjects[math::randomInt() % prayableObjects.size()];
			b.motion = MOTION_WALK;
			b.activityTime = 30.0 + (math::random<float>() * 60.0f);
			b.activity = ACTIVITY_TRAVELING;
			b.nextActivity = ACTIVITY_PRAYING;
			b.patient = prayable;
			b.affordance = prayable->getAffordance(AffordanceType::PRAY);
			break;
		}
		case ACTIVITY_SLEEPING: {
			// Walk to a sleepable object and sleep there
			auto &sleepableObjects = place->getAffordanceObjects(AffordanceType::SLEEP);
			auto sleepable = sleepableObjects[math::randomInt() % sleepableObjects.size()];
			b.motion = MOTION_WALK;
			b.activityTime = 30.0 + (math::random<float>() * 60.0f);
			b.activity = ACTIVITY_TRAVELING;
			b.nextActivity = ACTIVITY_SLEEPING;
			b.patient = sleepable;
			b.affordance = sleepable->getAffordance(AffordanceType::SLEEP);
			break;
		}
		case ACTIVITY_PATROLLING: {
			auto &patrolPaths = place->getPathWays(PathWayType::PATROL);
			currentPath_ = patrolPaths[math::randomInt() % patrolPaths.size()];
			// randomly reverse the path
			if (math::random<float>() < 0.5f) {
				std::reverse(currentPath_.begin(), currentPath_.end());
			}
			currentPathIndex_ = findClosestWP_idx(currentPath_);
			b.motion = MOTION_WALK;
			b.activityTime = 60.0 + (math::random<float>() * 60.0f);
			b.activity = ACTIVITY_PATROLLING;
			b.nextActivity = -1;
			b.patient = {};
			b.affordance = {};
			startNavigate(true, false);
			break;
		}
		case ACTIVITY_STROLLING: {
			auto &strollPaths = place->getPathWays(PathWayType::STROLL);
			currentPath_ = strollPaths[math::randomInt() % strollPaths.size()];
			// randomly reverse the path
			if (math::random<float>() < 0.5f) {
				std::reverse(currentPath_.begin(), currentPath_.end());
			}
			currentPathIndex_ = findClosestWP_idx(currentPath_);
			b.motion = MOTION_WALK;
			b.activityTime = 30.0 + (math::random<float>() * 60.0f);
			b.activity = ACTIVITY_STROLLING;
			b.nextActivity = -1;
			b.patient = {};
			b.affordance = {};
			startNavigate(true, false);
			break;
		}
		default:
			b.activity = ACTIVITY_IDLE;
			b.nextActivity = -1;
			b.motion = MOTION_IDLE;
			b.activityTime = 0.0f;
			b.patient = {};
			b.affordance = {};
			break;
	}
}

void NPCController::updatePlaceActivity(const ref_ptr<Place> &place) {
	Behaviour &b = currentBehaviour_;
	const Vec3f &pos = currentPos_;

	auto dist2D = [](const Vec3f &a, const Vec3f &b) -> float {
		return (Vec2f(a.x, a.z) - Vec2f(b.x, b.z)).length();
	};

	switch (b.activity) {
		case ACTIVITY_OBSERVING:
		case ACTIVITY_IDLE:
			b.motion = MOTION_IDLE;
			break;
		case ACTIVITY_SITTING:
			// TODO: sitting motion
			b.motion = MOTION_IDLE;
			//b.motion = MOTION_SIT;
			break;
		case ACTIVITY_PRAYING:
			// TODO: support special pray motion
			b.motion = MOTION_CROUCH;
			break;
		case ACTIVITY_SLEEPING:
			b.motion = MOTION_SLEEP;
			break;
		case ACTIVITY_PATROLLING:
		case ACTIVITY_STROLLING:
			startNavigate(true, true);
			break;
		case ACTIVITY_TRAVELING:
			if (b.nextActivity != -1) {
				const ref_ptr<WayPoint> &wp = currentPath_[currentPath_.size() - 1];
				auto targetPos = wp->pos()->getVertex(0);
				float distance = dist2D(pos, targetPos.r);
				if (distance < wp->radius()) {
					b.activity = static_cast<Activity>(b.nextActivity);
					b.nextActivity = -1;
				} else {
					startNavigate(false, true);
				}
			} else {
				startNavigate(false, true);
			}
			break;
		case ACTIVITY_FLEEING: {
			const ref_ptr<WayPoint> &wp = currentPath_[currentPath_.size() - 1];
			auto targetPos = wp->pos()->getVertex(0);
			float distance = dist2D(pos, targetPos.r);
			if (distance < wp->radius()) {
				setIdle();
			} else {
				startNavigate(false, true);
			}
			break;
		}
		case ACTIVITY_CONVERSING: {
			std::vector<Motion> possibleMotions;
			if (motionRanges_.find(MOTION_IDLE) != motionRanges_.end()) {
				possibleMotions.push_back(MOTION_IDLE);
			}
			if (motionRanges_.find(MOTION_YES) != motionRanges_.end()) {
				possibleMotions.push_back(MOTION_YES);
			}
			if (motionRanges_.find(MOTION_NO) != motionRanges_.end()) {
				possibleMotions.push_back(MOTION_NO);
			}
			if (!possibleMotions.empty()) {
				b.motion = possibleMotions[math::randomInt() % possibleMotions.size()];
			}
			break;
		}
		case ACTIVITY_FIGHTING: {
			std::vector<Motion> possibleMotions;
			if (motionRanges_.find(MOTION_ATTACK) != motionRanges_.end()) {
				possibleMotions.push_back(MOTION_ATTACK);
			}
			if (motionRanges_.find(MOTION_BLOCK) != motionRanges_.end()) {
				possibleMotions.push_back(MOTION_BLOCK);
			}
			if (!possibleMotions.empty()) {
				b.motion = possibleMotions[math::randomInt() % possibleMotions.size()];
			}
			break;
		}
	}
}

void NPCController::setAtHome() {
	currentBehaviour_.state = STATE_AT_HOME;
	currentBehaviour_.lingerTime = homeLingerTime_.x + (math::random<float>() * homeLingerTime_.y);
	setPlaceActivity(currentBehaviour_.target);
}

void NPCController::setAtMarket() {
	currentBehaviour_.state = STATE_AT_MARKET;
	currentBehaviour_.lingerTime = socialLingerTime_.x + (math::random<float>() * socialLingerTime_.y);
	setPlaceActivity(currentBehaviour_.target);
}

void NPCController::setAtShrine() {
	currentBehaviour_.state = STATE_AT_SHRINE;
	currentBehaviour_.lingerTime = prayLingerTime_.x + (math::random<float>() * prayLingerTime_.y);
	setPlaceActivity(currentBehaviour_.target);
}

void NPCController::updateBehavior(double dt) {
	Behaviour &b = currentBehaviour_;

	// Helper lambdas
	auto pickRandomPlace = [](const std::vector<ref_ptr<Place> > &places) -> ref_ptr<Place> {
		if (places.empty()) return {};
		return places[math::randomInt() % places.size()];
	};
	auto dist2D = [](const Vec3f &a, const Vec3f &b) -> float {
		return (Vec2f(a.x, a.z) - Vec2f(b.x, b.z)).length();
	};
	// Current world position
	const Vec3f &pos = currentPos_;

	switch (b.state) {
		case STATE_IDLE: {
			// Decide what to do next based on traits and available places
			float r = math::random<float>();

			if (!homePlaces_.empty() && r < (0.2f + laziness_ * 0.3f)) {
				// More likely to go home if lazy
				b.state = STATE_GO_TO_HOME;
				b.target = pickRandomPlace(homePlaces_);
			} else if (!gatheringPlaces_.empty() && r < 0.5f) {
				b.state = STATE_GO_TO_MARKET;
				b.target = pickRandomPlace(gatheringPlaces_);
			} else if (!spiritualPlaces_.empty() && r < (0.7f + spirituality_ * 0.3f)) {
				b.state = STATE_GO_TO_SHRINE;
				b.target = pickRandomPlace(spiritualPlaces_);
			} else {
				// No goal → just idle
				setIdle();
			}
			if (b.state != STATE_IDLE) {
				// Set the movement type
				b.motion = MOTION_WALK;
				b.activity = ACTIVITY_TRAVELING;
				b.nextActivity = -1;
				b.lingerTime = 0.0;
				b.activityTime = 0.0;
				b.patient = {};
				b.affordance = {};
				updatePlaceActivity(b.target);
			}
			break;
		}
		case STATE_GO_TO_HOME:
		case STATE_GO_TO_MARKET:
		case STATE_GO_TO_SHRINE: {
			if (!b.target) {
				setIdle();
				break;
			}
			// Check if we've arrived
			auto targetPos = b.target->pos()->getVertex(0);
			float distance = dist2D(pos, targetPos.r);
			if (distance < b.target->radius()) {
				// Switch to "being there" state
				switch (b.state) {
					case STATE_GO_TO_HOME: setAtHome();
						break;
					case STATE_GO_TO_MARKET: setAtMarket();
						break;
					case STATE_GO_TO_SHRINE: setAtShrine();
						break;
					default: setIdle();
						break;
				}
			} else {
				updatePlaceActivity(b.target);
			}
			break;
		}
		case STATE_AT_SHRINE:
		case STATE_AT_MARKET:
		case STATE_AT_HOME: {
			if (b.lingerTime < 0.0) {
				setIdle(); // switch to idle state
			} else if (b.activityTime < 0.0) {
				setPlaceActivity(b.target);
			} else {
				updatePlaceActivity(b.target);
			}
			break;
		}
		case STATE_ON_ALERT: {
			// TODO: Implement on alert behavior
			// Look around briefly, then choose fight/flee/idle
			if ((math::randomInt() % 100) < 10) {
				if (bravery_ > 0.5f) {
					b.state = STATE_FIGHTING;
					b.motion = MOTION_ATTACK;
				} else {
					b.state = STATE_RUN_AWAY;
					b.motion = MOTION_RUN;
				}
			}
			break;
		}
		case STATE_RUN_AWAY: {
			// TODO: Implement run away behavior
			// After some time, calm down and go idle
			if ((math::randomInt() % 100) < 5) {
				setIdle();
			}
			break;
		}
		case STATE_FIGHTING: {
			// TODO: Implement fighting behavior
			if ((math::randomInt() % 100) < 5) {
				setIdle();
			}
			break;
		}
		default:
			break;
	}
}
