#include <random>
#include "npc-controller.h"

#include "regen/shapes/obb.h"

using namespace regen;

//#define DISABLE_BONE_ANIMATION

NPCController::NPCController(
	const ref_ptr<Mesh> &mesh,
	const Indexed<ref_ptr<ModelTransformation>> &tfIndexed,
	const ref_ptr<NodeAnimationItem> &animItem,
	const ref_ptr<WorldModel> &world)
	: AnimationController(mesh, tfIndexed, animItem, world) {
	initializeBoneAnimations();
	// create the initial collision shape
	Bounds<Vec3f> collisionBounds(mesh->minPosition(), mesh->maxPosition());
	collisionBounds.min.z -= 15.0f;
	collisionBounds.min.x -= 0.0f;
	collisionBounds.max.x += 0.0f;
	collisionBounds.min.y -= 1.0f;
	collisionBounds.max.y += 1.0f;
	collisionShape_ = ref_ptr<OBB>::alloc(collisionBounds);
	collisionShape_->setTransform(tfIndexed.value, tfIndexed.index);
	collisionShape_->updateTransform(true);
	// and a world object that can use affordances
	npcWorldObject_ = ref_ptr<NPCWorldObject>::alloc(
		REGEN_STRING("npc-" << tfIndexed.index), Vec3f::zero());

	pathPlanner_ = ref_ptr<PathPlanner>::alloc();
	// add all places of interest and way points from the world model
	for (const auto &p : worldModel_->places) {
		knowledgeBase_.addPlaceOfInterest(p);
	}
	for (const auto &wp : worldModel_->wayPoints) {
		addWayPoint(wp);
	}
	for (const auto &conn : worldModel_->wayPointConnections) {
		addConnection(conn.first, conn.second);
	}
	// pick a home
	auto &homePlaces = knowledgeBase_.homePlaces();
	if (!homePlaces.empty()) {
		knowledgeBase_.setHomePlace(homePlaces[math::randomInt() % homePlaces.size()]);
	}
}

void NPCController::initializeBoneAnimations() {
	for (auto &range: animItem_->ranges) {
		if (range.name.find("walk") == 0) {
			motionRanges_[MotionType::WALK].push_back(&range);
		} else if (range.name.find("run") == 0) {
			motionRanges_[MotionType::RUN].push_back(&range);
		} else if (range.name.find("idle") == 0) {
			motionRanges_[MotionType::IDLE].push_back(&range);
		} else if (range.name.find("yes") == 0) {
			motionRanges_[MotionType::YES].push_back(&range);
		} else if (range.name.find("no") == 0) {
			motionRanges_[MotionType::NO].push_back(&range);
		} else if (range.name.find("crouch") == 0) {
			motionRanges_[MotionType::CROUCH].push_back(&range);
		} else if (range.name.find("pray") == 0) {
			motionRanges_[MotionType::PRAY].push_back(&range);
		} else if (range.name.find("attack") == 0) {
			motionRanges_[MotionType::ATTACK].push_back(&range);
		} else if (range.name.find("up") == 0) {
			motionRanges_[MotionType::STAND_UP].push_back(&range);
		} else if (range.name.find("sleep") == 0) {
			motionRanges_[MotionType::SLEEP].push_back(&range);
		}
	}
	if (!motionRanges_[MotionType::WALK].empty()) {
		auto walkRange = motionRanges_[MotionType::WALK][0]->range;
		auto walkTPS = animItem_->animation->ticksPerSecond(motionRanges_[MotionType::WALK][0]->channelIndex);
		walkTime_ = 1000.0f * (walkRange.y - walkRange.x) / walkTPS;
	}
	if (!motionRanges_[MotionType::RUN].empty()) {
		auto runRange = motionRanges_[MotionType::RUN][0]->range;
		auto runTPS = animItem_->animation->ticksPerSecond(motionRanges_[MotionType::RUN][0]->channelIndex);
		runTime_ = 1000.0f * (runRange.y - runRange.x) / runTPS;
	}
}

void NPCController::setWeaponMesh(const ref_ptr<Mesh> &mesh) {
	weaponMesh_ = mesh;
	weaponShape_ = mesh->indexedShape(tfIdx_);
	if (!weaponShape_) {
		REGEN_WARN("Unable to find weapon shape in NPC controller.");
	}
	hasDrawnWeapon_ = true;
	hideWeapon();
}

void NPCController::hideWeapon() {
	if (hasDrawnWeapon_) {
		hasDrawnWeapon_ = false;
		if (weaponShape_.get()) {
			weaponMask_ = weaponShape_->traversalMask();
			weaponShape_->setTraversalMask(0);
		}
	}
}

void NPCController::drawWeapon() {
	if (!hasDrawnWeapon_) {
		hasDrawnWeapon_ = true;
		if (weaponShape_.get()) {
			weaponShape_->setTraversalMask(weaponMask_);
		}
	}
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

void NPCController::initializeController() {
	startAnimation();
	// Start with idle animation
	setIdle();
	uint32_t initialIdleTime = math::random<float>() * 10.0f + 1.0f;
	knowledgeBase_.setLingerTime(initialIdleTime);
	knowledgeBase_.setActivityTime(initialIdleTime);
	startMotion();
}

Vec2f NPCController::pickTravelPosition(const WorldObject &wp) const {
	// Base target position (waypoint center)
	Vec2f baseTarget = wp.position2D();
	if (wp.radius() < 0.1f) {
		return baseTarget;
	} else {
		// Pick an angle roughly facing *towards* the waypoint from the NPC's current position
		Vec2f npcPos(currentPos_.x, currentPos_.z);
		Vec2f toWP = baseTarget - npcPos;
		float angle = atan2(toWP.y, toWP.x);
		// Allow a small angular variation (+-45°)
		angle += (math::random<float>() - 0.5f) * (M_PI / 2.0f);
		// Radius sampled between minimumDistance and radius
		float r = wp.radius() * math::random<float>();
		// Offset final target
		return baseTarget + Vec2f(cos(angle), sin(angle)) * r;
	}
}

Vec2f NPCController::pickTargetPosition(const WorldObject &wp) {
	auto &patient = knowledgeBase_.patient();
	if (!patient.affordance) {
		return pickTravelPosition(wp);
	} else {
		auto &slotPos =
			patient.affordance->slotPosition(patient.affordanceSlot);
		return Vec2f(slotPos.x, slotPos.z);
	}
}

static ref_ptr<WayPoint> pickArrivalWP(
	const ref_ptr<Place> &target, const Vec2f &fromPos) {
	const float desiredDistanceToCenter = target->radius() * 0.75f;
	Vec2f pos = target->position2D();
	// Direction from place center to source position
	Vec2f dir = fromPos - pos;
	dir.y = 0.0f;
	float l = dir.length();
	if (l < desiredDistanceToCenter) {
		// The source position is already within the desired distance to the center.
		return {};
	} else {
		dir /= l;
	}
	// move from center towards source position
	pos += dir * desiredDistanceToCenter;

	auto arrivalWP = ref_ptr<WayPoint>::alloc(target->name(), target->position3D());
	arrivalWP->setRadius(target->radius() * 0.5f);
	return arrivalWP;
}

bool NPCController::updatePathNPC() {
	Vec2f source(currentPos_.x, currentPos_.z);

	if (currentPath_.size() < 2 || currentPathIndex_+1 >= currentPath_.size()) {
		if (!knowledgeBase_.hasPatient()) {
			auto &target = knowledgeBase_.targetPlace();
			// Travel to place, prefer to use pre-computed waypoint but fallback to
			//  going to the center of the place.
			if (currentPath_.empty()) {
				updatePathCurve(source, pickTravelPosition(*target.get()));
			} else {
				auto &lastWP = currentPath_.back();
				updatePathCurve(source, pickTravelPosition(*lastWP.get()) );
			}
		} else {
			// Travel to affordance
			auto &target = knowledgeBase_.patient().object;
			Vec2f affordancePos = pickTargetPosition(*target.get());

			// NPC shall look from affordance slot to center of attention
			// (i.e. affordance target, or owner of affordance)
			Vec2f dir = (target->position2D() - affordancePos);
			float dist = dir.length();
			if (dist < affordanceSlotRadius_) {
				dir = Vec2f(cos(currentDir_.x + baseOrientation_), sin(currentDir_.x + baseOrientation_));
			} else {
				dir /= dist;
			}

			float o = atan2(dir.y, dir.x) - baseOrientation_;
			updatePathCurve(source, affordancePos, Vec3f(o, 0.0f, 0.0f) );
		}
	} else {
		auto &nextWP = currentPath_[currentPathIndex_];
		updatePathCurve(source, pickTravelPosition(*nextWP.get()));
	}
	return true;
}

bool NPCController::startNavigate(bool loopPath, bool advancePath) {
	auto &kb = knowledgeBase_;
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
			return updatePathNPC();
		} else if (advancePath) {
			currentPathIndex_ = 0;
			currentPath_.clear();
			return false;
		}
	}
	auto &currentPlace = kb.currentPlace();
	auto &targetPlace = kb.targetPlace();
	if (currentPlace.get() && currentPlace.get() == targetPlace.get()) {
		// Already at target place, no need for path planning, NPC can just move to target position.
		return updatePathNPC();
	}
	if (!targetPlace) {
		REGEN_WARN("No target place for navigation.");
		return false;
	}
	auto currentPos2D = Vec2f(currentPos_.x, currentPos_.z);
	// Find path to closest waypoint of target place.
	currentPath_ = pathPlanner_->findPath(currentPos2D, targetPlace->position2D());
	// Also pick a waypoint within the target place which is close to the last waypoint.
	if (currentPath_.empty()) {
		if (currentPlace.get() != targetPlace.get()) {
			// We are not at the target place yet, so add the arrival waypoint.
			auto arrivalWP = pickArrivalWP(targetPlace, currentPos2D);
			if (arrivalWP.get()) {
				currentPath_ = { arrivalWP };
			}
		}
	} else {
		auto &lastWP = currentPath_.back();
		auto arrivalWP = pickArrivalWP(targetPlace, lastWP->position2D());
		if (arrivalWP.get()) {
			currentPath_.push_back(arrivalWP);
		}
	}
	currentPathIndex_ = 0;
	return updatePathNPC();
}

void NPCController::startMotion() {
	auto &ranges = motionRanges_[currentMotion_];
	if (ranges.empty()) {
		REGEN_WARN("No animation ranges for movement type " << (int)currentMotion_ << ".");
		return;
	}
	auto &range = ranges[rand() % ranges.size()];
	lastRange_ = range;
#ifndef DISABLE_BONE_ANIMATION
	animItem_->animation->setAnimationActive(tfIdx_, range->channelName, range->range);
	animItem_->animation->startAnimation();
#endif
}

void NPCController::updateController(double dt) {
	auto &kb = knowledgeBase_;
	auto anim = animItem_->animation;
	float dt_s = dt / 1000.0f;
	lastDT_ = dt_s;
	kb.advanceTime(dt_s);

	if (footstepTrail_.get() && isLastAnimationMovement_) {
		auto movementTime = (currentMotion_ == MotionType::WALK ? walkTime_ : runTime_);
		auto elapsed = anim->elapsedTime(tfIdx_);
		for (int i = 0; i < 2; i++) {
			if (!footDown_[i] && elapsed > footTime_[i] * movementTime) {
				footDown_[i] = true;
				footstepTrail_->insertBlanket(currentPos_, currentDir_, footIdx_[i]);
			}
		}
	}

	if (it_ == frames_.end()) {
		// The TF animation has no active frame.
		if (anim->isNodeAnimationActive(tfIdx_) && isLastAnimationMovement_) {
			auto &currentPlace = kb.currentPlace();
			// Plus a bone animation is active at the moment.
			// Check if it was a movement animation, if so we need to set the next waypoint or stop
			// the bone animation.
			bool isNavActivity = (currentActivity_ == ActionType::PATROLLING ||
								  currentActivity_ == ActionType::STROLLING);
			bool loop = isNavActivity;
			bool forceStop = (currentPlace.get() &&
				(isNavActivity && kb.activityTime() <= 0.0f));
			if (!forceStop && !currentPath_.empty() && (loop || currentPathIndex_ < currentPath_.size())) {
				// Set next TF waypoint for movement
				REGEN_DEBUG("[" << tfIdx_ << "] Reached waypoint " <<
					currentPath_[currentPathIndex_]->name() <<
					" for navigation to " <<
					(kb.targetPlace().get() ? kb.targetPlace()->name() : "none"));
				startNavigate(loop, true);
				if (currentPathIndex_ < currentPath_.size()) {
					REGEN_DEBUG("[" << tfIdx_ << "] Next waypoint is " <<
						currentPath_[currentPathIndex_]->name());
				}
			} else {
				// No waypoint remaining
				REGEN_DEBUG("[" << tfIdx_ << "] reached end of path at place " <<
					(currentPlace.get() ? currentPlace->name() : "none") << " and patient " <<
					(kb.patient().object.get() ? kb.patient().object->name() : "none") <<
					", stopping animation.");
				anim->stopNodeAnimation(tfIdx_);
				currentPath_.clear();
			}
		}
	} else {
		// The TF animation has an active frame.
		if (!anim->isNodeAnimationActive(tfIdx_)) {
			// Plus no bone animation is active at the moment, so we need to start one
			// for the navigation action.
#ifndef DISABLE_BONE_ANIMATION
			if (lastRange_) {
				anim->setAnimationActive(tfIdx_, lastRange_->channelName, lastRange_->range);
				anim->startAnimation();
				footDown_[0] = false;
				footDown_[1] = false;
			}
#endif
		}
	}
	if (anim->isNodeAnimationActive(tfIdx_)) {
		return;
	}
	// No TF animation frame and no bone animation active, so we can update the behaviour
	// and start a new motion if needed.

	footDown_[0] = false;
	footDown_[1] = false;
	auto lastMovement = currentMotion_;
	updateBehavior(dt);
	if (lastMovement == MotionType::SLEEP) {
		if (lastMovement == currentMotion_) {
			// remain sleeping
			return;
		}
	}
	// Set flags for current motion
	switch (currentMotion_) {
		case MotionType::RUN:
			isWalking_ = false;
			isLastAnimationMovement_ = true;
			break;
		case MotionType::WALK:
			isWalking_ = true;
			isLastAnimationMovement_ = true;
			break;
		default:
			isLastAnimationMovement_ = false;
			break;
	}
	// Start the bone animation of the motion
	startMotion();

	if (weaponMesh_.get()) {
		bool requiresWeapon = (currentActivity_ == ActionType::ATTACKING ||
							   currentActivity_ == ActionType::BLOCKING);
		if (requiresWeapon && !hasDrawnWeapon_) {
			drawWeapon();
		} else if (!requiresWeapon && hasDrawnWeapon_) {
			hideWeapon();
		}
	}
}

static float traitStrength(
	const std::vector<float> &positive,
	const std::vector<float> &negative) {
	float factor = 0.5f; // neutral
	for (auto t : positive) {
		factor *= t;
	}
	for (auto t : negative) {
		factor *= (1.0f - t);
	}
	return factor;
}

void NPCController::setIdle() {
	auto &kb = knowledgeBase_;
	currentState_ = STATE_IDLE;
	currentMotion_ = MotionType::IDLE;
	currentActivity_ = ActionType::IDLE;
	nextActivity_ = -1;
	kb.setActivityTime(kb.baseTimeActivity() * (
		0.05f * math::random<float>() +
		0.2f * traitStrength(
			{kb.laziness()},
			{kb.alertness(), kb.sociability()})));
	kb.unsetPatient();
	currentPath_.clear();
}

uint32_t NPCController::findClosestWP_idx(const std::vector<ref_ptr<WayPoint>> &wps) const {
	if (wps.empty()) return 0;
	float bestDist = std::numeric_limits<float>::max();
	uint32_t bestIdx = 0;
	for (uint32_t i = 0; i < wps.size(); i++) {
		auto &wp = wps[i];
		float dist = (wp->position2D() - Vec2f(currentPos_.x, currentPos_.z)).lengthSquared();
		if (dist < bestDist) {
			bestDist = dist;
			bestIdx = i;
		}
	}
	return bestIdx;
}

bool NPCController::setPatient(const ref_ptr<WorldObject> &obj, AffordanceType type) {
	auto &kb = knowledgeBase_;
	if (!obj) return false;
	auto aff = obj->getAffordance(type);
	if (!aff) return false;
	if (aff.get() == kb.patient().affordance.get()) {
		// already set
		return true;
	}
	unsetPatient();

	if (!aff->hasFreeSlot()) return false;

	bool randomizeSlot = (aff->layout != SlotLayout::GRID);
	int slotIdx = aff->reserveSlot(npcWorldObject_.get(), randomizeSlot);
	if (slotIdx < 0) {
		REGEN_WARN("[" << tfIdx_ << "] Failed to reserve affordance slot "
			   << type << " on " << obj->name());
		return false;
	}

	kb.setPatient(obj, aff, slotIdx);
	// TODO: Set patient shape such that motion control can ignore the patient for collision.
	//patientShape_ = obj->collisionShape();
	patientPos_ = obj->position2D();
	return true;
}

void NPCController::unsetPatient() {
	knowledgeBase_.unsetPatient();
	patientPos_.x = std::numeric_limits<float>::max();
	patientPos_.y = std::numeric_limits<float>::max();
}

void NPCController::updateActionPossibilities(const ref_ptr<Place> &place) {
	auto &kb = knowledgeBase_;
	// TODO: Also check that we have a matching motion
	actionPossibilities_.clear();

	actionPossibilities_.emplace_back(ActionType::IDLE, (
			0.05f +
			0.05f * math::random<float>() +
			0.1f * traitStrength({kb.laziness()}, {kb.alertness(), kb.sociability()})));

	if (place->hasPathWay(PathWayType::PATROL)) {
		actionPossibilities_.emplace_back(ActionType::PATROLLING, (
				0.1f +
				0.1f * math::random<float>() +
				0.4f * traitStrength({kb.alertness(), kb.bravery()}, {kb.laziness(), kb.sociability()})));
	}
	if (place->hasPathWay(PathWayType::STROLL)) {
		actionPossibilities_.emplace_back(ActionType::STROLLING, (
				0.15f +
				0.15f * math::random<float>() +
				0.4f * traitStrength({kb.laziness()}, {kb.sociability(), kb.alertness(), kb.bravery()})));
	}
	if (place->hasAffordance(AffordanceType::OBSERVE)) {
		actionPossibilities_.emplace_back(ActionType::OBSERVING, (
				0.25f +
				0.25f * math::random<float>() +
				0.5f * traitStrength({kb.alertness()}, {kb.sociability()})));
	}
	if (place->hasAffordance(AffordanceType::CONVERSE)) {
		actionPossibilities_.emplace_back(ActionType::CONVERSING, (
				0.25f +
				0.25f * math::random<float>() +
				0.5f * traitStrength({kb.sociability()}, {kb.laziness()})));
	}
	if (place->hasAffordance(AffordanceType::PRAY)) {
		actionPossibilities_.emplace_back(ActionType::PRAYING, (
				0.25f +
				0.25f * math::random<float>() +
				0.5f * traitStrength({kb.spirituality()}, {kb.laziness()})));
	}
	if (place->hasAffordance(AffordanceType::SLEEP)) {
		actionPossibilities_.emplace_back(ActionType::SLEEPING, (
				0.1f +
				0.1f * math::random<float>() +
				0.5f * traitStrength({kb.laziness()}, {kb.alertness(), kb.sociability()})));
	}
	if (place->hasAffordance(AffordanceType::ATTACK)) {
		actionPossibilities_.emplace_back(ActionType::ATTACKING, (
				0.15f +
				0.15f * math::random<float>() +
				0.4f * traitStrength({kb.bravery()}, {kb.laziness(), kb.sociability()})));
	}
	// Sum and normalize weights.
	float totalWeight = 0.0f;
	for (auto &p : actionPossibilities_) {
		totalWeight += p.second;
	}
	for (auto &p : actionPossibilities_) {
		p.second /= totalWeight;
	}
}

void NPCController::setActivityTime(ActionType activity) {
	auto &kb = knowledgeBase_;
	switch (activity) {
		case ActionType::OBSERVING:
			kb.setActivityTime(kb.baseTimeActivity() * (0.5f +
				0.25f * math::random<float>() +
				0.5f * traitStrength({kb.laziness()}, {kb.alertness(), kb.sociability()})));
			break;
		case ActionType::CONVERSING:
			kb.setActivityTime(kb.baseTimeActivity() * (0.5f +
				0.25f * math::random<float>() +
				0.5f * traitStrength({kb.sociability()}, {kb.laziness()})));
			break;
		case ActionType::PRAYING:
			kb.setActivityTime(kb.baseTimeActivity() * (1.0f +
				0.5f * math::random<float>() +
				1.0f * traitStrength({kb.spirituality()}, {kb.laziness()})));
			break;
		case ActionType::SLEEPING:
			kb.setActivityTime(kb.baseTimeActivity() * (1.5f +
				0.5f * math::random<float>() +
				1.0f * traitStrength({kb.laziness()}, {kb.alertness(), kb.sociability()})));
			break;
		case ActionType::PATROLLING:
			kb.setActivityTime(kb.baseTimeActivity() * (1.5f +
				0.5f * math::random<float>() +
				0.5f * traitStrength({kb.alertness(), kb.bravery()}, {kb.laziness(), kb.sociability()})));
			break;
		case ActionType::STROLLING:
			kb.setActivityTime(kb.baseTimeActivity() * (1.0f +
				0.5f * math::random<float>() +
				0.5f * traitStrength({}, {kb.laziness(), kb.sociability()})));
			break;
		case ActionType::ATTACKING:
			kb.setActivityTime(kb.baseTimeActivity() * (0.5f +
				0.25f * math::random<float>() +
				0.5f * traitStrength({kb.bravery()}, {kb.laziness(), kb.sociability()})));
			break;
		case ActionType::TRAVELING:
			kb.setActivityTime(kb.baseTimeActivity() * (4.0f +
				2.0f * math::random<float>() +
				2.0f * traitStrength({}, {kb.laziness()})));
			break;
		default: // IDLE
			kb.setActivityTime(kb.baseTimeActivity() * (
				0.1f * math::random<float>() +
				0.5f * traitStrength({kb.laziness()}, {kb.alertness(), kb.sociability()})));
			break;
	}
}

void NPCController::setPlaceActivity(const ref_ptr<Place> &place) {
	// Update weights for action possibilities at this place.
	updateActionPossibilities(place);

	// Get a random number [0,1] and pick an activity based on the weights.
	ActionType nextActivity = ActionType::IDLE;
	float r = math::random<float>();
	float scan = 0.0f;
	for (uint32_t i = 0; i < actionPossibilities_.size(); i++) {
		scan += actionPossibilities_[i].second;
		if (r <= scan) {
			nextActivity = actionPossibilities_[i].first;
			break;
		}
	}
	REGEN_DEBUG("[" << tfIdx_ << "] Starting activity: " << static_cast<int>(nextActivity));

	switch (nextActivity) {
		case ActionType::OBSERVING: {
			// Walk to an observable object and observe it there
			auto &observables = place->getAffordanceObjects(AffordanceType::OBSERVE);
			auto observable = observables[math::randomInt() % observables.size()];
			currentMotion_ = MotionType::WALK;
			currentActivity_ = ActionType::TRAVELING;
			nextActivity_ = static_cast<int>(ActionType::OBSERVING);
			if (!setPatient(observable, AffordanceType::OBSERVE)) {
				setIdle();
			}
			break;
		}
		case ActionType::CONVERSING: {
			// Walk to a conversable object and converse there
			auto &conversables = place->getAffordanceObjects(AffordanceType::CONVERSE);
			auto conversable = conversables[math::randomInt() % conversables.size()];
			currentMotion_ = MotionType::WALK;
			currentActivity_ = ActionType::TRAVELING;
			nextActivity_ = static_cast<int>(ActionType::CONVERSING);
			if (!setPatient(conversable, AffordanceType::CONVERSE)) {
				setIdle();
			}
			break;
		}
		case ActionType::PRAYING: {
			// Walk to a prayable object and pray there
			auto &prayableObjects = place->getAffordanceObjects(AffordanceType::PRAY);
			auto prayable = prayableObjects[math::randomInt() % prayableObjects.size()];
			currentMotion_ = MotionType::WALK;
			currentActivity_ = ActionType::TRAVELING;
			nextActivity_ = static_cast<int>(ActionType::PRAYING);
			if (!setPatient(prayable, AffordanceType::PRAY)) {
				setIdle();
			}
			break;
		}
		case ActionType::ATTACKING: {
			// Walk to an attackable object and attack it there
			auto &attackableObjects = place->getAffordanceObjects(AffordanceType::ATTACK);
			auto attackable = attackableObjects[math::randomInt() % attackableObjects.size()];
			currentMotion_ = MotionType::WALK;
			currentActivity_ = ActionType::TRAVELING;
			nextActivity_ = static_cast<int>(ActionType::ATTACKING);
			if (!setPatient(attackable, AffordanceType::ATTACK)) {
				setIdle();
			}
			break;
		}
		case ActionType::SLEEPING: {
			// Walk to a sleepable object and sleep there
			auto &sleepableObjects = place->getAffordanceObjects(AffordanceType::SLEEP);
			auto sleepable = sleepableObjects[math::randomInt() % sleepableObjects.size()];
			currentMotion_ = MotionType::WALK;
			currentActivity_ = ActionType::TRAVELING;
			nextActivity_ = static_cast<int>(ActionType::SLEEPING);
			if (!setPatient(sleepable, AffordanceType::SLEEP)) {
				setIdle();
			}
			break;
		}
		case ActionType::PATROLLING: {
			auto &patrolPaths = place->getPathWays(PathWayType::PATROL);
			currentPath_ = patrolPaths[math::randomInt() % patrolPaths.size()];
			// randomly reverse the path
			if (math::random<float>() < 0.5f) {
				std::reverse(currentPath_.begin(), currentPath_.end());
			}
			currentPathIndex_ = findClosestWP_idx(currentPath_);
			currentMotion_ = MotionType::WALK;
			currentActivity_ = ActionType::PATROLLING;
			nextActivity_ = -1;
			unsetPatient();
			startNavigate(true, false);
			break;
		}
		case ActionType::STROLLING: {
			auto &strollPaths = place->getPathWays(PathWayType::STROLL);
			currentPath_ = strollPaths[math::randomInt() % strollPaths.size()];
			// randomly reverse the path
			if (math::random<float>() < 0.5f) {
				std::reverse(currentPath_.begin(), currentPath_.end());
			}
			currentPathIndex_ = findClosestWP_idx(currentPath_);
			currentMotion_ = MotionType::WALK;
			currentActivity_ = ActionType::STROLLING;
			nextActivity_ = -1;
			unsetPatient();
			startNavigate(true, false);
			break;
		}
		default: // IDLE
			currentMotion_ = MotionType::IDLE;
			currentActivity_ = ActionType::IDLE;
			nextActivity_ = -1;
			unsetPatient();
			break;
	}
	setActivityTime(nextActivity_ == -1 ? currentActivity_ : static_cast<ActionType>(nextActivity_));
}

static float dist2D(const Vec3f &a, const Vec3f &b) {
	return (Vec2f(a.x, a.z) - Vec2f(b.x, b.z)).length();
};

static float dist2D(const Vec3f &a, const Vec2f &b) {
	return (Vec2f(a.x, a.z) - b).length();
};

bool NPCController::isDoneWithPlace() const {
	return isDoneWithActivity() && (knowledgeBase_.lingerTime() <= 0.0f);
}

bool NPCController::isDoneWithActivity() const {
	return (knowledgeBase_.activityTime() <= 0.0f) && (nextActivity_ == -1);
}

void NPCController::updatePlaceActivity(const ref_ptr<Place> &place) {
	const Vec3f &pos = currentPos_;

	switch (currentActivity_) {
		case ActionType::OBSERVING:
		case ActionType::IDLE:
			currentMotion_ = MotionType::IDLE;
			break;
		case ActionType::SITTING:
			// TODO: sitting motion
			currentMotion_ = MotionType::IDLE;
			//currentMotion_ = MOTION_SIT;
			break;
		case ActionType::PRAYING:
			currentMotion_ = MotionType::CROUCH;
			break;
		case ActionType::ATTACKING:
			currentMotion_ = MotionType::ATTACK;
			break;
		case ActionType::BLOCKING:
			currentMotion_ = MotionType::BLOCK;
			break;
		case ActionType::SLEEPING:
			currentMotion_ = MotionType::SLEEP;
			break;
		case ActionType::PATROLLING:
		case ActionType::STROLLING:
			startNavigate(true, true);
			break;
		case ActionType::TRAVELING: {
			if (nextActivity_ != -1) {
				auto &patient = knowledgeBase_.patient();
				float wpRadius = 0.0f, distance;
				if (patient.affordance.get()) {
					const Vec3f &targetPos = patient.affordance->slotPosition(patient.affordanceSlot);
					wpRadius = affordanceSlotRadius_;
					distance = dist2D(pos, targetPos);
				} else if (!currentPath_.empty()) {
					const ref_ptr<WayPoint> &wp = currentPath_[currentPath_.size() - 1];
					wpRadius = patient.object->radius();
					distance = dist2D(pos, wp->position2D());
				} else {
					REGEN_WARN("Unable to find path to target place.");
					setIdle();
					return;
				}
				if (distance < wpRadius) {
					currentActivity_ = static_cast<ActionType>(nextActivity_);
					nextActivity_ = -1;
					currentMotion_ = MotionType::IDLE;
					currentPath_.clear();
					setActivityTime(currentActivity_);
					updatePlaceActivity(place);
					REGEN_DEBUG("[" << tfIdx_ << "] Arrived at affordance of " <<
						patient.object->name() << " at place " << place->name() << ".");
				} else {
					REGEN_DEBUG("[" << tfIdx_ << "] not arrived yet, distance = " << distance <<
						", wpRadius = " << wpRadius << ".");
					setActivityTime(static_cast<ActionType>(nextActivity_));
					currentPath_.clear();
					startNavigate(false, true);
				}
			} else {
				startNavigate(false, true);
			}
			break;
		}
		case ActionType::FLEEING: {
			const ref_ptr<WayPoint> &wp = currentPath_[currentPath_.size() - 1];
			float distance = dist2D(pos, wp->position2D());
			if (distance < wp->radius()) {
				setIdle();
			} else {
				startNavigate(false, true);
			}
			break;
		}
		case ActionType::CONVERSING: {
			std::vector<MotionType> possibleMotions;
			if (motionRanges_.find(MotionType::IDLE) != motionRanges_.end()) {
				possibleMotions.push_back(MotionType::IDLE);
			}
			if (motionRanges_.find(MotionType::YES) != motionRanges_.end()) {
				possibleMotions.push_back(MotionType::YES);
			}
			if (motionRanges_.find(MotionType::NO) != motionRanges_.end()) {
				possibleMotions.push_back(MotionType::NO);
			}
			if (!possibleMotions.empty()) {
				currentMotion_ = possibleMotions[math::randomInt() % possibleMotions.size()];
			}
			break;
		}
	}
}

void NPCController::setAtHome() {
	auto &kb = knowledgeBase_;
	currentState_ = STATE_AT_HOME;
	kb.setLingerTime(kb.baseTimePlace() * (0.75f +
		0.25f * math::random<float>() +
		0.5f * traitStrength({kb.laziness()}, {kb.sociability(), kb.spirituality(), kb.bravery()})));
	setPlaceActivity(kb.targetPlace());
}

void NPCController::setAtMarket() {
	auto &kb = knowledgeBase_;
	currentState_ = STATE_AT_MARKET;
	kb.setLingerTime(kb.baseTimePlace() * (0.75f +
		0.25f * math::random<float>() +
		0.5f * traitStrength({kb.sociability()}, {kb.laziness()})));
	setPlaceActivity(kb.targetPlace());
}

void NPCController::setAtShrine() {
	auto &kb = knowledgeBase_;
	currentState_ = STATE_AT_SHRINE;
	kb.setLingerTime(kb.baseTimePlace() * (1.5f +
		0.5f * math::random<float>() +
		1.0f * traitStrength({kb.spirituality(), kb.sociability()}, {kb.laziness()})));
	setPlaceActivity(kb.targetPlace());
}

void NPCController::updateBehavior(double dt) {
	auto &kb = knowledgeBase_;
	// Helper lambdas
	auto pickRandomPlace = [](const std::vector<ref_ptr<Place> > &places) -> ref_ptr<Place> {
		if (places.empty()) return {};
		return places[math::randomInt() % places.size()];
	};
	// Current world position
	const Vec3f &pos = currentPos_;

	switch (currentState_) {
		case STATE_IDLE: {
			// Decide what to do next based on traits and available places
			if (kb.activityTime() > 0.0) break; // Continue idling

			// Compute randomized weights for the places.
			float homeWeight = (0.5f + 0.25f * math::random<float>() +
				0.25f * traitStrength({kb.laziness()}, {kb.sociability(), kb.spirituality()}));
			float marketWeight = (0.5f + 0.25f * math::random<float>() +
				0.25f * traitStrength({kb.sociability()}, {kb.laziness()}));
			float shrineWeight = (0.5f + 0.25f * math::random<float>() +
				0.25f * traitStrength({kb.spirituality()}, {kb.laziness()}));
			if (!kb.homePlace()) homeWeight = 0.0f;
			if (kb.gatheringPlaces().empty()) marketWeight = 0.0f;
			if (kb.spiritualPlaces().empty()) shrineWeight = 0.0f;

			float r = math::random<float>() * (homeWeight + marketWeight + shrineWeight);
			marketWeight += homeWeight;
			shrineWeight += marketWeight;

			if (r < homeWeight) {
				// More likely to go home if lazy
				currentState_ = STATE_GO_TO_HOME;
				kb.setTargetPlace(kb.homePlace());
			} else if (r < marketWeight) {
				currentState_ = STATE_GO_TO_MARKET;
				kb.setTargetPlace(pickRandomPlace(kb.gatheringPlaces()));
			} else if (r < shrineWeight) {
				currentState_ = STATE_GO_TO_SHRINE;
				kb.setTargetPlace(pickRandomPlace(kb.spiritualPlaces()));
			} else {
				// No goal, just idle
				setIdle();
			}

			if (currentState_ != STATE_IDLE) {
				// Set the movement type
				currentMotion_ = MotionType::WALK;
				currentActivity_ = ActionType::TRAVELING;
				nextActivity_ = -1;
				kb.setLingerTime(0.0);
				kb.setActivityTime(0.0);
				// NPC is leaving current place
				kb.unsetCurrentPlace();
				unsetPatient();
				updatePlaceActivity(kb.targetPlace());
				setActivityTime(ActionType::TRAVELING);
				REGEN_DEBUG("[" << tfIdx_ << "] Traveling to place " <<
					(kb.targetPlace().get() ? kb.targetPlace()->name() : "none") << ".");
			}
			break;
		}
		case STATE_GO_TO_HOME:
		case STATE_GO_TO_MARKET:
		case STATE_GO_TO_SHRINE: {
			auto &targetPlace = kb.targetPlace();
			if (!targetPlace) {
				setIdle();
				break;
			}
			// Check if we've arrived
			float distance = dist2D(pos, targetPlace->position2D());
			if (distance < targetPlace->radius()) {
				// Set current place of the NPC
				kb.setCurrentPlace(targetPlace);
				// Switch to "being there" state
				switch (currentState_) {
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
				updatePlaceActivity(targetPlace);
			}
			break;
		}
		case STATE_AT_SHRINE:
		case STATE_AT_MARKET:
		case STATE_AT_HOME: {
			if (isDoneWithPlace()) {
				// Switch to idle state. From there we will pick a new place to go to.
				setIdle();
			} else if (isDoneWithActivity()) {
				setPlaceActivity(kb.targetPlace());
			} else {
				updatePlaceActivity(kb.targetPlace());
			}
			break;
		}
		case STATE_ON_ALERT: {
			// TODO: Implement on alert behavior
			// Look around briefly, then choose fight/flee/idle
			if ((math::randomInt() % 100) < 10) {
				if (kb.bravery() > 0.5f) {
					currentState_ = STATE_FIGHTING;
					currentMotion_ = MotionType::ATTACK;
				} else {
					currentState_ = STATE_RUN_AWAY;
					currentMotion_ = MotionType::RUN;
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
