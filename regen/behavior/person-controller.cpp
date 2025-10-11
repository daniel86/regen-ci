#include <random>
#include "person-controller.h"

#include "regen/shapes/obb.h"

using namespace regen;

//#define DISABLE_BONE_ANIMATION

PersonController::PersonController(
	const ref_ptr<Mesh> &mesh,
	const Indexed<ref_ptr<ModelTransformation>> &tfIndexed,
	const ref_ptr<BoneAnimationItem> &animItem,
	const ref_ptr<WorldModel> &world)
	: NonPlayerCharacterController(mesh, tfIndexed, animItem, world),
      knowledgeBase_(tfIndexed.index, &currentPos_, &currentDir_) {
	boneController_ = ref_ptr<BoneController>::alloc(tfIndexed.index, animItem);
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
	npcWorldObject_ = ref_ptr<PersonWorldObject>::alloc(
		REGEN_STRING("npc-" << tfIndexed.index), Vec3f::zero());
	knowledgeBase_.setCharacterObject(npcWorldObject_);

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
	auto &homePlaces = knowledgeBase_.getPlacesByType(PlaceType::HOME);
	if (!homePlaces.empty()) {
		knowledgeBase_.setHomePlace(homePlaces[math::randomInt() % homePlaces.size()]);
	}
}

void PersonController::initializeController() {
	uint32_t initialIdleTime = math::random<float>() * 10.0f + 1.0f;
	knowledgeBase_.setLingerTime(initialIdleTime);
	knowledgeBase_.setActivityTime(initialIdleTime);
	// Start with idle animation
	setIdle();
	startAnimation();
}

void PersonController::setIdle() {
	auto &kb = knowledgeBase_;
	kb.setCurrentAction(ActionType::IDLE);
	kb.setActivityTime(kb.baseTimeActivity() * 0.05f * math::random<float>());
	kb.unsetInteractionTarget();
	currentPath_.clear();
}

void PersonController::setBehaviorTree(std::unique_ptr<BehaviorTree::Node> rootNode) {
	behaviorTree_ = std::make_unique<BehaviorTree>(std::move(rootNode));
}

void PersonController::setWeaponMesh(const ref_ptr<Mesh> &mesh) {
	weaponMesh_ = mesh;
	weaponShape_ = mesh->indexedShape(tfIdx_);
	if (!weaponShape_) {
		REGEN_WARN("Unable to find weapon shape in NPC controller.");
	}
	hasDrawnWeapon_ = true;
	hideWeapon();
}

void PersonController::hideWeapon() {
	if (hasDrawnWeapon_) {
		hasDrawnWeapon_ = false;
		if (weaponShape_.get()) {
			weaponMask_ = weaponShape_->traversalMask();
			weaponShape_->setTraversalMask(0);
		}
	}
}

void PersonController::drawWeapon() {
	if (!hasDrawnWeapon_) {
		hasDrawnWeapon_ = true;
		if (weaponShape_.get()) {
			weaponShape_->setTraversalMask(weaponMask_);
		}
	}
}

void PersonController::addWayPoint(const ref_ptr<WayPoint> &wp) {
	pathPlanner_->addWayPoint(wp);
}

void PersonController::addConnection(
	const ref_ptr<WayPoint> &from,
	const ref_ptr<WayPoint> &to,
	bool bidirectional) {
	pathPlanner_->addConnection(from, to, bidirectional);
}

void PersonController::updateController(double dt) {
	auto &kb = knowledgeBase_;
	auto anim = animItem_->animation;
	float dt_s = dt / 1000.0f;
	lastDT_ = dt_s;
	// Insert new percepts into the knowledge base, and update existing ones.
	updatePerceptionSystem(dt_s);
	// Do some controller specific updates to the knowledge base.
	updateKnowledgeBase(dt_s);

	if (footstepTrail_.get() && isLastAnimationMovement_) {
		int32_t walkHandle = boneController_->getAnimationHandle(MotionType::WALK);
		int32_t runHandle = boneController_->getAnimationHandle(MotionType::RUN);
		int32_t rangeIdx = std::max(walkHandle, runHandle);
		if (rangeIdx != -1) {
			float movementTime = (walkHandle != -1 ?
				boneController_->walkTime() : boneController_->runTime());
			auto elapsed = anim->elapsedTime(tfIdx_, rangeIdx);
			for (int i = 0; i < 2; i++) {
				if (!footDown_[i] && elapsed > footTime_[i] * movementTime) {
					footDown_[i] = true;
					footstepTrail_->insertBlanket(currentPos_, currentDir_, footIdx_[i]);
				}
			}
		}
	}

	// Update blackboard and make some actions current.
	auto behaviorStatus = behaviorTree_->tick(kb);
	if (behaviorStatus == BehaviorStatus::FAILURE) {
		REGEN_WARN("Behavior tree failed for NPC '" << npcWorldObject_->name() << "'.");
		setIdle();
	}
	// Update the state of current bone animations based on current actions, intends etc.
	boneController_->updateBoneController(knowledgeBase_, dt_s);
	// Update path etc. for navigation action.
	updateNavigationTarget();

	// Set flags for current bone animations
	if (boneController_->isCurrentBoneAnimation(MotionType::RUN)) {
		isWalking_ = false;
		isLastAnimationMovement_ = true;
	} else if (boneController_->isCurrentBoneAnimation(MotionType::WALK)) {
		isWalking_ = true;
		isLastAnimationMovement_ = true;
	} else {
		isLastAnimationMovement_ = false;
	}

	if (weaponMesh_.get()) {
		bool useWeapon = kb.isWeaponRequired();
		if (useWeapon && !hasDrawnWeapon_) {
			drawWeapon();
		} else if (!useWeapon && hasDrawnWeapon_) {
			hideWeapon();
		}
	}
}

void PersonController::updatePerceptionSystem(float dt_s) {
	// TODO: implement perception updates
}

void PersonController::updateKnowledgeBase(float dt_s) {
	auto &kb = knowledgeBase_;
	kb.advanceTime(dt_s);

	// Re-compute distance to patient and target
	if (kb.hasInteractionTarget()) {
		// Update distance to patient
		auto &target = kb.interactionTarget();
		Vec2f delta;
		if (!target.affordance || target.affordanceSlot < 0) {
			// No affordance, so just go to the center of the object
			delta = target.object->position2D() - Vec2f(currentPos_.x, currentPos_.z);
		} else {
			// Go to the affordance slot position
			auto &slotPos = target.affordance->slotPosition(target.affordanceSlot);
			delta = Vec2f(slotPos.x, slotPos.z) - Vec2f(currentPos_.x, currentPos_.z);
		}
		kb.setDistanceToPatient(delta.length());
		// Also let motion controller know the shape of the patient, such that it can
		// ignore collisions with it.
		if (patientShape_.get() != target.object->shape().get()) {
			patientShape_ = target.object->shape();
		}
	} else if (patientShape_.get()) {
		// No patient anymore
		patientShape_ = {};
		kb.setDistanceToPatient(std::numeric_limits<float>::max());
	}
	if (kb.hasNavigationTarget()) {
		auto &target = kb.navigationTarget().object;
		if (target.get() == kb.interactionTarget().object.get()) {
			kb.setDistanceToTarget(kb.distanceToPatient());
		} else {
			auto delta = target->position2D() - Vec2f(currentPos_.x, currentPos_.z);
			kb.setDistanceToTarget(delta.length());
		}
		// Also make the distance accessible to motion controller, as it does not have access to the blackboard.
		distanceToTarget_ = kb.distanceToTarget();
	}
}

void PersonController::updateNavigationTarget() {
	auto &kb = knowledgeBase_;
	auto &anim = animItem_->animation;

	ActionType currentNavAction = ActionType::IDLE;
	if (kb.isCurrentAction(ActionType::NAVIGATING)) {
		currentNavAction = ActionType::NAVIGATING;
	} else if (kb.isCurrentAction(ActionType::PATROLLING)) {
		currentNavAction = ActionType::PATROLLING;
	} else if (kb.isCurrentAction(ActionType::STROLLING)) {
		currentNavAction = ActionType::STROLLING;
	}

	if (currentNavAction == ActionType::IDLE) {
		// No navigation action active, nothing to do.
		it_ = frames_.end();
		currentPath_.clear();
		lastNavAction_ = ActionType::IDLE;
		lastNavTarget_ = nullptr;
		return;
	}
	if (currentNavAction != lastNavAction_ || lastNavTarget_ != kb.navigationTarget().object.get()) {
		// Navigation action changed, or a new navigation target has been set.
		// Reset path to avoid new action following the path of the old action.
		currentPath_.clear();
	}
	lastNavAction_ = currentNavAction;
	lastNavTarget_ = kb.navigationTarget().object.get();

	bool loop = (currentNavAction == ActionType::PATROLLING || currentNavAction == ActionType::STROLLING);
	if (it_ != frames_.end()) {
		// The TF animation has an active frame, continue navigation.
		int32_t walkHandle = boneController_->getAnimationHandle(MotionType::WALK);
		int32_t runHandle = boneController_->getAnimationHandle(MotionType::RUN);
		int32_t rangeIdx = std::max(walkHandle, runHandle);
		if (rangeIdx != -1 && !anim->isBoneAnimationActive(tfIdx_, rangeIdx)) {
			// one animation cycle is done, reset the footstep flags
			footDown_[0] = false;
			footDown_[1] = false;
		}
	}
	else if (loop && currentPath_.empty() && kb.currentPlace().get()) {
		// We are patrolling or strolling, but have no path yet, so start one.
		auto &currentPlace = kb.currentPlace();
		auto loopPath = currentPlace->getPathWays(
			currentNavAction == ActionType::PATROLLING ? PathwayType::PATROL : PathwayType::STROLL);
		// Pick a random path
		if (!loopPath.empty()) {
			currentPath_ = loopPath[math::randomInt() % loopPath.size()];
			// TODO: Select closest waypoint index to start with.
			currentPathIndex_ = 0;
			startNavigate(true, false);
		} else {
			REGEN_WARN("No pathways of type " << currentNavAction <<
				" at place " << currentPlace->name());
			setIdle();
		}
	}
	else {
		// The TF animation has no active frame, advance to next waypoint if any.
		// TODO: If we got off rails a lot, maybe it could be worth re-planning? Could
		//      check the distance here....
		if (!currentPath_.empty() && (loop || currentPathIndex_ < currentPath_.size())) {
			// Set next TF waypoint for movement
			startNavigate(loop, true);
		} else {
			// No waypoint remaining, but navigation action is still active,
			// so it seems we need to re-plan.
			currentPath_.clear();
			startNavigate(false, false);
		}
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

bool PersonController::startNavigate(bool loopPath, bool advancePath) {
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
		// Already at target place.
		// For now we don't do local navigation planning within a place,
		// but just go directly to the target.
		return updatePathNPC();
	}
	if (!targetPlace) {
		REGEN_WARN("No target place for navigation.");
		return false;
	}
	auto currentPos2D = Vec2f(currentPos_.x, currentPos_.z);
	// Compute a path to the target.
	currentPath_ = pathPlanner_->findPath(currentPos2D, targetPlace->position2D());
	// Add a waypoint within the target place which is close to the last waypoint.
	if (currentPath_.empty()) {
		if (currentPlace.get() != targetPlace.get()) {
			// We are not at the target place yet, so add the arrival waypoint.
			auto arrivalWP = pickArrivalWP(targetPlace, currentPos2D);
			if (arrivalWP.get()) currentPath_ = { arrivalWP };
		}
	} else {
		auto &lastWP = currentPath_.back();
		auto arrivalWP = pickArrivalWP(targetPlace, lastWP->position2D());
		if (arrivalWP.get()) currentPath_.push_back(arrivalWP);
	}
	currentPathIndex_ = 0;
	return updatePathNPC();
}

Vec2f PersonController::pickTravelPosition(const WorldObject &wp) const {
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

Vec2f PersonController::pickTargetPosition(const WorldObject &wp) {
	auto &patient = knowledgeBase_.interactionTarget();
	if (!patient.affordance) {
		return pickTravelPosition(wp);
	} else {
		auto &slotPos =
			patient.affordance->slotPosition(patient.affordanceSlot);
		return Vec2f(slotPos.x, slotPos.z);
	}
}

bool PersonController::updatePathNPC() {
	Vec2f source(currentPos_.x, currentPos_.z);

	if (currentPath_.size() < 2 || currentPathIndex_+1 >= currentPath_.size()) {
		if (!knowledgeBase_.hasInteractionTarget()) {
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
			auto &target = knowledgeBase_.interactionTarget().object;
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
