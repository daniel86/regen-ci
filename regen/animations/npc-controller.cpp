#include <random>
#include "npc-controller.h"

using namespace regen;

#define SMOOTH_HEIGHT

NPCController::NPCController(
			const ref_ptr<ModelTransformation> &tf,
			uint32_t tfIdx,
			const ref_ptr<NodeAnimation> &animation,
			const std::vector<scene::AnimRange> &ranges)
				: AnimationController(tf, tfIdx, animation, ranges),
				  maxHeight_(std::numeric_limits<float>::max()),
				  minHeight_(std::numeric_limits<float>::lowest()),
				  heightMapBounds_(Vec2f::zero(), Vec2f::zero())
{
	for (auto &range: animationRanges_) {
		if (range.name.find("walk") == 0) {
			motionRanges_[MOTION_WALK].push_back(&range);
		}
		else if (range.name.find("run") == 0) {
			motionRanges_[MOTION_RUN].push_back(&range);
		}
		else if (range.name.find("idle") == 0) {
			motionRanges_[MOTION_IDLE].push_back(&range);
		}
		else if (range.name.find("attack") == 0) {
			motionRanges_[MOTION_ATTACK].push_back(&range);
		}
		else if (range.name.find("up") == 0) {
			motionRanges_[MOTION_STAND_UP].push_back(&range);
		}
		else if (range.name.find("sleep") == 0) {
			motionRanges_[MOTION_SLEEP].push_back(&range);
		}
	}
	REGEN_WARN("NPCController created for instance " << tfIdx_ <<
			" with " << animationRanges_.size() << " animation ranges" <<
			" initial-pos: " << currentPos_ <<
			" initial-rot: " << currentDir_);

}


void NPCController::addPlaceOfInterest(const ref_ptr<Place> &place) {
	switch (place->type) {
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
			REGEN_WARN("Unhandled place type " << place->type << ".");
			break;
	}
}

void NPCController::setHeightMap(
				const ref_ptr<Texture2D> &heightMap,
				const Vec2f &heightMapCenter,
				const Vec2f &heightMapSize,
				float heightMapFactor) {
	heightMap_ = heightMap;
	heightMap_->ensureTextureData();
	heightMapBounds_.min = heightMapCenter - heightMapSize;
	heightMapBounds_.max = heightMapCenter + heightMapSize;
	heightMapFactor_ = heightMapFactor;
}

float NPCController::getHeight(const Vec2f &pos) {
	float height = floorHeight_;
	if (heightMap_.get()) {
		auto mapCenter = (heightMapBounds_.max + heightMapBounds_.min) * 0.5f;
		auto mapSize = heightMapBounds_.max - heightMapBounds_.min;
		// compute UV for height map sampling
		auto uv = pos - mapCenter;
		uv /= mapSize * 0.5f;
		uv += Vec2f(0.5f);
#ifdef SMOOTH_HEIGHT
		auto texelSize = Vec2f(1.0f / heightMap_->width(), 1.0f / heightMap_->height());
		auto regionTS = texelSize*8.0f;
		auto mapValue = heightMap_->sampleAverage<float>(uv, regionTS, heightMap_->textureData());
#else
		auto mapValue = heightMap_->sampleLinear(uv, heightMap_->textureData(), 1);
#endif
		mapValue *= heightMapFactor_;
		// increase by small bias to avoid intersection with the floor
		mapValue += 0.02f;
		height += mapValue;
	}
	return height;
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

void NPCController::updatePose(const TransformKeyFrame &currentFrame, double t) {
	if (currentFrame.pos.has_value()) {
		//currentPos_ = math::mix(lastFrame_.pos.value(), currentFrame.pos.value(), t);
		auto sample = bezierPath_.sample(t);
		currentPos_.x = sample.x;
		currentPos_.z = sample.y;
		currentPos_.y = getHeight(sample);
	}
	if (currentFrame.rotation.has_value()) {
		auto tangent = bezierPath_.tangent(t);
		tangent.normalize();
		// Convert the tangent vector to Euler angles
        currentDir_.x = atan2(tangent.y, tangent.x) - baseOrientation_;
	}
}

void NPCController::startMotion() {
	REGEN_INFO("NPC " << tfIdx_ << " start motion type " << currentBehaviour_.motion);
	const Motion movementType = currentBehaviour_.motion;
	auto &ranges = motionRanges_[movementType];
	if (ranges.empty()) {
		REGEN_WARN("No animation ranges for movement type " << movementType << ".");
		return;
	}
	auto &range = ranges[rand() % ranges.size()];
	lastRange_ = range;
	animation_->setAnimationActive(range->channelName, range->range);
	animation_->startAnimation();
}

void NPCController::startNavigate() {
	if (!currentBehaviour_.target) {
		REGEN_WARN("No target place for navigation.");
		return;
	}
	REGEN_INFO("NPC " << tfIdx_ << " navigating to place of type " << currentBehaviour_.target->type);
	// pick a random point close to the target place
	Vec3f target3D = currentBehaviour_.target->pos->getVertex(0).r;
	Vec2f target = Vec2f(target3D.x, target3D.z) + Vec2f(
			1.0f + (math::random<float>() - 0.5f),
			1.0f + (math::random<float>() - 0.5f)
	) * currentBehaviour_.target->radius;
	// pick a random orientation vector
	auto orientation = Vec3f(
			currentDir_.x,
			static_cast<float>((static_cast<double>(rand()) / RAND_MAX - 0.5)*M_PI),
			currentDir_.z
	);
	orientation = currentDir_;

	bezierPath_.p0 = Vec2f(currentPos_.x, currentPos_.z);
	bezierPath_.p3 = target;
	// compute control points using Euler angles
	float directDistance = (bezierPath_.p0 - bezierPath_.p3).length() * 0.75f;
	// add base orientation
	float angle_rad1 = currentDir_.x + baseOrientation_;
	float angle_rad2 = orientation.x + baseOrientation_;
	bezierPath_.p1 = bezierPath_.p0 + Vec2f(cos(angle_rad1), sin(angle_rad1)) * directDistance;
	bezierPath_.p2 = bezierPath_.p3 + Vec2f(cos(angle_rad2), sin(angle_rad2)) * directDistance;
	// TODO: make sure control points are within the map bounds
	//intersectWithBounds(bezierPath_.p1, bezierPath_.p0, mapBounds_);
	//intersectWithBounds(bezierPath_.p2, bezierPath_.p3, mapBounds_);
	float bezierLength = bezierPath_.length1();

	// compute dt based on distance and speed
	float dt = bezierLength / (currentBehaviour_.motion==MOTION_RUN ? runSpeed_ : walkSpeed_);
	// set the target position
	setTarget(
		Vec3f(target.x, getHeight(target), target.y),
		orientation,
		dt);
}

void NPCController::updateController(double dt) {
	if (it_ == frames_.end()) {
		// currently no movement.
		if (animation_->isNodeAnimationActive()) {
			if (isLastAnimationMovement_) {
				// animation is movement, deactivate
				animation_->stopNodeAnimation();
			}
			return;
		}
	}
	else {
		// movement active
		if (!animation_->isNodeAnimationActive()) {
			// animation not active, activate
			if (lastRange_) {
				animation_->setAnimationActive(lastRange_->channelName, lastRange_->range);
				animation_->startAnimation();
			}
		}
		return;
	}

	auto lastMovement = currentBehaviour_.motion;
	updateBehavior();
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
			startNavigate();
			startMotion();
			break;
		default:
			isLastAnimationMovement_ = false;
			startMotion();
			//auto &ranges = behaviorRanges_[behavior_];
			//auto &range = out[0];
			//animation_->setAnimationActive(range->channelName, range->range);
			break;
	}
}

void NPCController::setIdle() {
	REGEN_INFO("NPC " << tfIdx_ << " is now idle.");
	currentBehaviour_.state = STATE_IDLE;
	currentBehaviour_.motion = MOTION_IDLE;
	currentBehaviour_.target = {};
	currentBehaviour_.source = {};
	//currentBehaviour_.patient = {};
	//currentBehaviour_.instrument = {};
}

void NPCController::updateBehavior() {
	Behaviour &b = currentBehaviour_;

	// Helper lambdas
	auto pickRandomPlace = [](const std::vector<ref_ptr<Place>> &places) -> ref_ptr<Place> {
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
			b.state = STATE_GO_HOME;
			b.target = pickRandomPlace(homePlaces_);
		}
		else if (!gatheringPlaces_.empty() && r < 0.5f) {
			b.state = STATE_GO_SOCIALIZE;
			b.target = pickRandomPlace(gatheringPlaces_);
		}
		else if (!spiritualPlaces_.empty() && r < (0.7f + spirituality_ * 0.3f)) {
			b.state = STATE_GO_PRAY;
			b.target = pickRandomPlace(spiritualPlaces_);
		}
		else if (!patrolPoints_.empty()) {
			b.state = STATE_GO_PATROL;
			b.target = pickRandomPlace(patrolPoints_);
		}
		else {
			// No goal → just idle
			setIdle();
		}
		if (b.state != STATE_IDLE) {
			// Set the movement type
			// TODO: think of some variations here
			//b.movement = (math::random<float>() < 0.7f) ? MOVEMENT_WALK : MOVEMENT_RUN;
			b.motion = MOTION_WALK;
		}
		break;
	}
	case STATE_GO_HOME:
	case STATE_GO_SOCIALIZE:
	case STATE_GO_PRAY:
	case STATE_GO_PATROL: {
		if (!b.target) {
			setIdle();
			break;
		}

		// Check if we've arrived
		auto targetPos = b.target->pos->getVertex(0);
		float distance = dist2D(pos, targetPos.r);
		if (distance < b.target->radius) {
			REGEN_INFO("NPC " << tfIdx_ << " arrived at place of type " << b.target->type);
			// Switch to "being there" state
			switch (b.state) {
				case STATE_GO_HOME:      b.state = STATE_AT_HOME;     break;
				case STATE_GO_SOCIALIZE: b.state = STATE_SOCIALIZE;   break;
				case STATE_GO_PRAY:      b.state = STATE_PRAYING;     break;
				case STATE_GO_PATROL:    b.state = STATE_PATROLLING;  break;
				default: break;
			}
			// For now just switch to idle when arriving.
			setIdle();
		} else {
			// Still traveling
		}
		break;
	}
	case STATE_AT_HOME: {
		// TODO: Implement at home behavior
		// Stay for a short time before going idle again
		if ((math::randomInt() % 100) < 3) {
			setIdle();
		}
		break;
	}
	case STATE_SOCIALIZE: {
		// TODO: Implement socialize behavior
		// Stay for a short time before going idle again
		if ((math::randomInt() % 100) < 3) {
			setIdle();
		}
		break;
	}
	case STATE_PATROLLING: {
		// TODO: Implement patrolling behavior
		// Stay for a short time before going idle again
		if ((math::randomInt() % 100) < 3) {
			setIdle();
		}
		break;
	}
	case STATE_PRAYING: {
		// TODO: Implement praying behavior
		// Stay for a short time before going idle again
		if ((math::randomInt() % 100) < 3) {
			setIdle();
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
	case STATE_SLEEPING: {
		// TODO: Implement sleeping behavior
		if ((math::randomInt() % 100) < 2) {
			setIdle();
		}
		break;
	}
	default:
		break;
	}
}

std::ostream &regen::operator<<(std::ostream &out, const NPCController::PlaceType &v) {
	switch (v) {
		case NPCController::PLACE_GATHERING:
			return out << "GATHERING";
		case NPCController::PLACE_HOME:
			return out << "HOME";
		case NPCController::PLACE_PATROL_POINT:
			return out << "PATROL_POINT";
		case NPCController::PLACE_SPIRITUAL:
			return out << "SPIRITUAL";
	}
	return out;
}

std::istream &regen::operator>>(std::istream &in, NPCController::PlaceType &v) {
	std::string val;
	in >> val;
	boost::to_upper(val);
	if (val == "GATHERING") v = NPCController::PLACE_GATHERING;
	else if (val == "HOME") v = NPCController::PLACE_HOME;
	else if (val == "PATROL_POINT" || val == "PATROL") v = NPCController::PLACE_PATROL_POINT;
	else if (val == "SPIRITUAL") v = NPCController::PLACE_SPIRITUAL;
	else {
		REGEN_WARN("Unknown place type '" << val << "'. Using HOME.");
		v = NPCController::PLACE_HOME;
	}
	return in;
}
