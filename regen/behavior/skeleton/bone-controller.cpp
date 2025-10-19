#include "bone-controller.h"

using namespace regen;

//#define BONE_CONTROLLER_DEBUG_ACTION ActionType::ATTACKING
//#define BONE_CONTROLLER_DEBUG_STATE
#if defined(BONE_CONTROLLER_DEBUG_ACTION) && !defined(BONE_CONTROLLER_DEBUG_STATE)
#define BONE_CONTROLLER_DEBUG_STATE
#endif

#ifdef BONE_CONTROLLER_DEBUG_ACTION
#define BONE_CTRL_DEBUG(act, ...) do { if (act == BONE_CONTROLLER_DEBUG_ACTION) {\
	std::stringstream ctrl_ss;\
	ctrl_ss << "[" << instanceIdx_ << "] ";\
	ctrl_ss << "Action " << act << " ";\
	ctrl_ss << __VA_ARGS__;\
	Logging::log(Logging::INFO, ctrl_ss.str(), __FILE__, __LINE__);\
	}} while(0)
#elif BONE_CONTROLLER_DEBUG_STATE
#define BONE_CTRL_DEBUG(act, ...) do {\
	std::stringstream ctrl_ss;\
	ctrl_ss << "[" << instanceIdx_ << "] ";\
	ctrl_ss << "Action " << act << " " << __VA_ARGS__;\
	Logging::log(Logging::INFO, ctrl_ss.str(), __FILE__, __LINE__);\
	} while(0)
#else
#define BONE_CTRL_DEBUG(act, ...)
#endif

// FIXME: I think there might be a rare edge case bug when multiple actions that map
//         to the same motion type are active at the same time.

BoneController::BoneController(uint32_t instanceIdx, const ref_ptr<BoneAnimationItem> &animItem)
		: instanceIdx_(instanceIdx), animItem_(animItem) {
	// Load the track index for each range.
	for (auto &range: animItem_->ranges) {
		if (!range.trackName.empty()) {
			range.trackIndex = animItem_->animation->getTrackIndex(range.trackName);
			if (range.trackIndex < 0) {
				REGEN_WARN("Unable to find track name '" << range.trackName
					<< "' for animation range '" << range.name << "'.");
				continue;
			}
		}
	}

	// Fill in the motion clips for each motion type.
	motionToData_.resize(static_cast<size_t>(MotionType::MOTION_LAST));
	for (uint32_t clipIdx = 0; clipIdx < animItem_->clips.size(); ++clipIdx) {
		auto &clip = animItem_->clips[clipIdx];
		motionToData_[static_cast<int>(clip.motion)].clips.push_back(clipIdx);
	}

	// Extract some special motion parameters that are useful for navigation.
	if (!motionToData_[static_cast<int>(MotionType::WALK)].clips.empty()) {
		auto walkClip = animItem_->clips[motionToData_[static_cast<int>(MotionType::WALK)].clips[0]];
		auto walkTPS = animItem_->animation->ticksPerSecond(walkClip.range->trackIndex);
		walkTime_ = 1000.0f * (walkClip.range->range.y - walkClip.range->range.x) / walkTPS;
	}
	if (!motionToData_[static_cast<int>(MotionType::RUN)].clips.empty()) {
		auto runClip = animItem_->clips[motionToData_[static_cast<int>(MotionType::RUN)].clips[0]];
		auto runTPS = animItem_->animation->ticksPerSecond(runClip.range->trackIndex);
		runTime_ = 1000.0f * (runClip.range->range.y - runClip.range->range.x) / runTPS;
	}

	// initialize the action-to-motion mapping.
	// note: behavior tree selects actions, we map here to motions, and have
	//       a set of animation clips for each motion type.
	// TODO: this could be done in the scene file maybe, but somewhere else or?
	actionToMotion_.resize(static_cast<size_t>(ActionType::LAST_ACTION));
	actionToMotion_[static_cast<size_t>(ActionType::IDLE)] = { MotionType::IDLE };
	actionToMotion_[static_cast<size_t>(ActionType::OBSERVING)] = { MotionType::IDLE };
	actionToMotion_[static_cast<size_t>(ActionType::SITTING)] = { MotionType::SIT };
	actionToMotion_[static_cast<size_t>(ActionType::PRAYING)] = { MotionType::CROUCH };
	actionToMotion_[static_cast<size_t>(ActionType::ATTACKING)] = { MotionType::ATTACK };
	actionToMotion_[static_cast<size_t>(ActionType::BLOCKING)] = { MotionType::BLOCK };
	actionToMotion_[static_cast<size_t>(ActionType::SLEEPING)] = { MotionType::SLEEP };
	actionToMotion_[static_cast<size_t>(ActionType::NAVIGATING)] = { MotionType::WALK };
	actionToMotion_[static_cast<size_t>(ActionType::WALKING)] = { MotionType::WALK };
	actionToMotion_[static_cast<size_t>(ActionType::PATROLLING)] = { MotionType::WALK };
	actionToMotion_[static_cast<size_t>(ActionType::STROLLING)] = { MotionType::WALK };
	actionToMotion_[static_cast<size_t>(ActionType::FLEEING)] = { MotionType::RUN };
	actionToMotion_[static_cast<size_t>(ActionType::CONVERSING)] = {
		MotionType::IDLE, MotionType::AGREE, MotionType::DISAGREE };
	// by default no motion for these actions.
	actionToMotion_[static_cast<size_t>(ActionType::FLOCKING)] = { MotionType::MOTION_LAST };
}

int32_t BoneController::getAnimationHandle(MotionType type) const {
	return motionToData_[static_cast<int>(type)].handle;
}

bool BoneController::isCurrentBoneAnimation(MotionType type) const {
	for (uint32_t i=0; i<numActiveMotions_; i++) {
		if (activeMotions_[i] == type) return true;
	}
	return false;
}

void BoneController::setMotionWeight(MotionData &motion, float weight) {
	animItem_->animation->setAnimationWeight(instanceIdx_, motion.handle, weight);
}

void BoneController::addActiveMotion(MotionType motion) {
	if (numActiveMotions_ >= activeMotions_.size()) {
		activeMotions_.resize(activeMotions_.size() + 4);
	}
	activeMotions_[numActiveMotions_++] = motion;
}

void BoneController::updateDesiredMotions(const ActionType *desiredActions, uint32_t numDesiredActions) {
	auto anim = animItem_->animation;

	for (uint32_t i = 0; i < numDesiredActions; ++i) {
		MotionType desiredMotion = MotionType::MOTION_LAST;
		ActionType desiredAction = desiredActions[i];
		bool selectNewClip = false;

		// Check if the action is currently being performed.
		for (uint32_t j = 0; j < numActiveMotions_; ++j) {
			MotionType currentMotion = activeMotions_[j];
			auto &motionData = motionToData_[static_cast<int>(currentMotion)];
			if (motionData.action == desiredAction) {
				// For some action types, allow rapid changing of motion types.
				// For now just allow that for ATTACK actions.
				// Later, this should be defined in an ontology!
				bool stickToCurrent = (desiredAction != ActionType::ATTACKING ||
					anim->isBoneAnimationActive(instanceIdx_, motionData.handle));
				if (stickToCurrent) {
					// The current animation clip segment is still active, or should be
					// repeated.
					desiredMotion = currentMotion;
				} else {
					// TODO: Interact with clips here?
					//     - If clip was starting or looping, and there is an end segment, then
					//       start the end segment, and set desiredMotion = currentMotion
					//       before activating another motion.
					selectNewClip = true;
					break;
				}

			}
		}
		// If not active, then select a motion for the action.
		if (desiredMotion == MotionType::MOTION_LAST) {
			auto &possibleMotions = actionToMotion_[static_cast<size_t>(desiredAction)];
			if (!possibleMotions.empty()) {
				if (possibleMotions.size() == 1) {
					desiredMotion = possibleMotions[0];
				} else {
					desiredMotion = possibleMotions[math::randomInt() % possibleMotions.size()];
				}
			} else {
				REGEN_WARN("No motion defined for action " << desiredAction << ".");
				continue;
			}
		}
		if (desiredMotion == MotionType::MOTION_LAST) {
			continue;
		}

		auto &motionData = motionToData_[static_cast<int>(desiredMotion)];
		motionData.desired = true;
		motionData.action = desiredAction;
		if (selectNewClip && motionData.status != MOTION_INACTIVE) {
			// Select a new clip for this motion type.
			startMotionClip(motionData, 0.0f);
			BONE_CTRL_DEBUG(desiredAction, "selecting new clip");
		}
		if (motionData.status == MOTION_FADING_OUT) {
			// was fading out, but is now desired again, so make active again.
			// note: we can keep the current weight and fade time.
			BONE_CTRL_DEBUG(desiredAction, "fading in");
			motionData.status = MOTION_FADING_IN;
		} else if (motionData.status == MOTION_INACTIVE) {
			// was inactive, so we can start it right away.
			BONE_CTRL_DEBUG(desiredAction, "is now starting");
			motionData.status = MOTION_STARTING;
			addActiveMotion(desiredMotion);
		} else { // ACTIVE, STARTING, FADING_IN
			// already active, nothing to do.
			continue;
		}
	}
}

bool BoneController::startMotionClip(MotionData &motion, float initialWeight) {
	if (motion.clips.empty()) return false;

	// Select a clip for this motion type.
	uint32_t clipIndex;
	if (motion.clips.size() == 1) {
		clipIndex = 0;
	} else {
		clipIndex = math::randomInt() % motion.clips.size();
	}
	auto &clip = animItem_->clips[motion.clips[clipIndex]];
	// begin range is optional, if not present we loop the clip directly.
	bool hasBeginRange = (clip.begin != nullptr);

	motion.lastClip = clipIndex;
	motion.clipStatus = (hasBeginRange ? CLIP_BEGINNING : CLIP_LOOPING);

	if (initialWeight + 1e-5f >= 1.0f) {
		initialWeight = 1.0f;
		motion.status = MOTION_ACTIVE;
		motion.fadeTime = motionFadeDuration_;
	} else {
		motion.status = MOTION_FADING_IN;
		motion.fadeTime = 0.0f;
	}
	motion.handle = animItem_->animation->startBoneAnimation(
		instanceIdx_, clip.range->trackIndex, clip.range->range);
	setMotionWeight(motion, initialWeight);
	animItem_->animation->startAnimation();

	return true;
}

void BoneController::stopMotionClip(MotionData &motion) {
	animItem_->animation->stopBoneAnimation(instanceIdx_, motion.handle);
	motion.handle = -1;
	motion.status = MOTION_INACTIVE;
}

void BoneController::updateMotionClip(MotionData &motion, MotionClip &clip) {
	// animation-status is ACTIVE | FADING_OUT | FADING_IN
	// clip-status is BEGINNING | LOOPING | ENDING
	auto &anim = animItem_->animation;
	// Check if the current segment is still active, if so we are done.
	bool isActive = anim->isBoneAnimationActive(instanceIdx_, motion.handle);
	if (isActive) return;

	bool hasEndRange = (clip.end != nullptr);
	bool isFadingOut = (motion.status == MOTION_FADING_OUT);

	if (motion.clipStatus == CLIP_BEGINNING) {
		// Beginning segment is done, now start looping segment if any...
		if (clip.range && (!isFadingOut || !hasEndRange)) {
			// Activate the looping segment, unless we are fading out and have an end segment.
			// In that case we skip the looping segment and go directly to the end segment.
			motion.handle = anim->startBoneAnimation(instanceIdx_, clip.range->trackIndex, clip.range->range);
			motion.clipStatus = CLIP_LOOPING;
			BONE_CTRL_DEBUG(motion.action, "is now looping");
		} else if (clip.end) {
			// Start the ending segment directly.
			motion.handle = anim->startBoneAnimation(instanceIdx_, clip.end->trackIndex, clip.end->range);
			motion.clipStatus = CLIP_ENDING;
			BONE_CTRL_DEBUG(motion.action, "is now ending");
		} else if (!isFadingOut) {
			// Switch back to beginning.
			motion.handle = anim->startBoneAnimation(instanceIdx_, clip.begin->trackIndex, clip.begin->range);
			motion.clipStatus = CLIP_BEGINNING;
		} else {
			// TODO: Make BoneTree return the last transform until faded out?
			//           Or just deactivate the motion early?
			motion.status = MOTION_INACTIVE;
			BONE_CTRL_DEBUG(motion.action, "is now inactive");
		}
	}
	else if (motion.clipStatus == CLIP_LOOPING) {
		// Looping segment is done.
		if (!isFadingOut) {
			// Restart the looping segment.
			//BONE_CTRL_DEBUG(motion.action, "is now restarting");
			motion.handle = anim->startBoneAnimation(instanceIdx_, clip.range->trackIndex, clip.range->range);
		} else if (clip.end) {
			// Switch to ending segment if we are fading out, i.e. if the motion is no longer desired.
			motion.handle = anim->startBoneAnimation(instanceIdx_, clip.end->trackIndex, clip.end->range);
			motion.clipStatus = CLIP_ENDING;
			BONE_CTRL_DEBUG(motion.action, "is now ending");
		} else {
			// No ending segment, so toggle the motion off.
			motion.status = MOTION_INACTIVE;
			BONE_CTRL_DEBUG(motion.action, "is now active");
		}
	}
	else if (motion.clipStatus == CLIP_ENDING) {
		// Ending segment is done, so toggle the motion off.
		motion.status = MOTION_INACTIVE;
		BONE_CTRL_DEBUG(motion.action, "is now inactive");
	}
}

void BoneController::updateMotionClip(MotionData &motion, float dt_s) {
	// Each clip is made up of one or more segments that are played in sequence.
	if (motion.status != MOTION_INACTIVE && motion.status != MOTION_STARTING) {
		auto &clip = animItem_->clips[motion.clips[motion.lastClip]];
		updateMotionClip(motion, clip);
	}
	// Update the motion state machine and blend weights.
	switch (motion.status) {
		case MOTION_ACTIVE:
		case MOTION_INACTIVE:
			// nothing to do
			break;
		case MOTION_STARTING:
			startMotionClip(motion, 0.0f);
			BONE_CTRL_DEBUG(motion.action, "started");
			break;
		case MOTION_FADING_IN:
			motion.fadeTime += dt_s;
			if (motion.fadeTime >= motionFadeDuration_) {
				BONE_CTRL_DEBUG(motion.action, "fully faded in");
				motion.status = MOTION_ACTIVE;
				setMotionWeight(motion, 1.0f);
			} else {
				setMotionWeight(motion, motion.fadeTime / motionFadeDuration_);
			}
			break;
		case MOTION_FADING_OUT:
			motion.fadeTime -= dt_s;
			if (motion.fadeTime <= 0.0f) {
				stopMotionClip(motion);
				BONE_CTRL_DEBUG(motion.action, "stopped after fade out");
			} else {
				setMotionWeight(motion, motion.fadeTime / motionFadeDuration_);
			}
			break;
	}
}

void BoneController::updateBoneController(const Blackboard &kb, float dt_s) {
	// Unset all desired flags.
	for (uint32_t activeIdx = 0; activeIdx < numActiveMotions_; ++activeIdx) {
		auto &motion = motionToData_[static_cast<int>(activeMotions_[activeIdx])];
		motion.desired = false;
	}

	// Map current actions to motions, mark only these as desired.
	if (kb.hasCurrentAction()) {
		updateDesiredMotions(kb.currentActions(), kb.numCurrentActions());
	}

	// Update the state of all active motions.
	uint32_t newNumActiveMotions = 0;
	for (uint32_t activeIdx = 0; activeIdx < numActiveMotions_; activeIdx++) {
		auto &motion = motionToData_[static_cast<int>(activeMotions_[activeIdx])];
		// Flip some motion state before update
		if (!motion.desired && motion.status != MOTION_FADING_OUT) {
			// Not desired anymore, so fade out.
			BONE_CTRL_DEBUG(motion.action, "fading out");
			motion.status = MOTION_FADING_OUT;
			motion.fadeTime = motionFadeDuration_;
		}
		// Update the motion state, start/stop animations, set animation weights for fading etc.
		updateMotionClip(motion, dt_s);
		// Only add motions that are still active to the new active list.
		// This will remove motions that have faded out completely.
		if (motion.status != MOTION_INACTIVE) {
			activeMotions_[newNumActiveMotions++] = activeMotions_[activeIdx];
		}
	}
	numActiveMotions_ = newNumActiveMotions;
}
