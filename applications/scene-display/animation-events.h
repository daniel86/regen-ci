#ifndef SCENE_DISPLAY_ANIMATION_H_
#define SCENE_DISPLAY_ANIMATION_H_

#include <list>
#include <algorithm>
#include <regen/animation/bone-tree.h>

void setAnimationRangeActive(
			const ref_ptr<BoneTree> &anim,
			uint32_t instanceIdx,
			const AnimationRange &animRange) {
	if (animRange.trackName.empty()) {
		anim->startBoneAnimation(instanceIdx, animRange.trackIndex, animRange.range);
	} else {
		int32_t trackIndex = anim->getTrackIndex(animRange.trackName);
		if (trackIndex < 0) {
			REGEN_WARN("Unable to find track name '" << animRange.trackName
				<< "' for animation range '" << animRange.name << "'.");
			return;
		}
		anim->startBoneAnimation(instanceIdx, trackIndex, animRange.range);
	}
}

class RandomAnimationRangeUpdater : public EventHandler {
public:
	RandomAnimationRangeUpdater(const ref_ptr<BoneAnimationItem> &animItem)
			: EventHandler(), animItem_(animItem) {}

	~RandomAnimationRangeUpdater() override = default;

	void call(EventObject *ev, EventData *data) override {
		BoneTree::BoneEvent *evData = (BoneTree::BoneEvent *) data;
		int index = rand() % animItem_->ranges.size();
		setAnimationRangeActive(animItem_->animation, evData->instanceIdx, animItem_->ranges[index]);
	}

protected:
	ref_ptr<BoneAnimationItem> animItem_;
};

class FixedAnimationRangeUpdater : public EventHandler {
public:
	FixedAnimationRangeUpdater(
			const ref_ptr<BoneTree> &anim,
			const AnimationRange &animRange)
			: EventHandler(),
			  anim_(anim),
			  animRange_(animRange) {}

	~FixedAnimationRangeUpdater() override = default;
	void call(EventObject *ev, EventData *data) override {
		BoneTree::BoneEvent *evData = (BoneTree::BoneEvent *) data;
		setAnimationRangeActive(anim_, evData->instanceIdx, animRange_);
	}
protected:
	ref_ptr<BoneTree> anim_;
	AnimationRange animRange_;
};


class RandomAnimationRangeUpdater2 : public EventHandler {
public:
	explicit RandomAnimationRangeUpdater2(
			const ref_ptr<BoneTree> &anim)
			: EventHandler(),
			  anim_(anim) {}

	~RandomAnimationRangeUpdater2() override = default;

	void call(EventObject *ev, EventData *data) override {
		BoneTree::BoneEvent *evData = (BoneTree::BoneEvent *) data;
		static const Vec2d fullRange(-1.0, -1.0);
		int index = rand() % anim_->numAnimationTracks();
		anim_->startBoneAnimation(index, evData->instanceIdx, fullRange);
	}

protected:
	ref_ptr<BoneTree> anim_;
};

struct KeyAnimationMapping {
	KeyAnimationMapping()
			: toggle(GL_FALSE),
			  backwards(GL_FALSE),
			  interrupt(GL_FALSE),
			  releaseInterrupt(GL_FALSE) {}

	std::string key;
	std::string press;
	std::string idle;
	GLboolean toggle;
	GLboolean backwards;
	GLboolean interrupt;
	GLboolean releaseInterrupt;
};

class KeyAnimationRangeUpdater : public EventHandler {
public:
	KeyAnimationRangeUpdater(
			const ref_ptr<BoneAnimationItem> &animItem,
			const std::map<std::string, KeyAnimationMapping> &mappings,
			const std::string &idleAnimation,
			uint32_t instanceIdx = 0)
			: EventHandler(),
			  animItem_(animItem),
			  mappings_(mappings),
			  instanceIdx_(instanceIdx){
		for (std::vector<AnimationRange>::const_iterator it = animItem_->ranges.begin(); it != animItem_->ranges.end(); ++it) {
			animRanges_[it->name] = *it;
		}
		active_ = "";
		idleAnimation_ = idleAnimation;
		startIdleAnimation();
	}

	~KeyAnimationRangeUpdater() override = default;

	void startIdleAnimation() {
		active_ = "";
		if (toggles_.empty()) {
			if (!idleAnimation_.empty()) {
				setAnimationRangeActive(animItem_->animation, instanceIdx_, animRanges_[idleAnimation_]);
			}
		} else {
			KeyAnimationMapping &m0 = mappings_[*toggles_.begin()];
			if (!m0.idle.empty()) {
				setAnimationRangeActive(animItem_->animation, instanceIdx_, animRanges_[m0.idle]);
			}
		}
	}

	void startAnimation() {
		// Turn the toggle 'off' before starting another animation.
		if (toggles_.empty()) {
			active_ = pressed_.front();
		} else {
			active_ = *toggles_.begin();
		}

		KeyAnimationMapping &m = mappings_[active_];
		Vec2d animRange = animRanges_[m.press].range;
		// For toggle animations, the animation range is processed backwards.
		if (m.toggle) {
			// Turn off toggle
			if (toggles_.count(active_) > 0) {
				toggles_.erase(active_);
				animRange = Vec2d(animRange.y, animRange.x);
			}
				// Turn on toggle
			else {
				toggles_.insert(active_);
			}
		} else if (m.backwards) {
			animRange = Vec2d(animRange.y, animRange.x);
		}
		setAnimationRangeActive(animItem_->animation, instanceIdx_, animRanges_[m.press]);
	}

	void nextAnimation() {
		// If there is a key pressed start the animation...
		if (!pressed_.empty()) {
			// Start animation associated to last pressed key
			startAnimation();
		} else {
			startIdleAnimation();
		}
	}

	void call(EventObject *evObject, EventData *data) override {
		if (data->eventID == Animation::ANIMATION_STOPPED) {
			nextAnimation();
			return;
		}
		std::string eventKey;
		GLboolean isUp;
		if (data->eventID == Scene::KEY_EVENT) {
			auto *ev = (Scene::KeyEvent *) data;
			// TODO: mapping of non-asci keys
			eventKey = REGEN_STRING((char) ev->key);
			isUp = ev->isUp;
		} else if (data->eventID == Scene::BUTTON_EVENT) {
			auto *ev = (Scene::ButtonEvent *) data;
			eventKey = REGEN_STRING("button" << ev->button);
			isUp = !ev->pressed;
		} else {
			return;
		}

		if (mappings_.count(eventKey) == 0) return;
		KeyAnimationMapping &m = mappings_[eventKey];

		if (isUp) {
			// Remember that the key was released
			pressed_.remove(eventKey);

			// Interrupt current animation
			if (m.releaseInterrupt && active_ == eventKey) {
				nextAnimation();
			}
		} else {
			// Remember that the key was pressed
			pressed_.remove(eventKey);
			pressed_.push_front(eventKey);

			if (active_.empty()) {
				// Start if no animation is active
				startAnimation();
			} else { // Interrupt current animation if allowed
				KeyAnimationMapping &a = mappings_[active_];
				if (a.interrupt) {
					startAnimation();
				}
			}
			// Avoid that the toggle turns off automatically
			if (m.toggle) {
				pressed_.remove(eventKey);
			}
		}
	}


protected:
	ref_ptr<BoneAnimationItem> animItem_;
	std::map<std::string, KeyAnimationMapping> mappings_;
	std::map<std::string, AnimationRange> animRanges_;
	std::list<std::string> pressed_;
	std::set<std::string> toggles_;
	std::string active_;
	std::string idleAnimation_;
	uint32_t instanceIdx_;
};


#endif /* SCENE_DISPLAY_ANIMATION_H_ */
