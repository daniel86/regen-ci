#ifndef SCENE_DISPLAY_ANIMATION_H_
#define SCENE_DISPLAY_ANIMATION_H_

#include <regen/animation/bone-tree.h>

using namespace regen;

inline BoneTree::AnimationHandle setAnimationRangeActive(
			const ref_ptr<BoneTree> &anim,
			uint32_t instanceIdx,
			const AnimationRange &animRange) {
	if (animRange.trackName.empty()) {
		REGEN_INFO("Starting animation range '" << animRange.name
			<< "' on track index " << animRange.trackIndex
			<< " for instance " << instanceIdx << ".");
		return anim->startBoneAnimation(instanceIdx, animRange.trackIndex, animRange.range);
	} else {
		int32_t trackIndex = anim->getTrackIndex(animRange.trackName);
		if (trackIndex < 0) {
			REGEN_WARN("Unable to find track name '" << animRange.trackName
				<< "' for animation range '" << animRange.name << "'.");
			return -1;
		}
		return anim->startBoneAnimation(instanceIdx, trackIndex, animRange.range);
	}
}

class RandomAnimationRangeUpdater : public EventHandler {
public:
	RandomAnimationRangeUpdater(const ref_ptr<BoneAnimationItem> &animItem)
			: EventHandler(), animItem_(animItem) {
		// Load the track index for each range.
		for (auto &range: animItem_->ranges) {
			if (!range.trackName.empty()) {
				range.trackIndex = animItem_->boneTree->getTrackIndex(range.trackName);
			}
		}
	}

	~RandomAnimationRangeUpdater() override = default;

	void call(EventObject *ev, EventData *data) override {
		BoneTree::BoneEvent *evData = (BoneTree::BoneEvent *) data;
		int index = rand() % animItem_->ranges.size();
		setAnimationRangeActive(animItem_->boneTree, evData->instanceIdx, animItem_->ranges[index]);
	}

protected:
	ref_ptr<BoneAnimationItem> animItem_{};
};

class FixedAnimationRangeUpdater : public EventHandler {
public:
	FixedAnimationRangeUpdater(const ref_ptr<BoneTree> &anim, const AnimationRange &animRange)
			: EventHandler(), anim_(anim), animRange_(animRange) {}

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
	explicit RandomAnimationRangeUpdater2(const ref_ptr<BoneTree> &anim) : EventHandler(), anim_(anim) {}

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

#endif /* SCENE_DISPLAY_ANIMATION_H_ */
