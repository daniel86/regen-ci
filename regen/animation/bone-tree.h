#ifndef REGEN_BONE_TREE_H_
#define REGEN_BONE_TREE_H_

#include <regen/utility/ref-ptr.h>
#include <regen/math/matrix.h>
#include <regen/math/quaternion.h>
#include <regen/animation/animation.h>
#include <regen/animation/animation-range.h>

#include <unordered_map>
#include <vector>

#include "regen/utility/stamped.h"
#include "animation-channel.h"

namespace regen {
	/**
	 * \brief A node in a skeleton with parent and children.
	 */
	class BoneNode {
	public:
		// The node name.
		std::string name;
		// The parent node.
		ref_ptr<BoneNode> parent;
		// The node children.
		std::vector<ref_ptr<BoneNode>> children;
		// Local transformation matrix.
		Mat4f localTransform;
		Mat4f globalTransform; // temporary storage
		// Matrix that transforms from mesh space to bone space in bind pose.
		Mat4f offsetMatrix;
		// per-instance offsetMatrix * nodeTransform * inverseTransform
		std::vector<Mat4f> boneTransformationMatrix;
		// The index of the currently active animation channel
		int32_t channelIndex = -1;
		bool isBoneNode;
		bool localDirty = true;
		bool globalDirty = true;

		/**
		 * @param name the node name.
		 * @param parent the parent node.
		 */
		BoneNode(const std::string &name, const ref_ptr<BoneNode> &parent);

		/**
		 * Add a node child.
		 * @param child
		 */
		void addChild(const ref_ptr<BoneNode> &child);

		/**
		 * Recursively updates the internal node transformations from the given matrix array.
		 * @param transforms transformation matrices.
		 */
		void updateTransforms(uint32_t instanceIdx, Mat4f *transforms);

		/**
		 * Concatenates all parent transforms to get the global transform for this node.
		 */
		void calculateGlobalTransform();

	protected:
		Stack<BoneNode *> traversalStack;
	};

	/**
	 * \brief A skeletal animation.
	 */
	class BoneTree : public Animation {
	public:
		/**
		 * The handle for an active animation.
		 * This is returned by startBoneAnimation() and used to
		 * identify the animation in other functions.
		 */
		using AnimationHandle = int32_t;

		/**
		 * @param rootNode animation tree.
		 */
		explicit BoneTree(const ref_ptr<BoneNode> &rootNode, uint32_t numInstances = 1);

		/**
		 * Add an animation.
		 * @param trackName the track name.
		 * @param channels the key frames.
		 * @param duration animation duration.
		 * @param ticksPerSecond number of animation ticks per second.
		 * @return the animation index.
		 */
		int32_t addChannels(
				const std::string &trackName,
				ref_ptr<std::vector<AnimationChannel>> &channels,
				double duration,
				double ticksPerSecond);

		/**
		 * Get the index of an animation track by name.
		 * @param trackName the track name.
		 * @return the track index or -1 if not found.
		 */
		int32_t getTrackIndex(const std::string &trackName) const;

		/**
		 * Start an animation within a given track.
		 * Multiple animations can be active at the same time.
		 * @param instanceIdx the instance index.
		 * @param trackIdx the animation track index as returned by getTrackIndex().
		 * @param tickRange the tick range, negative values indicate full range.
		 * @return the animation handle or -1 if the animation could not be started.
		 */
		AnimationHandle startBoneAnimation(uint32_t instanceIdx, int32_t trackIdx, const Vec2d &tickRange);

		/**
		 * Stop an active animation.
		 * @param instanceIdx the instance index.
		 * @param animHandle the animation handle as returned by startBoneAnimation().
		 */
		void stopBoneAnimation(uint32_t instanceIdx, AnimationHandle animHandle);

		/**
		 * @param instanceIdx the instance index.
		 * @param animHandle the animation handle as returned by startBoneAnimation().
		 * @return true if the animation is active.
		 */
		bool isBoneAnimationActive(uint32_t instanceIdx, AnimationHandle animHandle) const;

		/**
		 * @param instanceIdx the instance index.
		 * @param animHandle the animation handle as returned by startBoneAnimation().
		 * @return the weight of the animation in the range [0.0 .. 1.0].
		 */
		float animationWeight(uint32_t instanceIdx, AnimationHandle animHandle) const;

		/**
		 * Sets the weight of an active animation.
		 * This is used for blending multiple animations.
		 * @param instanceIdx the instance index.
		 * @param animHandle the animation handle as returned by startBoneAnimation().
		 * @param weight the weight of the animation in the range [0.0 .. 1.0].
		 */
		void setAnimationWeight(uint32_t instanceIdx, AnimationHandle animHandle, float weight);

		/**
		 * Sets tick range for a given animation handle.
		 * @param instanceIdx the instance index.
		 * @param animHandle the animation handle.
		 * @param tickRange the tick range, negative values indicate full range.
		 */
		void setTickRange(uint32_t instanceIdx, AnimationHandle animHandle, const Vec2d &tickRange);

		/**
		 * @param timeFactor the slow down (<1.0) / speed up (>1.0) factor.
		 */
		void setTimeFactor(double timeFactor) { timeFactor_ = timeFactor * 0.001; }

		/**
		 * @return the slow down (<1.0) / speed up (>1.0) factor.
		 */
		double timeFactor() const { return timeFactor_ * 1000.0; }

		/**
		 * Each track has a name and contains multiple channels.
		 * Only one track can be active at a time, however multiple ranges within
		 * one track can be active.
		 * @return the number of animation tracks.
		 */
		uint32_t numAnimationTracks() const {
			return static_cast<uint32_t>(animTracks_.size());
		}

		/**
		 * @return the number of currently active animation ranges.
		 */
		uint32_t numActiveRanges(uint32_t instanceIdx) const {
			return instanceData_[instanceIdx].numActiveRanges_;
		}

		/**
		 * @return the elapsed time of an active animation in milliseconds.
		 */
		double elapsedTime(uint32_t instanceIdx, AnimationHandle animHandle) const;

		/**
		 * @param trackIdx the animation track index.
		 * @return the ticks per second of the given animation track.
		 */
		double ticksPerSecond(uint32_t trackIdx) const;

		// override
		void animate(double dt) override;

		/**
		 * Find node with given name.
		 * @param name the node name.
		 * @return the node or a null reference.
		 */
		ref_ptr<BoneNode> findNode(const std::string &name);

		/**
		 * Event data for node animation events.
		 */
		class BoneEvent : public EventData {
		public:
			BoneEvent() = default;
			~BoneEvent() override = default;
			uint32_t instanceIdx = 0;
			uint32_t rangeIdx = 0;
		};

		struct ActiveRange {
			// -1 indicates that the range is inactive
			int32_t animationIndex_ = -1;
			// config for currently active anim
			double startTick_ = 0.0;
			double duration_ = 0.0;
			Vec2d tickRange_ = Vec2d(0.0, 0.0);
			// milliseconds from start of animation
			double elapsedTime_ = 0.0;
			// last update time in ticks
			double lastTime_ = 0.0;
			double timeInTicks_ = 0.0;
			float weight_ = 1.0;
			// Remember last frame of each channel for interpolation.
			std::vector<Vec3ui> lastFramePosition_;
			std::vector<Vec3ui> startFramePosition_;
			std::vector<Vec3f> lastInterpolation_;
		};
	protected:
		ref_ptr<BoneNode> rootNode_;
		uint32_t numInstances_ = 1;
		double timeFactor_ = 0.001;

		/**
		 * A track in the bone animation.
		 * One animation may have different tracks, each with its own set of channels.
		 */
		struct Track {
			// string identifier for animation
			std::string trackName_;
			double ticksPerSecond_;
			// Duration of the animation in ticks.
			double duration_;
			// Static animation data
			ref_ptr<std::vector<AnimationChannel>> channels_;
			// per-instance + per-channel local node transformation
			std::vector<Mat4f> transforms_;
		};
		std::vector<Track> animTracks_;
		// The animation track which is currently in use.
		Track *currentTrack_ = nullptr;

		struct InstanceData {
			// An array of active ranges, however the array only grows so we may have
			// inactive ranges in between.
			std::vector<ActiveRange> ranges_;
			// The number of active ranges, i.e. animations that are active in parallel.
			// This is usually a very small number.
			uint32_t numActiveRanges_ = 0;
			// The last sum of weights of all active ranges.
			float lastWeightSum_ = 0.0f;
		};
		std::vector<InstanceData> instanceData_;
		ref_ptr<BoneEvent> eventData_ = ref_ptr<BoneEvent>::alloc();

		// tmp storage
		Stamped<Vec3f> tmpPos_;
		Stamped<Vec3f> tmpScale_;
		Stamped<Quaternion> tmpRot_;

		// tmp storage
		Vec3f accPos_;
		Vec3f accScale_;
		Quaternion accRot_;

		std::vector<BoneNode*> nodes_;
		std::unordered_map<std::string, uint32_t> nameToNode_;
		std::unordered_map<std::string, int32_t> trackNameToIndex_;

		void setTickRange(ActiveRange &ar, const Vec2d &forcedTickRange);

		Quaternion nodeRotation(ActiveRange &ar, const AnimationChannel &channel, bool &isDirty, uint32_t i);

		Vec3f nodePosition(ActiveRange &ar, const AnimationChannel &channel, bool &isDirty, uint32_t i);

		Vec3f nodeScaling(ActiveRange &ar, const AnimationChannel &channel, bool &isDirty, uint32_t i);

		static ref_ptr<BoneNode> findNode(ref_ptr<BoneNode> &n, const std::string &name);
	};
} // namespace

#endif /* REGEN_BONE_TREE_H_ */
