#ifndef REGEN_ANIMATION_NODE_H_
#define REGEN_ANIMATION_NODE_H_

#include <regen/utility/ref-ptr.h>
#include <regen/math/matrix.h>
#include <regen/math/quaternion.h>
#include <regen/animations/animation.h>

#include <unordered_map>
#include <vector>

namespace regen {
	/**
	 * \brief A skeletal animation.
	 */
	class NodeAnimation : public Animation {
	public:
		/**
		 * \brief A key frame containing a 3 dimensional vector.
		 */
		struct KeyFrame3f {
			double time; /**< frame timestamp. **/
			Vec3f value; /**< frame value. **/
		};

		/**
		 * \brief Key frame of bone rotation.
		 */
		struct KeyFrameQuaternion {
			double time; /**< frame timestamp. **/
			Quaternion value; /**< frame value. **/
		};

		/**
		 * Defines behavior for first or last key frame.
		 */
		enum Behavior {
			/**
			 * The value from the default node transformation is taken.
			 */
			BEHAVIOR_DEFAULT = 0x0,
			/**
			 * The nearest key value is used without interpolation.
			 */
			BEHAVIOR_CONSTANT = 0x1,
			/**
			 * The value of the nearest two keys is linearly
			 * extrapolated for the current time value.
			 */
			BEHAVIOR_LINEAR = 0x2,
			/**
			 * The animation is repeated.
			 * If the animation key go from n to m and the current
			 * time is t, use the value at (t-n) % (|m-n|).
			 */
			BEHAVIOR_REPEAT = 0x3
		};

		/**
		 * \brief Each channel affects a single node.
		 */
		struct Channel {
			/**
			 * The name of the node affected by this animation. The node
			 * must exist and it must be unique.
			 */
			std::string nodeName_;
			/**
			 * Defines how the animation behaves after the last key was processed.
			 * The default value is ANIM_BEHAVIOR_DEFAULT
			 * (the original transformation matrix of the affected node is taken).
			 */
			Behavior postState;
			/**
			 * Defines how the animation behaves before the first key is encountered.
			 * The default value is ANIM_BEHAVIOR_DEFAULT
			 * (the original transformation matrix of the affected node is used).
			 */
			Behavior preState;
			ref_ptr<std::vector<KeyFrame3f> > scalingKeys_; /**< Scaling key frames. */
			ref_ptr<std::vector<KeyFrame3f> > positionKeys_; /**< Position key frames. */
			ref_ptr<std::vector<KeyFrameQuaternion> > rotationKeys_; /**< Rotation key frames. */
		};

		/**
		 * \brief A node in a skeleton with parent and children.
		 */
		class Node {
		public:
			// The node name.
			std::string name;
			// The parent node.
			ref_ptr<Node> parent;
			// The node children.
			std::vector<ref_ptr<Node>> children;
			// Local transformation matrix.
			Mat4f localTransform;
			Mat4f globalTransform; // temporary storage
			// Matrix that transforms from mesh space to bone space in bind pose.
			Mat4f offsetMatrix;
			// per-instance offsetMatrix * nodeTransform * inverseTransform
			std::vector<Mat4f> boneTransformationMatrix;
			// The index of the currently active animation channel
			int32_t channelIndex;
			bool isBoneNode;

			/**
			 * @param name the node name.
			 * @param parent the parent node.
			 */
			Node(const std::string &name, const ref_ptr<Node> &parent);

			/**
			 * Add a node child.
			 * @param child
			 */
			void addChild(const ref_ptr<Node> &child);

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
			Stack<Node *> traversalStack;
		};

		/**
		 * Event data for node animation events.
		 */
		class NodeEventData : public EventData {
		public:
			NodeEventData() = default;
			~NodeEventData() override = default;
			uint32_t instanceIdx = 0;
		};

		/**
		 * @param rootNode animation tree.
		 */
		explicit NodeAnimation(const ref_ptr<Node> &rootNode, uint32_t numInstances = 1);

		/**
		 * @return true if the animation is active.
		 */
		bool isNodeAnimationActive(uint32_t instanceIdx) const;

		/**
		 * Add an animation.
		 * @param animationName the animation name.
		 * @param channels the key frames.
		 * @param duration animation duration.
		 * @param ticksPerSecond number of animation ticks per second.
		 * @return the animation index.
		 */
		int32_t addChannels(
				const std::string &animationName,
				ref_ptr<std::vector<Channel> > &channels,
				double duration,
				double ticksPerSecond);

		/**
		 * Sets animation tick range.
		 * @param name the animation name.
		 * @param tickRange the tick range.
		 */
		void setAnimationActive(uint32_t instanceIdx, const std::string &name, const Vec2d &tickRange);

		/**
		 * Sets animation tick range.
		 * @param index the animation index.
		 * @param tickRange the tick range.
		 */
		void setAnimationIndexActive(uint32_t instanceIdx, int32_t index, const Vec2d &tickRange);

		/**
		 * Sets tick range for the currently activated
		 * animation index.
		 * @param forcedTickRange the tick range.
		 */
		void setTickRange(uint32_t instanceIdx, const Vec2d &forcedTickRange);

		/**
		 * @param timeFactor the slow down (<1.0) / speed up (>1.0) factor.
		 */
		void set_timeFactor(double timeFactor) { timeFactor_ = timeFactor * 0.001; }

		/**
		 * @return the slow down (<1.0) / speed up (>1.0) factor.
		 */
		double timeFactor() const { return timeFactor_ * 1000.0; }

		/**
		 * Find node with given name.
		 * @param name the node name.
		 * @return the node or a null reference.
		 */
		ref_ptr<Node> findNode(const std::string &name);

		/**
		 * Stop the node animation.
		 */
		void stopNodeAnimation(uint32_t instanceIdx);

		/**
		 * @return the number of animations.
		 */
		uint32_t numAnimations() const {
			return static_cast<uint32_t>(animData_.size());
		}

		/**
		 * @return the elapsed time of the currently active animation in milliseconds.
		 */
		double elapsedTime(uint32_t instanceIdx) const;

		/**
		 * @param animationIndex the animation index.
		 * @return the ticks per second of the given animation.
		 */
		double ticksPerSecond(uint32_t animationIndex) const;

		// override
		void animate(double dt) override;

	protected:
		ref_ptr<Node> rootNode_;
		uint32_t numInstances_ = 1;
		double timeFactor_ = 0.001;

		struct Data {
			// string identifier for animation
			std::string animationName_;
			double ticksPerSecond_;
			// Duration of the animation in ticks.
			double duration_;
			// Static animation data
			ref_ptr<std::vector<Channel>> channels_;
			// per-instance + per-channel local node transformation
			std::vector<Mat4f> transforms_;
			// per-instance + per-channel remember last frame for interpolation
			std::vector<Vec3ui> lastFramePosition_;
			std::vector<Vec3ui> startFramePosition_;
		};
		std::vector<Data> animData_;

		struct InstanceData {
			int32_t animationIndex_ = -1;
			// config for currently active anim
			double startTick_ = 0.0;
			double duration_ = 0.0;
			Vec2d tickRange_ = Vec2d(0.0, 0.0);
			// milliseconds from start of animation
			double elapsedTime_ = 0.0;
			double lastTime_ = 0.0;
			// flag indicating if an animation is active
			bool active_ = false;
		};
		std::vector<InstanceData> instanceData_;
		ref_ptr<NodeEventData> eventData_ = ref_ptr<NodeEventData>::alloc();

		std::unordered_map<std::string, Node *> nameToNode_;
		std::unordered_map<std::string, int32_t> animNameToIndex_;

		Quaternion nodeRotation(uint32_t instanceIdx, Data &anim,
				const Channel &channel,
				double timeInTicks,
				uint32_t i) const;

		Vec3f nodePosition(uint32_t instanceIdx, Data &anim,
				const Channel &channel,
				double timeInTicks,
				uint32_t i) const;

		Vec3f nodeScaling(uint32_t instanceIdx, Data &anim,
				const Channel &channel,
				double timeInTicks,
				uint32_t i) const;

		ref_ptr<Node> findNode(ref_ptr<Node> &n, const std::string &name);

		void deallocateAnimationAtIndex(uint32_t instanceIdx, int32_t animationIndex);

		void stopNodeAnimation(uint32_t instanceIdx, Data &anim);
	};

	std::ostream &operator<<(std::ostream &out, const NodeAnimation::Behavior &v);

	std::istream &operator>>(std::istream &in, NodeAnimation::Behavior &v);
} // namespace

#endif /* REGEN_ANIMATION_NODE_H_ */
