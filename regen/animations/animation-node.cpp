#include <regen/utility/logging.h>

#include "animation-node.h"

#include "regen/gl-types/queries/elapsed-time.h"

using namespace regen;

#define INTERPOLATE_QUATERNION_LINEAR
// assume identity as local root node transform.
// not sure if this is safe.
#define LOCAL_ROOT_IS_IDENTITY
//#define ANIMATION_NODE_DEBUG_TIME

////////////////

static void loadNodeNames(NodeAnimation::Node *n, std::unordered_map<std::string, NodeAnimation::Node *> &nameToNode_) {
	nameToNode_[n->name] = n;
	for (auto & it : n->children) {
		loadNodeNames(it.get(), nameToNode_);
	}
}

static void setNumInstances(NodeAnimation::Node *n, uint32_t numInstances) {
	n->boneTransformationMatrix.resize(numInstances, Mat4f::identity());
	for (auto & it : n->children) {
		setNumInstances(it.get(), numInstances);
	}
}

// Look for present frame number.
// Search from last position if time is after the last time, else from beginning
template<class T>
static void findFrameAfterTick(double tick, uint32_t &frame, const std::vector<T> &keys) {
	double dt;
	while (frame < keys.size() - 1) {
		dt = tick - keys[++frame].time;
		if (dt < 0.00001) {
			--frame;
			return;
		}
	}
}

template<class T>
static void findFrameBeforeTick(double &tick, uint32_t &frame, const std::vector<T> &keys) {
	if (keys.empty()) return;
	double dt;
	for (frame = keys.size() - 1; frame > 0;) {
		dt = tick - keys[--frame].time;
		if (dt > 0.000001) return;
	}
}

template<class T>
static inline GLdouble interpolationFactor(
		const T &key, const T &nextKey,
		double t, double duration) {
	double timeDifference = nextKey.time - key.time;
	if (timeDifference < 0.0) timeDifference += duration;
	return ((t - key.time) / timeDifference);
}

template<class T>
static inline GLboolean handleFrameLoop(T &dst,
		const uint32_t &frame, const uint32_t &lastFrame,
		const NodeAnimation::Channel &channel,
		const T &key, const T &first) {
	if (frame >= lastFrame) {
		return false;
	} else if (channel.postState == NodeAnimation::BEHAVIOR_DEFAULT) {
		dst.value = first.value;
		return true;
	} else if (channel.postState == NodeAnimation::BEHAVIOR_CONSTANT) {
		dst.value = key.value;
		return true;
	} else {
		return false;
	}
}

static uint32_t nodeIdx(const uint32_t instanceIdx, const uint32_t channelIdx, const uint32_t numChannels) {
	return instanceIdx * numChannels + channelIdx;
}

//////

NodeAnimation::NodeAnimation(const ref_ptr<Node> &rootNode, uint32_t numInstances)
		: Animation(false, true),
		  rootNode_(rootNode),
		  numInstances_(numInstances) {
	instanceData_.resize(numInstances_);
	loadNodeNames(rootNode_.get(), nameToNode_);
	setNumInstances(rootNode_.get(), numInstances_);
}

int32_t NodeAnimation::addChannels(
		const std::string &animationName,
		ref_ptr<std::vector<Channel> > &channels,
		double duration,
		double ticksPerSecond) {
	Data &data = animData_.emplace_back();
	data.animationName_ = animationName;
	data.ticksPerSecond_ = ticksPerSecond;
	data.duration_ = duration;
	data.channels_ = channels;
	const auto idx = static_cast<int32_t>(animData_.size());
	animNameToIndex_[data.animationName_] = idx;
	return idx;
}

#define ANIM_INDEX(i) std::min(animationIndex, (GLint)animData_.size()-1)

void NodeAnimation::setAnimationActive(uint32_t instanceIdx,
		const std::string &animationName, const Vec2d &forcedTickRange) {
	setAnimationIndexActive(instanceIdx, animNameToIndex_[animationName], forcedTickRange);
}

void NodeAnimation::setAnimationIndexActive(uint32_t instanceIdx,
		int32_t animationIndex, const Vec2d &forcedTickRange) {
	auto &instance = instanceData_[instanceIdx];
	if (instance.animationIndex_ == animationIndex) {
		// already active
		setTickRange(instanceIdx, forcedTickRange);
		return;
	}

	if (instance.animationIndex_ > 0) {
		instance.active_ = false;
	}
	instance.animationIndex_ = ANIM_INDEX(animationIndex);
	if (instance.animationIndex_ < 0) return;
	instance.active_ = true;

	Data &anim = animData_[instance.animationIndex_];
	const uint32_t numElements = numInstances_ * anim.channels_->size();
	anim.transforms_.resize(numElements, Mat4f::identity());
	anim.lastFramePosition_.resize(numElements);
	anim.startFramePosition_.resize(numElements);

	// set matching channel index
	for (uint32_t a = 0; a < anim.channels_->size(); a++) {
		const std::string nodeName = anim.channels_->data()[a].nodeName_;
		nameToNode_[nodeName]->channelIndex = static_cast<int32_t>(a);
	}

	instance.tickRange_.x = -1.0;
	instance.tickRange_.y = -1.0;
	setTickRange(instanceIdx, forcedTickRange);
}

void NodeAnimation::setTickRange(uint32_t instanceIdx, const Vec2d &forcedTickRange) {
	auto &instance = instanceData_[instanceIdx];
	if (instance.animationIndex_ < 0) {
		REGEN_WARN("can not set tick range without animation index set.");
		return;
	}
	Data &anim = animData_[instance.animationIndex_];
	const uint32_t numChannels = anim.channels_->size();
	const uint32_t nIdx = nodeIdx(instanceIdx,0,numChannels);

	// get first and last tick of animation
	Vec2d tickRange;
	if (forcedTickRange.x < 0.0f || forcedTickRange.y < 0.0f) {
		tickRange.x = 0.0;
		tickRange.y = anim.duration_;
	} else {
		tickRange = forcedTickRange;
	}
	if (tickRange.x == instance.tickRange_.x && tickRange.y == instance.tickRange_.y) {
		// nothing changed
		return;
	}
	instance.tickRange_ = tickRange;

	// set start frames
	if (instance.tickRange_.x < 0.00001) {
		Vec3ui *startFrameData = &anim.startFramePosition_[instanceIdx * anim.channels_->size()];
		std::memset((byte*)startFrameData, 0, sizeof(Vec3ui) * anim.channels_->size());
	} else {
		for (uint32_t a = 0; a < anim.channels_->size(); a++) {
			Channel &channel = anim.channels_->data()[a];
			Vec3ui framePos(0u);
			findFrameBeforeTick(instance.tickRange_.x, framePos.x, *channel.positionKeys_.get());
			findFrameBeforeTick(instance.tickRange_.x, framePos.y, *channel.rotationKeys_.get());
			findFrameBeforeTick(instance.tickRange_.x, framePos.z, *channel.scalingKeys_.get());
			anim.startFramePosition_[nIdx+a] = framePos;
		}
	}

	// initial last frame to start frame
	std::memcpy(
		(byte*)&anim.lastFramePosition_[nIdx],
		(byte*)&anim.startFramePosition_[nIdx],
		sizeof(Vec3ui) * anim.channels_->size());

	// set to start pos of the new tick range
	instance.lastTime_ = 0.0;
	instance.elapsedTime_ = 0.0;

	// set start tick and duration in ticks
	instance.startTick_ = instance.tickRange_.x;
	instance.duration_ = abs(instance.tickRange_.y - instance.tickRange_.x);
}

void NodeAnimation::deallocateAnimationAtIndex(uint32_t instanceIdx, int32_t animationIndex) {
	auto &instance = instanceData_[instanceIdx];
	if (instance.animationIndex_ < 0) return;
	instance.active_ = false;

	Data &anim = animData_[ANIM_INDEX(animationIndex)];
	anim.transforms_.resize(0);
	anim.startFramePosition_.resize(0);
	anim.lastFramePosition_.resize(0);
}

bool NodeAnimation::isNodeAnimationActive(uint32_t instanceIdx) const {
	auto &instance = instanceData_[instanceIdx];
	return instance.animationIndex_ >= 0;
}

void NodeAnimation::stopNodeAnimation(uint32_t instanceIdx) {
	auto &instance = instanceData_[instanceIdx];
	if (instance.animationIndex_ < 0) return;
	Data &anim = animData_[instance.animationIndex_];
	stopNodeAnimation(instanceIdx, anim);
}

void NodeAnimation::stopNodeAnimation(uint32_t instanceIdx, Data &anim) {
	auto &instance = instanceData_[instanceIdx];
	int32_t currIndex = instance.animationIndex_;
	instance.animationIndex_ = -1;
	eventData_->instanceIdx = instanceIdx;
	emitEvent(ANIMATION_STOPPED, eventData_);
	instance.elapsedTime_ = 0.0;
	instance.lastTime_ = 0;

	if (instance.animationIndex_ == currIndex) {
		// repeat, signal handler set animationIndex_=currIndex again
	} else {
		instance.active_ = false;
		deallocateAnimationAtIndex(instanceIdx, instance.animationIndex_);
	}
}

double NodeAnimation::elapsedTime(uint32_t instanceIdx) const {
	auto &instance = instanceData_[instanceIdx];
	if (instance.animationIndex_ < 0 || instance.animationIndex_ >= static_cast<int32_t>(animData_.size())) {
		return 0.0;
	}
	return instance.elapsedTime_;
}

double NodeAnimation::ticksPerSecond(uint32_t animationIndex) const {
	if (animationIndex >= animData_.size()) {
		return 0;
	}
	return animData_[animationIndex].ticksPerSecond_;
}

void NodeAnimation::animate(double milliSeconds) {
#ifdef ANIMATION_NODE_DEBUG_TIME
	static ElapsedTimeDebugger elapsedTime("NodeAnimation Update", 300);
	elapsedTime.beginFrame();
#endif

	for (uint32_t instanceIdx = 0; instanceIdx < numInstances_; instanceIdx++) {
		auto &instance = instanceData_[instanceIdx];
		if (instance.animationIndex_ < 0) continue;

		Data &anim = animData_[instance.animationIndex_];
		const uint32_t numChannels = anim.channels_->size();
		instance.elapsedTime_ += milliSeconds;
		if (instance.duration_ <= 0.0) continue;

		// map into anim's duration
		double timeInTicks = instance.elapsedTime_ * timeFactor_ * anim.ticksPerSecond_;
		if (timeInTicks > instance.duration_) {
			// for repeating we could do...
			//timeInTicks = fmod(timeInTicks, duration_);
			stopNodeAnimation(instanceIdx, anim);
			continue;
		}
		// Time runs backwards when start tick is higher then stop tick
		if (instance.tickRange_.x > instance.tickRange_.y) {
			timeInTicks = -timeInTicks;
		}
		timeInTicks += instance.startTick_;

		// update transformations
		for (uint32_t i = 0; i < anim.channels_->size(); i++) {
			const Channel &channel = anim.channels_->data()[i];
			Mat4f &m = anim.transforms_[nodeIdx(instanceIdx,i,numChannels)];

			if (channel.rotationKeys_->empty()) {
				m = Mat4f::identity();
			} else if (channel.rotationKeys_->size() == 1) {
				m = channel.rotationKeys_->data()[0].value.calculateMatrix();
			} else {
				m = nodeRotation(instanceIdx, anim, channel, timeInTicks, i).calculateMatrix();
			}

			if (channel.scalingKeys_->empty()) {
			} else if (channel.scalingKeys_->size() == 1) {
				m.scale(channel.scalingKeys_->data()[0].value);
			} else {
				m.scale(nodeScaling(instanceIdx, anim, channel, timeInTicks, i));
			}

			if (channel.positionKeys_->empty()) {
			} else if (channel.positionKeys_->size() == 1) {
				Vec3f &pos = channel.positionKeys_->data()[0].value;
				m.x[3] = pos.x;
				m.x[7] = pos.y;
				m.x[11] = pos.z;
			} else {
				Vec3f pos = nodePosition(instanceIdx, anim, channel, timeInTicks, i);
				m.x[3] = pos.x;
				m.x[7] = pos.y;
				m.x[11] = pos.z;
			}
		}

		instance.lastTime_ = timeInTicks;

		rootNode_->updateTransforms(instanceIdx,
			anim.transforms_.data() + instanceIdx * anim.channels_->size());
	}

#ifdef ANIMATION_NODE_DEBUG_TIME
	elapsedTime.push("animate");
	elapsedTime.endFrame();
#endif
}

Vec3f NodeAnimation::nodePosition(uint32_t instanceIdx, Data &anim, const Channel &channel, double timeInTicks, uint32_t i) const {
	const uint32_t elementIdx = nodeIdx(instanceIdx,i,anim.channels_->size());
	auto &instance = instanceData_[instanceIdx];
	uint32_t keyCount = channel.positionKeys_->size();
	const std::vector<KeyFrame3f> &keys = *channel.positionKeys_.get();
	KeyFrame3f pos;

	pos.value = Vec3f(0);

	// Look for present frame number.
	uint32_t lastFrame = anim.lastFramePosition_[elementIdx].x;
	uint32_t frame;
	if (instance.tickRange_.x > instance.tickRange_.y) {
		frame = (timeInTicks <= instance.lastTime_ ? lastFrame : anim.startFramePosition_[elementIdx].x);
		findFrameBeforeTick(timeInTicks, frame, keys);
	} else {
		frame = (timeInTicks >= instance.lastTime_ ? lastFrame : anim.startFramePosition_[elementIdx].x);
		findFrameAfterTick(timeInTicks, frame, keys);
	}
	anim.lastFramePosition_[elementIdx].x = frame;

	// lookup nearest two keys
	const KeyFrame3f &key = keys[frame];
	// interpolate between this frame's value and next frame's value
	if (!handleFrameLoop(pos, frame, lastFrame, channel, key, keys[0])) {
		const KeyFrame3f &nextKey = keys[(frame + 1) % keyCount];
		double fac = interpolationFactor(key, nextKey, timeInTicks, instance.duration_);
		if (fac <= 0) return key.value;
		pos.value = key.value + (nextKey.value - key.value) * static_cast<float>(fac);
	}

	return pos.value;
}

Quaternion NodeAnimation::nodeRotation(uint32_t instanceIdx, Data &anim, const Channel &channel, double timeInTicks, uint32_t i) const {
	const uint32_t elementIdx = nodeIdx(instanceIdx,i,anim.channels_->size());
	auto &instance = instanceData_[instanceIdx];
	KeyFrameQuaternion rot;
	uint32_t keyCount = channel.rotationKeys_->size();
	const std::vector<KeyFrameQuaternion> &keys = *channel.rotationKeys_.get();

	rot.value = Quaternion(1, 0, 0, 0);

	// Look for present frame number.
	uint32_t lastFrame = anim.lastFramePosition_[elementIdx].y;
	uint32_t frame;
	if (instance.tickRange_.x > instance.tickRange_.y) {
		frame = (timeInTicks <= instance.lastTime_ ? lastFrame : anim.startFramePosition_[elementIdx].y);
		findFrameBeforeTick(timeInTicks, frame, keys);
	} else {
		frame = (timeInTicks >= instance.lastTime_ ? lastFrame : anim.startFramePosition_[elementIdx].y);
		findFrameAfterTick(timeInTicks, frame, keys);
	}
	anim.lastFramePosition_[elementIdx].y = frame;

	// lookup nearest two keys
	const KeyFrameQuaternion &key = keys[frame];
	// interpolate between this frame's value and next frame's value
	if (!handleFrameLoop(rot, frame, lastFrame, channel, key, keys[0])) {
		const KeyFrameQuaternion &nextKey = keys[(frame + 1) % keyCount];
		double fac = interpolationFactor(key, nextKey, timeInTicks, instance.duration_);
		if (fac <= 0) return key.value;
#ifdef INTERPOLATE_QUATERNION_LINEAR
		rot.value.interpolateLinear(key.value, nextKey.value, static_cast<float>(fac));
#else
		rot.value.interpolate(key.value, nextKey.value, fac);
#endif
	}

	return rot.value;
}

Vec3f NodeAnimation::nodeScaling(uint32_t instanceIdx, Data &anim, const Channel &channel, double timeInTicks, uint32_t i) const {
	const uint32_t elementIdx = nodeIdx(instanceIdx,i,anim.channels_->size());
	auto &instance = instanceData_[instanceIdx];
	const std::vector<KeyFrame3f> &keys = *channel.scalingKeys_.get();
	KeyFrame3f scale;

	// Look for present frame number.
	uint32_t lastFrame = anim.lastFramePosition_[elementIdx].z;
	uint32_t frame;
	if (instance.tickRange_.x > instance.tickRange_.y) {
		frame = (timeInTicks <= instance.lastTime_ ? lastFrame : anim.startFramePosition_[elementIdx].z);
		findFrameBeforeTick(timeInTicks, frame, keys);
	} else {
		frame = (timeInTicks >= instance.lastTime_ ? lastFrame : anim.startFramePosition_[elementIdx].z);
		findFrameAfterTick(timeInTicks, frame, keys);
	}
	anim.lastFramePosition_[elementIdx].z = frame;

	// lookup nearest key
	const KeyFrame3f &key = keys[frame];
	// set current value
	if (!handleFrameLoop(scale, frame, lastFrame, channel, key, keys[0])) {
		scale.value = key.value;
	}

	return scale.value;
}

ref_ptr<NodeAnimation::Node> NodeAnimation::findNode(const std::string &name) {
	return findNode(rootNode_, name);
}

ref_ptr<NodeAnimation::Node> NodeAnimation::findNode(ref_ptr<Node> &n, const std::string &name) {
	if (n->name == name) { return n; }
	for (auto & it : n->children) {
		ref_ptr<Node> n_ = findNode(it, name);
		if (n_.get()) { return n_; }
	}
	return {};
}

NodeAnimation::Node::Node(const std::string &name, const ref_ptr<Node> &parent)
		: name(name),
		  parent(parent),
		  localTransform(Mat4f::identity()),
		  globalTransform(Mat4f::identity()),
		  offsetMatrix(Mat4f::identity()),
		  channelIndex(-1),
		  isBoneNode(false) {
	boneTransformationMatrix.resize(1, Mat4f::identity());
}

void NodeAnimation::Node::addChild(const ref_ptr<Node> &child) {
	children.push_back(child);
}

void NodeAnimation::Node::calculateGlobalTransform() {
	// concatenate all parent transforms to get the global transform for this node
	globalTransform = localTransform;
	for (Node *p = parent.get(); p != nullptr; p = p->parent.get()) {
		globalTransform.multiplyr(p->localTransform);
	}
}

void NodeAnimation::Node::updateTransforms(uint32_t instanceIdx, Mat4f *transforms) {
#ifndef LOCAL_ROOT_IS_IDENTITY
	globalTransform = channelIndex!=-1 ? transforms[channelIndex] : localTransform;
	Mat4f rootInverse = globalTransform.inverse();
#endif
	if (isBoneNode) {
		boneTransformationMatrix[instanceIdx] = offsetMatrix;
	}

	for (auto &child : children) {
		traversalStack.push(child.get());
	}
	while (!traversalStack.isEmpty()) {
		Node *n = traversalStack.top();
		traversalStack.pop();
		// update node global transform
		n->globalTransform = (n->channelIndex != -1) ? transforms[n->channelIndex] : n->localTransform;
		n->globalTransform.multiplyr(n->parent->globalTransform);

		if (n->isBoneNode) {
			// Bone matrices transform from mesh coordinates in bind pose
			// to mesh coordinates in skinned pose
#ifdef LOCAL_ROOT_IS_IDENTITY
			n->boneTransformationMatrix[instanceIdx] = (n->globalTransform * n->offsetMatrix).transpose();
#else
			n->boneTransformationMatrix = (rootInverse*n->globalTransform*n->offsetMatrix).transpose();
#endif
		}

		// continue for all children
		for (auto &child : n->children) {
			traversalStack.push(child.get());
		}
	}
}

namespace regen {
	std::ostream &operator<<(std::ostream &out, const NodeAnimation::Behavior &mode) {
		switch (mode) {
			case NodeAnimation::BEHAVIOR_DEFAULT:
				return out << "DEFAULT";
			case NodeAnimation::BEHAVIOR_CONSTANT:
				return out << "CONSTANT";
			case NodeAnimation::BEHAVIOR_LINEAR:
				return out << "LINEAR";
			case NodeAnimation::BEHAVIOR_REPEAT:
				return out << "REPEAT";
		}
		return out;
	}

	std::istream &operator>>(std::istream &in, NodeAnimation::Behavior &mode) {
		std::string val;
		in >> val;
		boost::to_upper(val);
		if (val == "DEFAULT") mode = NodeAnimation::BEHAVIOR_DEFAULT;
		else if (val == "CONSTANT") mode = NodeAnimation::BEHAVIOR_CONSTANT;
		else if (val == "LINEAR") mode = NodeAnimation::BEHAVIOR_LINEAR;
		else if (val == "REPEAT") mode = NodeAnimation::BEHAVIOR_REPEAT;
		else {
			REGEN_WARN("Unknown animation behavior '" << val << "'. Using default DEFAULT behavior.");
			mode = NodeAnimation::BEHAVIOR_DEFAULT;
		}
		return in;
	}
}
