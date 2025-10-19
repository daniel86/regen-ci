#ifndef REGEN_BEHAVIOR_TREE_H_
#define REGEN_BEHAVIOR_TREE_H_
#include <functional>
#include <memory>
#include <vector>
#include "blackboard.h"

namespace regen {
	/**
	 * The status of a behavior tree node after being ticked.
	 */
	enum class BehaviorStatus {
		SUCCESS,
		FAILURE,
		RUNNING
	};

	/** A function type for behavior tree tick functions. */
	using BehaviorTickFunction = std::function<BehaviorStatus(Blackboard& bb)>;

	/**
	 * A behavior tree implementation.
	 * Nodes can be sequences, selectors, or priority nodes.
	 * Conditions can be used to evaluate the blackboard state.
	 */
	class BehaviorTree {
	public:
		/**
		 * A node in the behavior tree.
		 */
		class Node {
		public:
			std::vector<std::unique_ptr<Node>> children;
			virtual ~Node() = default;
			/**
			 * Add a child node.
			 * @param child the child node.
			 */
			void addChild(std::unique_ptr<Node> child) {
				children.push_back(std::move(child));
			}
			/**
			 * Tick the node with the given blackboard.
			 * @param bb the blackboard.
			 * @return the behavior status.
			 */
			virtual BehaviorStatus tick(Blackboard& bb, float dt_s) = 0;
		};
		/**
		 * A condition that can be evaluated against a blackboard.
		 */
		class Condition {
			bool negated = false;
			bool exclusive = false;
			BehaviorStatus failureStatus_ = BehaviorStatus::FAILURE;
		public:
			virtual ~Condition() = default;

			/**
			 * Set whether to negate the condition.
			 * @param n true to negate, false otherwise.
			 */
			void setNegated(bool n) { negated = n; }

			/**
			 * Check if the condition is negated.
			 * @return true if negated, false otherwise.
			 */
			bool isNegated() const { return negated; }

			/**
			 * Set whether the condition excludes others on the same level if being true,
			 * i.e. conditions within a common select node.
			 * @param n true to exclusive, false otherwise.
			 */
			void setExclusive(bool n) { exclusive = n; }

			/**
			 * Check if the condition is exclusive.
			 * @return true if exclusive, false otherwise.
			 */
			bool isExclusive() const { return exclusive; }

			/**
			 * Set the failure status to return if the condition is not met.
			 * @param status the failure status.
			 */
			void setFailureStatus(BehaviorStatus status) { failureStatus_ = status; }

			/**
			 * Get the failure status.
			 * @return the failure status.
			 */
			BehaviorStatus failureStatus() const { return failureStatus_; }

			/**
			 * Evaluate the condition against the given blackboard.
			 * @param bb the blackboard.
			 * @return true if the condition is met, false otherwise.
			 */
			bool evaluate(const Blackboard& bb) const {
				return negated ? !doEvaluate(bb) : doEvaluate(bb);
			}

		protected:
			virtual bool doEvaluate(const Blackboard& bb) const = 0;
		};

		explicit BehaviorTree(std::unique_ptr<Node> root) : root_(std::move(root)) {};

		//BehaviorTree(const BehaviorTree &other) : root_(copyNode(other.root_)) {};
		BehaviorTree(const BehaviorTree &other) = delete;

		~BehaviorTree() = default;

		/**
		 * Tick the behavior tree with the given blackboard.
		 * @param bb the blackboard.
		 * @return the behavior status.
		 */
		BehaviorStatus tick(Blackboard& bb, float dt_s) {
			return root_->tick(bb, dt_s);
		}

		/**
		 * Load a behavior tree node from a scene input node.
		 * @param ctx the loading context.
		 * @param input the scene input node.
		 * @return the loaded behavior tree node.
		 */
		static std::unique_ptr<Node> load(LoadingContext &ctx, scene::SceneInputNode &xmlNode);
	protected:
		std::unique_ptr<Node> root_;
	};

	/**
	 * A sequence node that ticks its children in order.
	 * If a child returns RUNNING, the sequence returns RUNNING.
	 * If a child returns FAILURE, the sequence returns FAILURE.
	 * If all children return SUCCESS, the sequence returns SUCCESS.
	 */
	class BehaviorSequenceNode : public BehaviorTree::Node {
		size_t current = 0;
	public:
		// Tick children in order until one returns RUNNING or FAILURE.
		BehaviorStatus tick(Blackboard& bb, float dt_s) override {
			while (current < children.size()) {
				BehaviorStatus s = children[current]->tick(bb, dt_s);
				if (s == BehaviorStatus::RUNNING) return BehaviorStatus::RUNNING;
				if (s == BehaviorStatus::FAILURE) { current = 0; return BehaviorStatus::FAILURE; }
				++current;
			}
			current = 0;
			return BehaviorStatus::SUCCESS;
		}
	};

	/**
	 * A selector node that ticks its children in order.
	 * If a child returns RUNNING, the selector returns RUNNING.
	 * If a child returns SUCCESS, the selector returns SUCCESS.
	 * If all children return FAILURE, the selector returns FAILURE.
	 */
	class BehaviorSelectorNode : public BehaviorTree::Node {
		size_t current = 0;
	public:
		// Tick children in order until one returns RUNNING or SUCCESS.
		BehaviorStatus tick(Blackboard& bb, float dt_s) override {
			while (current < children.size()) {
				BehaviorStatus s = children[current]->tick(bb, dt_s);
				if (s == BehaviorStatus::RUNNING) return BehaviorStatus::RUNNING;
				if (s == BehaviorStatus::SUCCESS) { current = 0; return BehaviorStatus::SUCCESS; }
				++current;
			}
			current = 0;
			return BehaviorStatus::FAILURE;
		}
	};

	/**
	 * A priority node that ticks its children in order.
	 * If a child returns SUCCESS or RUNNING, the priority node returns that status.
	 * If all children return FAILURE, the priority node returns FAILURE.
	 */
	class BehaviorPriorityNode : public BehaviorTree::Node {
	public:
		// Tick children in order until one returns SUCCESS or RUNNING.
		BehaviorStatus tick(Blackboard& bb, float dt_s) override {
			for (const auto &child : children) {
				BehaviorStatus s = child->tick(bb, dt_s);
				if (s != BehaviorStatus::FAILURE) return s;
			}
			return BehaviorStatus::FAILURE;
		}
	};

	/**
	 * A parallel node that ticks all its children.
	 * The node returns SUCCESS or FAILURE based on the configured modes.
	 */
	class BehaviorParallelNode : public BehaviorTree::Node {
	public:
		enum StatusMode {
			// succeed/fail if ANY children succeed/fail
			ANY = 1,
			// succeed/fail if ALL children succeed/fail
			ALL
		};
		explicit BehaviorParallelNode(StatusMode successMode, StatusMode failureMode)
			: BehaviorTree::Node(), successMode_(successMode), failureMode_(failureMode) {
		}
		// Tick all children, return SUCCESS if at least successThreshold children succeed,
		// RUNNING if any child is running, else FAILURE.
		BehaviorStatus tick(Blackboard& bb, float dt_s) override {
			size_t successCount = 0;
			size_t failureCount = 0;
			for (const auto &child : children) {
				BehaviorStatus s = child->tick(bb, dt_s);
				if (s == BehaviorStatus::SUCCESS) {
					++successCount;
					if (successMode_ == ANY) return BehaviorStatus::SUCCESS;
				} else if (s == BehaviorStatus::FAILURE) {
					++failureCount;
					if (failureMode_ == ANY) return BehaviorStatus::FAILURE;
				}
			}
			if (successMode_ == ALL && successCount == children.size()) {
				return BehaviorStatus::SUCCESS;
			}
			if (failureMode_ == ALL && failureCount == children.size()) {
				return BehaviorStatus::FAILURE;
			}
			return BehaviorStatus::RUNNING;
		}
	protected:
		const StatusMode successMode_;
		const StatusMode failureMode_;
	};

	std::ostream &operator<<(std::ostream &out, const BehaviorParallelNode::StatusMode &v);

	std::istream &operator>>(std::istream &in, BehaviorParallelNode::StatusMode &v);

	std::ostream &operator<<(std::ostream &out, const BehaviorStatus &v);

	std::istream &operator>>(std::istream &in, BehaviorStatus &v);
} // namespace

#endif /* REGEN_BEHAVIOR_TREE_H_ */
