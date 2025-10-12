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
			virtual BehaviorStatus tick(Blackboard& bb) = 0;
		};
		/**
		 * A condition that can be evaluated against a blackboard.
		 */
		class Condition {
			bool negated = false;
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
		BehaviorStatus tick(Blackboard& bb) {
			return root_->tick(bb);
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
		BehaviorStatus tick(Blackboard& bb) override {
			while (current < children.size()) {
				BehaviorStatus s = children[current]->tick(bb);
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
		BehaviorStatus tick(Blackboard& bb) override {
			while (current < children.size()) {
				BehaviorStatus s = children[current]->tick(bb);
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
		BehaviorStatus tick(Blackboard& bb) override {
			for (const auto &child : children) {
				BehaviorStatus s = child->tick(bb);
				if (s != BehaviorStatus::FAILURE) return s;
			}
			return BehaviorStatus::FAILURE;
		}
	};
} // namespace

#endif /* REGEN_BEHAVIOR_TREE_H_ */
