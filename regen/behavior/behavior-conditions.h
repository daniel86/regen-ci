#ifndef REGEN_BEHAVIOR_CONDITIONS_H_
#define REGEN_BEHAVIOR_CONDITIONS_H_

#include <functional>
#include <memory>
#include "behavior-tree.h"
#include "blackboard.h"

namespace regen {
	/**
	 * A condition node that can negate the condition.
	 * If the condition evaluates to true (or false if negated), the child node is ticked.
	 * Otherwise, FAILURE is returned.
	 */
	class BehaviorConditionNode : public BehaviorTree::Node {
		std::unique_ptr<BehaviorTree::Condition> cond;
		std::unique_ptr<Node> child;
	public:
		/**
		 * Create a condition node with a condition and a child node.
		 * @param c the condition.
		 * @param ch the child node.
		 */
		BehaviorConditionNode(std::unique_ptr<BehaviorTree::Condition> c, std::unique_ptr<Node> ch)
			: cond(std::move(c)), child(std::move(ch)) {}

		~BehaviorConditionNode() override = default;

		// If condition is true (or false if negated), tick the child node.
		// Otherwise, return FAILURE.
		BehaviorStatus tick(Blackboard& bb, float dt_s) override {
			if (cond->evaluate(bb)) {
				auto childStatus = child->tick(bb, dt_s);
				// In case of "exclusive" conditions, we want to prevent that neighbor branches
				// are activated in case the subtree of the condition fails which would
				// happen in case of forwarding the FAILURE status.
				if (childStatus == BehaviorStatus::FAILURE && cond->isExclusive()) {
					return BehaviorStatus::SUCCESS;
				} else {
					return childStatus;
				}
			}
			return cond->failureStatus();
		}
	};

	/**
	 * A condition that evaluates to true if all child conditions evaluate to true.
	 */
	class BehaviorConditionSequence : public BehaviorTree::Condition {
		std::vector<std::unique_ptr<BehaviorTree::Condition>> conditions;
	public:
		~BehaviorConditionSequence() override = default;
		/**
		 * Add a child condition.
		 * @param c the child condition.
		 */
		void addCondition(std::unique_ptr<BehaviorTree::Condition> c) {
			conditions.push_back(std::move(c));
		}
	protected:
		// Evaluate all child conditions, return true if all are true.
		bool doEvaluate(const Blackboard& bb) const override {
			for (const auto &c : conditions) {
				if (!c->evaluate(bb)) {
					return false;
				}
			}
			return true;
		}
	};

	/**
	 * A condition that evaluates to true if any child condition evaluates to true.
	 */
	class BehaviorConditionSelection : public BehaviorTree::Condition {
		std::vector<std::unique_ptr<BehaviorTree::Condition>> conditions;
	public:
		~BehaviorConditionSelection() override = default;
		/**
		 * Add a child condition.
		 * @param c the child condition.
		 */
		void addCondition(std::unique_ptr<BehaviorTree::Condition> c) {
			conditions.push_back(std::move(c));
		}
	protected:
		// Evaluate all child conditions, return true if any is true.
		bool doEvaluate(const Blackboard& bb) const override {
			for (const auto &c : conditions) {
				if (c->evaluate(bb)) {
					return true;
				}
			}
			return false;
		}
	};

	/**
	 * A condition that evaluates to true if the NPC is at its target place.
	 */
	class IsAtTargetPlace : public BehaviorTree::Condition {
	public:
		constexpr IsAtTargetPlace() noexcept = default;
		~IsAtTargetPlace() override = default;
	protected:
		// Evaluate to true if current place is the same as target place.
		bool doEvaluate(const Blackboard& bb) const override {
			return bb.currentPlace().get() == bb.targetPlace().get();
		}
	};

	/**
	 * A condition over the character's attributes.
	 */
	class BehaviorAttributeCondition : public BehaviorTree::Condition {
	public:
		constexpr explicit BehaviorAttributeCondition(CharacterAttribute a) noexcept : attr(a) {}
		~BehaviorAttributeCondition() override = default;
	protected:
		const CharacterAttribute attr;
	};

	/**
	 * A condition that checks if a character attribute has a specific value.
	 */
	class HasAttributeValue : public BehaviorAttributeCondition {
		const float value;
	public:
		constexpr explicit HasAttributeValue(CharacterAttribute a, float v) noexcept
			: BehaviorAttributeCondition(a), value(v) {}
		~HasAttributeValue() override = default;
	protected:
		// Evaluate to true if the attribute equals the specified value.
		bool doEvaluate(const Blackboard& bb) const override {
			return std::fabs(bb.getAttribute(attr) - value) < 1e-3f;
		}
	};

	/**
	 * A condition that checks if a character attribute is above a certain value.
	 */
	class IsAttributeAbove : public BehaviorAttributeCondition {
		float minValue;
	public:
		constexpr explicit IsAttributeAbove(CharacterAttribute a, float v) noexcept
			: BehaviorAttributeCondition(a), minValue(v) {}
		~IsAttributeAbove() override = default;
	protected:
		// Evaluate to true if the attribute is above the specified minimum value.
		bool doEvaluate(const Blackboard& bb) const override {
			return bb.getAttribute(attr) > minValue;
		}
	};

	/**
	 * A condition that checks if a character attribute is below a certain value.
	 */
	class IsAttributeBelow : public BehaviorAttributeCondition {
		float maxValue;
	public:
		constexpr explicit IsAttributeBelow(CharacterAttribute a, float v) noexcept
			: BehaviorAttributeCondition(a), maxValue(v) {}
		~IsAttributeBelow() override = default;
	protected:
		// Evaluate to true if the attribute is below the specified maximum value.
		bool doEvaluate(const Blackboard& bb) const override {
			return bb.getAttribute(attr) < maxValue;
		}
	};

	/**
	 * A condition over the character's personality traits.
	 */
	class BehaviorTraitCondition : public BehaviorTree::Condition {
	public:
		constexpr explicit BehaviorTraitCondition(Trait t) noexcept : trait(t) {}
		~BehaviorTraitCondition() override = default;
	protected:
		const Trait trait;
	};

	/**
	 * A condition that checks if a personality trait has a specific value.
	 */
	class HasTraitValue : public BehaviorTraitCondition {
		const float value;
	public:
		constexpr explicit HasTraitValue(Trait t, float v) noexcept
			: BehaviorTraitCondition(t), value(v) {}
		~HasTraitValue() override = default;
	protected:
		// Evaluate to true if the trait equals the specified value.
		bool doEvaluate(const Blackboard& bb) const override {
			return std::fabs(bb.traitStrength(trait) - value) < 1e-3f;
		}
	};

	/**
	 * A condition that checks if a personality trait is above a certain value.
	 */
	class IsTraitAbove : public BehaviorTraitCondition {
		float minValue;
	public:
		constexpr explicit IsTraitAbove(Trait t, float v) noexcept
			: BehaviorTraitCondition(t), minValue(v) {}
		~IsTraitAbove() override = default;
	protected:
		// Evaluate to true if the trait is above the specified minimum value.
		bool doEvaluate(const Blackboard& bb) const override {
			return bb.traitStrength(trait) > minValue;
		}
	};

	/**
	 * A condition that checks if a personality trait is below a certain value.
	 */
	class IsTraitBelow : public BehaviorTraitCondition {
		float maxValue;
	public:
		constexpr explicit IsTraitBelow(Trait t, float v) noexcept
			: BehaviorTraitCondition(t), maxValue(v) {}
		~IsTraitBelow() override = default;
	protected:
		// Evaluate to true if the trait is below the specified maximum value.
		bool doEvaluate(const Blackboard& bb) const override {
			return bb.traitStrength(trait) < maxValue;
		}
	};

	/**
	 * A condition that checks if the NPC has a desired place type.
	 */
	class HasDesiredPlaceType : public BehaviorTree::Condition {
		const PlaceType placeType;
	public:
		constexpr explicit HasDesiredPlaceType(PlaceType pt) noexcept : placeType(pt) {}
		~HasDesiredPlaceType() override = default;
	protected:
		// Evaluate to true if the target place has the specified type.
		bool doEvaluate(const Blackboard& bb) const override {
			auto place = bb.targetPlace();
			return (place.get() != nullptr && place->placeType() == placeType);
		}
	};

	/**
	 * A condition that checks if the NPC has a desired activity.
	 */
	class HasDesiredActivity : public BehaviorTree::Condition {
		const ActionType activity;
	public:
		constexpr explicit HasDesiredActivity(ActionType a) noexcept : activity(a) {}
		~HasDesiredActivity() override = default;
	protected:
		// Evaluate to true if the desired activity matches the specified activity.
		bool doEvaluate(const Blackboard& bb) const override {
			return bb.desiredAction() == activity;
		}
	};

	/**
	 * A condition that checks if the NPC has been initialized with a desire.
	 * This is useful to ensure that the NPC has a valid state before performing actions.
	 */
	class HasDesire : public BehaviorTree::Condition {
	public:
		constexpr HasDesire() noexcept = default;
		~HasDesire() override = default;
	protected:
		// Evaluate to true if the agent has some desire, else false.
		bool doEvaluate(const Blackboard& bb) const override {
			return bb.desiredAction() != ActionType::LAST_ACTION;
		}
	};

	/**
	 * A condition that checks if the NPC is at its desired location.
	 * This is useful to ensure that the NPC has reached the location
	 * associated with its current interaction target (patient).
	 */
	class IsAtDesiredLocation : public BehaviorTree::Condition {
	public:
		constexpr IsAtDesiredLocation() noexcept = default;
		~IsAtDesiredLocation() override = default;
	protected:
		// Evaluate to true if the agent is at the desired location.
		bool doEvaluate(const Blackboard& kb) const override {
			// TODO: better introduce desired location attribute?
			return kb.currentLocation().get() == kb.interactionTarget().object.get();
		}
	};

	/**
	 * A condition that checks if the NPC is the last member of its social group.
	 * This can be useful to trigger specific behaviors when the NPC is alone.
	 */
	class IsLastGroupMember : public BehaviorTree::Condition {
	public:
		constexpr IsLastGroupMember() noexcept = default;
		~IsLastGroupMember() override = default;
	protected:
		// Evaluate to true if the agent is the last member of its social group.
		bool doEvaluate(const Blackboard& kb) const override {
			auto group = kb.currentGroup();
			return (group.get() != nullptr && group->numMembers() == 1);
		}
	};

	/**
	 * A condition that checks if the NPC is part of a social group.
	 */
	class IsPartOfGroup : public BehaviorTree::Condition {
	public:
		constexpr IsPartOfGroup() noexcept = default;
		~IsPartOfGroup() override = default;
	protected:
		// Evaluate to true if the agent is part of a social group.
		bool doEvaluate(const Blackboard& kb) const override {
			return (kb.currentGroup().get() != nullptr); // && (kb.currentGroup()->numMembers() > 1);
		}
	};

	// TODO: Support more conditions
	//	- IsEnemyVisible, IsDangerVisible, IsPlayerVisible
	//  - HasFaction, IsEnemy, IsAlly, IsNeutral
	//  - IsMorning, IsAfternoon, IsEvening, IsNight, IsTimeBetween
	//  - IsSunny, IsRainy, IsSnowy, etc
	//  - IsSpring, IsSummer, IsFall, IsWinter
} // namespace

#endif /* REGEN_BEHAVIOR_CONDITIONS_H_ */
