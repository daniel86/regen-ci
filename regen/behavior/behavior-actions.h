#ifndef REGEN_BEHAVIOR_ACTIONS_H_
#define REGEN_BEHAVIOR_ACTIONS_H_
#include "behavior-tree.h"
#include "world/action-type.h"
#include "world/place.h"

namespace regen {
	/**
	 * A base class for action nodes in a behavior tree.
	 */
	class BehaviorActionNode : public BehaviorTree::Node {
	public:
		BehaviorActionNode(): BehaviorTree::Node() {}

		~BehaviorActionNode() override = default;
	};

	/**
	 * An action node that executes a lambda function.
	 */
	class BehaviorLambdaNode : public BehaviorActionNode {
		BehaviorTickFunction func;
	public:
		explicit BehaviorLambdaNode(BehaviorTickFunction f)
			: BehaviorActionNode(), func(std::move(f)) {}

		~BehaviorLambdaNode() override = default;

		BehaviorStatus tick(Blackboard& bb) override {
			return func(bb);
		}
	};

	/**
	 * An action node that selects a target place of a desired type,
	 * or one based on the NPC's traits if no type is specified.
	 */
	class SelectTargetPlace : public BehaviorActionNode {
		PlaceType desiredPlaceType = PlaceType::LAST;
	public:
		SelectTargetPlace() : BehaviorActionNode() {}

		explicit SelectTargetPlace(PlaceType pt)
			: BehaviorActionNode(), desiredPlaceType(pt) {}

		~SelectTargetPlace() override = default;

		void setDesiredPlaceType(PlaceType pt) { desiredPlaceType = pt; }

		BehaviorStatus tick(Blackboard& bb) override;
	};

	/**
	 * An action node that sets the target place to a specific place.
	 */
	class SetTargetPlace : public BehaviorActionNode {
		ref_ptr<Place> targetPlace;
	public:
		explicit SetTargetPlace(const ref_ptr<Place> &place)
			: BehaviorActionNode(), targetPlace(place) {}

		~SetTargetPlace() override = default;

		BehaviorStatus tick(Blackboard& bb) override;
	};

	/**
	 * An action node that selects an activity based on the NPC's traits
	 * and the possibilities afforded by the current place.
	 */
	class SelectPlaceActivity : public BehaviorActionNode {
		ref_ptr<Place> lastPlace_;
		std::vector<std::pair<ActionType, float>> actionPossibilities_;
	public:
		SelectPlaceActivity() : BehaviorActionNode() {}

		~SelectPlaceActivity() override = default;

		BehaviorStatus tick(Blackboard& bb) override;

	protected:
		void updateActionPossibilities(Blackboard& kb);
	};

	/**
	 * An action node that sets a desired activity.
	 */
	class SetDesiredActivity : public BehaviorActionNode {
		const ActionType desiredAction;
	public:
		explicit SetDesiredActivity(ActionType a)
			: BehaviorActionNode(), desiredAction(a) {}

		~SetDesiredActivity() override = default;

		BehaviorStatus tick(Blackboard& bb) override;
	};

	/**
	 * An action node that selects a patient (object) for the desired activity
	 * at the current place.
	 */
	class SelectPlacePatient : public BehaviorActionNode {
		ActionType lastDesiredAction_ = ActionType::IDLE;
		ref_ptr<Place> lastPlace_;
	public:
		SelectPlacePatient() : BehaviorActionNode() {}

		~SelectPlacePatient() override = default;

		BehaviorStatus tick(Blackboard& bb) override;
	};

	/**
	 * An action node that sets a specific patient (object) for the desired activity.
	 */
	class SetPatient : public BehaviorActionNode {
		ref_ptr<WorldObject> patient = {};
	public:
		explicit SetPatient(const ref_ptr<WorldObject> &obj) : BehaviorActionNode(), patient(obj) {}

		~SetPatient() override = default;

		BehaviorStatus tick(Blackboard& bb) override;
	};

	/**
	 * An action node that unsets the current patient (object).
	 */
	class UnsetPatient : public BehaviorActionNode {
	public:
		UnsetPatient() : BehaviorActionNode() {}

		~UnsetPatient() override = default;

		BehaviorStatus tick(Blackboard& bb) override;
	};

	/**
	 * An action node that moves the NPC to its target place.
	 */
	class MoveToTargetPlace : public BehaviorActionNode {
	public:
		MoveToTargetPlace() : BehaviorActionNode() {}

		~MoveToTargetPlace() override = default;

		BehaviorStatus tick(Blackboard& bb) override;
	};

	/**
	 * An action node that moves the NPC to its patient (object).
	 */
	class MoveToPatient : public BehaviorActionNode {
	public:
		MoveToPatient() : BehaviorActionNode() {}

		~MoveToPatient() override = default;

		BehaviorStatus tick(Blackboard& bb) override;
	};

	/**
	 * An action node that makes the NPC perform its desired action.
	 */
	class PerformDesiredAction : public BehaviorActionNode {
	public:
		PerformDesiredAction() : BehaviorActionNode() {}

		~PerformDesiredAction() override = default;

		BehaviorStatus tick(Blackboard& bb) override;
	};

	/**
	 * An action node that makes the NPC perform the afforded action
	 * with its current patient (object).
	 * This will report failure in case the object is out of reach.
	 */
	class PerformAffordedAction : public BehaviorActionNode {
	public:
		PerformAffordedAction() : BehaviorActionNode() {}

		~PerformAffordedAction() override = default;

		BehaviorStatus tick(Blackboard& bb) override;
	};

} // namespace

#endif /* REGEN_BEHAVIOR_ACTIONS_H_ */
