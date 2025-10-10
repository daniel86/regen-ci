#include "behavior-tree.h"
#include "behavior-actions.h"
#include "behavior-conditions.h"
#include "regen/utility/xml.h"

using namespace regen;

static std::unique_ptr<BehaviorTree::Condition> makeCondition(scene::SceneInputNode *xmlCond) {
	const auto category = xmlCond->getValue("type");
	bool negate = xmlCond->getValue("negate", false);
	std::unique_ptr<BehaviorTree::Condition> cond;

	if (!xmlCond->hasAttribute("type")) {
		REGEN_WARN("Ignoring " << xmlCond->getDescription() << ", missing 'type' attribute.");
		return {};
	}

	if (category == "predicate") {
		auto predicateName = xmlCond->getName();
		if (predicateName == "IsAtTargetPlace") {
			cond = std::make_unique<IsAtTargetPlace>();
		}
		else if (predicateName == "HasDesiredPlaceType") {
			const PlaceType placeType = xmlCond->getValue<PlaceType>("value", PlaceType::HOME);
			cond = std::make_unique<HasDesiredPlaceType>(placeType);
		}
		else if (predicateName == "HasDesiredActivity") {
			const ActionType actionType = xmlCond->getValue<ActionType>("value", ActionType::IDLE);
			cond = std::make_unique<HasDesiredActivity>(actionType);
		}
		// TODO: support more conditions
		/**
			<condition type="visible" object-type="ENEMY"/>
			<condition type="visible" object-type="DANGER"/>
			else if (predicateName == "IsEnemyVisible") {
				conditions.push_back(std::make_unique<IsEnemyVisible>());
			}
			else if (predicateName == "IsDangerVisible") {
				conditions.push_back(std::make_unique<IsDangerVisible>());
			}
			else if (predicateName == "IsPlayerVisible") {
				conditions.push_back(std::make_unique<IsPlayerVisible>());
			}
		**/
		else {
			REGEN_WARN("Ignoring " << xmlCond->getDescription() << ", unknown predicate '" << predicateName << "'.");
			return {};
		}
	} else if (category == "attribute") {
		CharacterAttribute attr = xmlCond->getValue<CharacterAttribute>(
			"name", CharacterAttribute::HEALTH);
		if (xmlCond->hasAttribute("value")) {
			float v = xmlCond->getValue<float>("value", 0.0f);
			cond = std::make_unique<HasAttributeValue>(attr, v);
		}
		else if (xmlCond->hasAttribute("min-value")) {
			float v = xmlCond->getValue<float>("min-value", 0.0f);
			cond = std::make_unique<IsAttributeAbove>(attr, v);
		}
		else if (xmlCond->hasAttribute("max-value")) {
			float v = xmlCond->getValue<float>("max-value", 0.0f);
			cond = std::make_unique<IsAttributeBelow>(attr, v);
		} else {
			REGEN_WARN("Ignoring " << xmlCond->getDescription() << ", missing value, min-value or max-value.");
		}
	}
	else if (category == "trait") {
		Trait trait = xmlCond->getValue<Trait>("name", Trait::BRAVERY);
		if (xmlCond->hasAttribute("value")) {
			float v = xmlCond->getValue<float>("value", 0.0f);
			cond = std::make_unique<HasTraitValue>(trait, v);
		}
		else if (xmlCond->hasAttribute("min-value")) {
			float v = xmlCond->getValue<float>("min-value", 0.0f);
			cond = std::make_unique<IsTraitAbove>(trait, v);
		}
		else if (xmlCond->hasAttribute("max-value")) {
			float v = xmlCond->getValue<float>("max-value", 0.0f);
			cond = std::make_unique<IsTraitBelow>(trait, v);
		} else {
			REGEN_WARN("Ignoring " << xmlCond->getDescription() << ", missing value, min-value or max-value.");
		}
	} else if (category == "disjunction") {
		auto selection = std::make_unique<BehaviorConditionSelection>();
		for (auto &childNode : xmlCond->getChildren("condition")) {
			auto childCond = makeCondition(childNode.get());
			if (childCond) {
				selection->addCondition(std::move(childCond));
			} else {
				REGEN_WARN("Ignoring child condition in " << xmlCond->getDescription() << ".");
			}
		}
		cond = std::move(selection);
	} else {
		REGEN_WARN("Ignoring " << xmlCond->getDescription() << ", unknown condition type '" << category << "'.");
	}
	if (cond) {
		cond->setNegated(negate);
	}
	return cond;
}

static BehaviorTree::Node* addConditionDecorator(
	LoadingContext &ctx,
	scene::SceneInputNode &xmlNode,
	BehaviorTree::Node *node) {
	std::vector<std::unique_ptr<BehaviorTree::Condition>> conditions;

	auto conditionChildren = xmlNode.getChildren("condition");
	for (auto &xmlCond : conditionChildren) {
		auto cond = makeCondition(xmlCond.get());
		if (cond) {
			conditions.push_back(std::move(cond));
		} else {
			REGEN_WARN("Ignoring child condition in " << xmlNode.getDescription() << ".");
		}
		xmlNode.removeChild(xmlCond);
	}
	if (conditions.size() > 1) {
		BehaviorConditionSequence *seq = new BehaviorConditionSequence();
		for (auto &c : conditions) {
			seq->addCondition(std::move(c));
		}
		std::unique_ptr<BehaviorTree::Condition> seqPtr(seq);
		std::unique_ptr<BehaviorTree::Node> nodePtr(node);
		return new BehaviorConditionNode(std::move(seqPtr), std::move(nodePtr));
	} else if (conditions.size() == 1) {
		std::unique_ptr<BehaviorTree::Node> nodePtr(node);
		return new BehaviorConditionNode(std::move(conditions[0]), std::move(nodePtr));
	} else {
		return node;
	}
}

static BehaviorTree::Node* loadNode(LoadingContext &ctx, scene::SceneInputNode &xmlNode, BehaviorTree::Node *parent) {
	const std::string category = xmlNode.getCategory();
	BehaviorTree::Node *node = nullptr;

	if (category == "sequence") {
		auto seqNode = new BehaviorSequenceNode();
		node = addConditionDecorator(ctx, xmlNode, seqNode);
		for (auto &child : xmlNode.getChildren()) {
			if (!loadNode(ctx, *child.get(), seqNode)) {
				REGEN_WARN("Failed to load child node in '" << child->getDescription() << "'.");
			}
		}
	}
	else if (category == "selector") {
		if (xmlNode.getValue("type") == "priority") {
			auto priNode = new BehaviorPriorityNode();
			node = addConditionDecorator(ctx, xmlNode, priNode);
			for (auto &child : xmlNode.getChildren()) {
				if (!loadNode(ctx, *child.get(), priNode)) {
					REGEN_WARN("Failed to load child node in '" << child->getDescription() << "'.");
				}
			}
		} else {
			auto selNode = new BehaviorPriorityNode();
			node = addConditionDecorator(ctx, xmlNode, selNode);
			for (auto &child : xmlNode.getChildren()) {
				if (!loadNode(ctx, *child.get(), selNode)) {
					REGEN_WARN("Failed to load child node in '" << child->getDescription() << "'.");
				}
			}
		}
	}
	else if (category == "action") {
		auto actionType = xmlNode.getValue("type");
		if (actionType == "SelectTargetPlace") {
			auto placeType = xmlNode.getValue<PlaceType>("place-type", PlaceType::LAST);
			node = new SelectTargetPlace(placeType);
		}
		else if (actionType == "SelectPlaceActivity") {
			node = new SelectPlaceActivity();
		}
		else if (actionType == "SetDesiredActivity") {
			auto action = xmlNode.getValue<ActionType>("action", ActionType::IDLE);
			node = new SetDesiredActivity(action);
		}
		else if (actionType == "SelectPlacePatient") {
			node = new SelectPlacePatient();
		}
		else if (actionType == "UnsetPatient") {
			node = new UnsetPatient();
		}
		// TODO: SetTargetPlace and SetPatient would need a by-name lookup.
		/**
		else if (actionType == "SetTargetPlace") {
			auto placeName = xmlNode.getValue("place");
			auto place = parser->getResource<Place>(placeName);
			if (place.get() == nullptr) {
				REGEN_WARN("Ignoring " << xmlNode.getDescription() << ", unknown place '" << placeName << "'.");
			} else {
				node = new SetTargetPlace(place);
			}
		}
		else if (actionType == "SetPatient") {
			auto objName = xmlNode.getValue("object");
			auto obj = parser->getResource<WorldObject>(objName);
			if (obj.get() == nullptr) {
				REGEN_WARN("Ignoring " << xmlNode.getDescription() << ", unknown object '" << objName << "'.");
			} else {
				node = new SetPatient(obj);
			}
		}
		**/
		else if (actionType == "MoveToTargetPlace") {
			node = new MoveToTargetPlace();
		}
		else if (actionType == "MoveToPatient") {
			node = new MoveToPatient();
		}
		else if (actionType == "PerformDesiredAction") {
			node = new PerformDesiredAction();
		}
		else if (actionType == "PerformAffordedAction") {
			node = new PerformAffordedAction();
		}
		else {
			REGEN_WARN("Ignoring " << xmlNode.getDescription() << ", unknown action type '" << actionType << "'.");
		}
	}

	if (node == nullptr) {
		REGEN_WARN("Unknown behavior tree node type '" << category << "' in '" << xmlNode.getDescription() << "'.");
		return nullptr;
	}
	if (parent != nullptr) {
		parent->addChild(std::unique_ptr<BehaviorTree::Node>(node));
	}
	return node;
}

std::unique_ptr<BehaviorTree::Node> BehaviorTree::load(LoadingContext &ctx, scene::SceneInputNode &xmlNode) {
	BehaviorTree::Node *node = loadNode(ctx, xmlNode, nullptr);
	if (node == nullptr) {
		return {};
	} else {
		return std::unique_ptr<BehaviorTree::Node>(node);
	}
}
