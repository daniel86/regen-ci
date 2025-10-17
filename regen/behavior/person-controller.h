#ifndef REGEN_PERSON_CONTROLLER_H_
#define REGEN_PERSON_CONTROLLER_H_

#include <regen/math/vector.h>
#include <regen/behavior/npc-controller.h>
#include "world/world-model.h"
#include <regen/behavior/path-planner.h>

#include "behavior-tree.h"
#include "blackboard.h"
#include "perception/perception-system.h"
#include "regen/states/model-transformation.h"
#include "regen/objects/terrain/blanket-trail.h"
#include "skeleton/bone-controller.h"

namespace regen {
	class PersonController : public NonPlayerCharacterController {
	public:
		PersonController(
			const ref_ptr<Mesh> &mesh,
			const Indexed<ref_ptr<ModelTransformation>> &tfIndexed,
			const ref_ptr<BoneAnimationItem> &animItem,
			const ref_ptr<WorldModel> &world);

		~PersonController() override;

		/**
		 * Set the decision interval for the NPC.
		 * @param interval_s the decision interval in seconds.
		 */
		void setDecisionInterval(float interval_s) { decisionInterval_s_ = interval_s; }

		/**
		 * Set the perception interval for the NPC.
		 * @param interval_s the perception interval in seconds.
		 */
		void setPerceptionInterval(float interval_s) { perceptionInterval_s_ = interval_s; }

		/**
		 * Get the knowledge base of the NPC.
		 * @return the knowledge base.
		 */
		Blackboard& knowledgeBase() { return knowledgeBase_; }

		/**
		 * Set the perception system for the NPC.
		 * @param ps the perception system.
		 */
		void setPerceptionSystem(std::unique_ptr<PerceptionSystem> ps);

		/**
		 * Set the behavior tree for the NPC.
		 * @param rootNode the behavior tree.
		 */
		void setBehaviorTree(std::unique_ptr<BehaviorTree::Node> rootNode);

		/**
		 * @param mesh A mesh that is only seen when the NPC is holding a weapon.
		 */
		void setWeaponMesh(const ref_ptr<Mesh> &mesh);

		/**
		 * Set the footstep trail for visualizing footsteps.
		 * @param trail the footstep trail.
		 */
		void setFootstepTrail(
					const ref_ptr<BlanketTrail> &trail,
					float leftFootTime, float rightFootTime) {
			footstepTrail_ = trail;
			footTime_[0] = leftFootTime;
			footTime_[1] = rightFootTime;
		}

		/**
		 * Add a waypoint to the path planner.
		 * @param wp the waypoint.
		 */
		void addWayPoint(const ref_ptr<WayPoint> &wp);

		/**
		 * Add a connection between two waypoints to the path planner.
		 * @param from the from waypoint.
		 * @param to the to waypoint.
		 * @param bidirectional if true, add a connection in both directions.
		 */
		void addConnection(
				const ref_ptr<WayPoint> &from,
				const ref_ptr<WayPoint> &to,
				bool bidirectional = true);

		// override
		void initializeController() override;

		// override
		void updateController(double dt) override;

	protected:
		Blackboard knowledgeBase_;
		std::unique_ptr<BehaviorTree> behaviorTree_;
		std::unique_ptr<PerceptionSystem> perceptionSystem_;

		float decisionInterval_s_ = 0.25f;
		float perceptionInterval_s_ = 0.1f;

		float timeSinceLastDecision_s_ = 100.0f;
		float timeSinceLastPerception_s_ = 100.0f;

		ref_ptr<Mesh> weaponMesh_;
		ref_ptr<BoundingShape> weaponShape_;
		bool hasDrawnWeapon_ = false;
		uint32_t weaponMask_ = 0;

		ref_ptr<BlanketTrail> footstepTrail_;
		uint32_t footIdx_[2] = { 0, 1 };
		bool footDown_[2] = { false, false };
		float footTime_[2] = { 0.2f, 0.8f };
		float footLastElapsed_ = 0.0f;

		ref_ptr<PathPlanner> pathPlanner_;
		// Flag is used for continuous movement.
		bool isLastAnimationMovement_ = false;

		ref_ptr<BoneController> boneController_;

		void updateKnowledgeBase(float dt_s);

		bool startNavigate(bool loopPath, bool advancePath);

		void updateNavigationBehavior();

		Vec2f pickTargetPosition(const Patient &navTarget) const;

		Vec2f pickTravelPosition(const WorldObject &wp) const;

		bool updatePathNPC();

		void setIdle();

		void hideWeapon();

		void drawWeapon();

		uint32_t findClosestWP(const std::vector<ref_ptr<WayPoint>> &wps) const;
	};
} // namespace

#endif /* REGEN_PERSON_CONTROLLER_H_ */
