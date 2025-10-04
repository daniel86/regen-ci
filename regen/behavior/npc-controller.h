#ifndef REGEN_NPC_CONTROLLER_H_
#define REGEN_NPC_CONTROLLER_H_

#include <regen/math/vector.h>
#include <regen/behavior/animation-controller.h>
#include "world/world-model.h"
#include <regen/behavior/path-planner.h>

#include "blackboard.h"
#include "../animation/animation-node.h"
#include "motion/motion-type.h"
#include "regen/states/model-transformation.h"
#include "regen/objects/terrain/blanket-trail.h"
#include "world/action-type.h"

namespace regen {
	class NPCWorldObject : public WorldObject {
	public:
		NPCWorldObject(std::string_view name, const Vec3f &position) :
			WorldObject(name, position) {}
	};

	class NPCController : public AnimationController {
	public:
		/**
		 * The state of the NPC.
		 */
		enum State {
			STATE_IDLE = 0,
			// Try to reach home place as fast as possible.
			STATE_GO_TO_HOME,
			// Go to a place to socialize with other NPCs.
			STATE_GO_TO_MARKET,
			// Go to a place to pray.
			// But if not spiritual the NPC would not take long detours.
			STATE_GO_TO_SHRINE,
			// Alert, e.g. because of a noise.
			// Look around, try to find the threat.
			STATE_ON_ALERT,
			// Run away from a threat.
			STATE_RUN_AWAY,
			// Fight against a threat.
			STATE_FIGHTING,
			STATE_AT_HOME,
			STATE_AT_MARKET,
			STATE_AT_SHRINE,
			STATE_LAST
		};

		NPCController(
			const ref_ptr<Mesh> &mesh,
			const Indexed<ref_ptr<ModelTransformation>> &tfIndexed,
			const ref_ptr<NodeAnimationItem> &animItem,
			const ref_ptr<WorldModel> &world);

		/**
		 * Get the knowledge base of the NPC.
		 * @return the knowledge base.
		 */
		Blackboard& knowledgeBase() { return knowledgeBase_; }

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

		ref_ptr<NPCWorldObject> npcWorldObject_;
		std::vector<std::pair<ActionType,float>> actionPossibilities_;

		ref_ptr<Mesh> weaponMesh_;
		ref_ptr<BoundingShape> weaponShape_;
		bool hasDrawnWeapon_ = false;
		uint32_t weaponMask_ = 0;

		ref_ptr<BlanketTrail> footstepTrail_;
		uint32_t footIdx_[2] = { 0, 1 };
		bool footDown_[2] = { false, false };
		float footTime_[2] = { 0.2f, 0.8f };

		ref_ptr<PathPlanner> pathPlanner_;
		// All semantic movements the NPC can perform.
		std::map<MotionType, std::vector<const AnimRange*>> motionRanges_;
		// The current movement behavior of the NPC.
		MotionType currentMotion_ = MotionType::IDLE;
		// Flag is used for continuous movement.
		bool isLastAnimationMovement_ = false;
		// Walking behavior parameters.
		float walkTime_ = 0.0f;
		float runTime_ = 0.0f;

		// The current state of the NPC.
		State currentState_ = STATE_IDLE;
		// The current activity of the NPC.
		ActionType currentActivity_ = ActionType::IDLE;
		int nextActivity_ = -1;

		bool startNavigate(bool loopPath, bool advancePath);

		void startMotion();

		void setIdle();

		void setAtHome();

		void setAtMarket();

		void setAtShrine();

		void setPlaceActivity(const ref_ptr<Place> &place);

		void updatePlaceActivity(const ref_ptr<Place> &place);

		void updateActionPossibilities(const ref_ptr<Place> &place);

		void updateBehavior(double dt);

		Vec2f pickTargetPosition(const WorldObject &wp);

		Vec2f pickTravelPosition(const WorldObject &wp) const;

		uint32_t findClosestWP_idx(const std::vector<ref_ptr<WayPoint>> &wps) const;

		bool setPatient(const ref_ptr<WorldObject> &obj, AffordanceType type);

		void unsetPatient();

		void setActivityTime(ActionType activity);

		bool updatePathNPC();

		void hideWeapon();

		void drawWeapon();

		bool isDoneWithActivity() const;

		bool isDoneWithPlace() const;

		void initializeBoneAnimations();
	};
} // namespace

#endif /* REGEN_NPC_CONTROLLER_H_ */
