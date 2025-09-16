#ifndef REGEN_NPC_CONTROLLER_H_
#define REGEN_NPC_CONTROLLER_H_

#include <regen/math/vector.h>
#include <regen/textures/texture.h>
#include <regen/animations/animation-controller.h>
#include <regen/animations/world-model.h>
#include <regen/animations/path-planner.h>
#include "animation-node.h"
#include "regen/shapes/bounds.h"
#include "regen/states/model-transformation.h"
#include "regen/math/bezier.h"
#include "regen/textures/height-map.h"

namespace regen {
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

		enum Activity {
			ACTIVITY_IDLE = 0,
			ACTIVITY_CONVERSING,
			ACTIVITY_STROLLING,
			ACTIVITY_OBSERVING,
			ACTIVITY_PATROLLING,
			ACTIVITY_PRAYING,
			ACTIVITY_SLEEPING,
			ACTIVITY_FIGHTING,
			ACTIVITY_FLEEING,
			ACTIVITY_TRAVELING,
			ACTIVITY_SITTING
		};

		/**
		 * The motion type of the NPC.
		 * There is a 1:1 mapping to animation ranges.
		 */
		enum Motion {
			// fast movement, usually to flee or chase
			MOTION_RUN = 0,
			// normal movement, usually to walk around
			MOTION_WALK,
			// no movement, just standing around
			MOTION_IDLE,
			MOTION_YES,
			MOTION_NO,
			// attacking, e.g. with a weapon
			MOTION_ATTACK,
			// blocking, e.g. with a shield
			MOTION_BLOCK,
			// crouching, e.g. to hide or sneak
			MOTION_CROUCH,
			// praying or crouching to meditate or to worship
			MOTION_PRAY,
			// standing up, e.g. after sleeping
			MOTION_STAND_UP,
			// sleeping, usually while lying down and being still
			MOTION_SLEEP,
			MOTION_LAST
		};

		/**
		 * Constructor.
		 * @param tf the model transformation.
		 * @param animation the node animation.
		 * @param ranges the animation ranges.
		 */
		NPCController(
			const ref_ptr<WorldModel> &world,
			const ref_ptr<ModelTransformation> &tf,
			uint32_t tfIdx,
			const ref_ptr<NodeAnimation> &animation,
			const std::vector<scene::AnimRange> &ranges);

		/**
		 * Set the spatial index for path finding.
		 * @param spatialIndex the spatial index.
		 */
		void setSpatialIndex(const ref_ptr<SpatialIndex> &spatialIndex, std::string_view shapeName);

		/**
		 * Set the world time.
		 * @param worldTime the world time.
		 */
		void setWorldTime(const WorldTime *worldTime) { worldTime_ = worldTime; }

		/**
		 * Set the walk speed.
		 * @param speed the speed.
		 */
		void setWalkSpeed(float speed) { walkSpeed_ = speed; }

		/**
		 * Set the run speed.
		 * @param speed the speed.
		 */
		void setRunSpeed(float speed) { runSpeed_ = speed; }

		/**
		 * Set the (normalized) laziness (1 = very lazy, 0 = not at all).
		 * @param laziness the laziness.
		 */
		void setLaziness(float laziness) { laziness_ = laziness; }

		/**
		 * Set the (normalized) spirituality (1 = very spiritual, 0 = not at all).
		 * @param spirituality the spirituality.
		 */
		void setSpirituality(float spirituality) { spirituality_ = spirituality; }

		/**
		 * Set the (normalized) alertness (1 = very alert, 0 = not at all).
		 * @param alertness the alertness.
		 */
		void setAlertness(float alertness) { alertness_ = alertness; }

		/**
		 * Set the (normalized) bravery (1 = very brave, 0 = not at all).
		 * @param bravery the bravery.
		 */
		void setBravery(float bravery) { bravery_ = bravery; }

		/**
		 * Set the base orientation of the mesh around y axis for looking into z direction.
		 * @param baseOrientation the base orientation.
		 */
		void setBaseOrientation(float baseOrientation) { baseOrientation_ = baseOrientation; }

		/**
		 * Set the floor height, only used if no height map is set.
		 * @param floorHeight the floor height.
		 */
		void setFloorHeight(float floorHeight) { floorHeight_ = floorHeight; }

		/**
		 * Set the height map.
		 * @param heightMap the height map.
		 */
		void setHeightMap(const ref_ptr<HeightMap> &heightMap);

		/**
		 * Add a place of interest for the NPC.
		 * @param place the place.
		 */
		void addPlaceOfInterest(const ref_ptr<Place> &place);

		void addWayPoint(const ref_ptr<WayPoint> &wp);

		void addConnection(
				const ref_ptr<WayPoint> &from,
				const ref_ptr<WayPoint> &to,
				bool bidirectional = true);

		// override
		void updateController(double dt) override;

		// override
		void updatePose(const TransformKeyFrame &currentFrame, double t) override;

	protected:
		ref_ptr<WorldModel> worldModel_;
		ref_ptr<SpatialIndex> spatialIndex_;
		ref_ptr<BoundingShape> indexedShape_;
		// Base orientation of the mesh around y axis
		float baseOrientation_ = M_PI_2;

		// some high level character traits
		float laziness_ = 0.5f;
		float spirituality_ = 0.5f;
		float alertness_ = 0.5f;
		float bravery_ = 0.5f;

		Vec2f homeLingerTime_ = Vec2f(60.0f, 300.0f);
		Vec2f socialLingerTime_ = Vec2f(60.0f, 300.0f);
		Vec2f prayLingerTime_ = Vec2f(60.0f, 300.0f);

		float avoidanceBlend_ = 0.7f; // 0 = ignore avoidance, 1 = only avoidance
		float maxTurnDegPerSec_ = 180.0f; // 90..360
		float maxTurn_ = maxTurnDegPerSec_ * (M_PI / 180.0f);

		// Terrain information.
		float floorHeight_ = 0.0f;
		ref_ptr<HeightMap> heightMap_;

		ref_ptr<PathPlanner> pathPlanner_;
		// Walking behavior parameters.
		float walkSpeed_ = 0.05f;
		float runSpeed_ = 0.1f;
		// The current path of the NPC, if any.
		std::vector<ref_ptr<WayPoint>> currentPath_;
		uint32_t currentPathIndex_ = 0;
		math::Bezier<Vec2f> bezierPath_;
		math::ArcLengthLUT bezierLUT_;
		// All semantic movements the NPC can perform.
		std::map<Motion, std::vector<const scene::AnimRange*>> motionRanges_;
		// Flag is used for continuous movement.
		bool isLastAnimationMovement_ = false;

		double lastDT_ = 0.0;

		// Places of interest.
		std::vector<ref_ptr<Place>> homePlaces_;
		std::vector<ref_ptr<Place>> spiritualPlaces_;
		std::vector<ref_ptr<Place>> gatheringPlaces_;
		std::vector<ref_ptr<Place>> patrolPoints_;

		struct Behaviour {
			// The current state of the NPC.
			State state = STATE_IDLE;
			// The current activity of the NPC.
			Activity activity = ACTIVITY_IDLE;
			int nextActivity = -1;
			// The current movement behavior of the NPC.
			Motion motion = MOTION_IDLE;
			// The place in focus of the NPC, if any.
			// e.g. idle does not have a target place, but pray has:
			// the NPC will try to be close to the target place, and oriented towards it.
			ref_ptr<Place> target;
			// The place of origin of the NPC, if any.
			ref_ptr<Place> source;
			ref_ptr<WorldObject> patient;
			ref_ptr<Affordance> affordance;
			// The object in focus of the NPC, if any.
			//ref_ptr<Object> patient;
			// The instrument used by the NPC, if any.
			//ref_ptr<Object> instrument;
			double lingerTime = 0.0;
			double activityTime = 0.0;
		};
		Behaviour currentBehaviour_;

		const WorldTime *worldTime_ = nullptr;

		void startNavigate(bool loopPath, bool advancePath);

		void startMotion();

		void setIdle();

		void setAtHome();

		void setAtMarket();

		void setAtShrine();

		void setPlaceActivity(const ref_ptr<Place> &place);

		void updatePlaceActivity(const ref_ptr<Place> &place);

		void updateBehavior(double dt);

		Vec3f computeNeighborAvoidance();

		float getHeight(const Vec2f &pos);

		void updatePathCurve(const Vec2f &source, const Vec2f &target);

		void updatePathCurve(const Vec2f &source, const Vec2f &target, const Vec3f &orientation);

		Vec2f pickTargetPosition(const ref_ptr<WayPoint> &wp);

		static void handleNeighbour(const BoundingShape &b_shape, void *userData);

		uint32_t findClosestWP_idx(const std::vector<ref_ptr<WayPoint>> &wps);
	};
} // namespace

#endif /* REGEN_NPC_CONTROLLER_H_ */
