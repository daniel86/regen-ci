#ifndef REGEN_NPC_CONTROLLER_H_
#define REGEN_NPC_CONTROLLER_H_

#include <regen/math/vector.h>
#include <regen/textures/texture.h>
#include <regen/animations/animation-controller.h>
#include "animation-node.h"
#include "regen/shapes/bounds.h"
#include "regen/states/model-transformation.h"
#include "regen/math/bezier.h"

namespace regen {
	class NPCController : public AnimationController {
	public:
		/**
		 * The state of the NPC.
		 */
		enum State {
			STATE_IDLE = 0,
			// Try to reach home place as fast as possible.
			STATE_GO_HOME,
			// Go to a place to socialize with other NPCs.
			STATE_GO_SOCIALIZE,
			// Go to a patrolling way point.
			STATE_GO_PATROL,
			// Go to a place to pray.
			// But if not spiritual the NPC would not take long detours.
			STATE_GO_PRAY,
			// Alert, e.g. because of a noise.
			// Look around, try to find the threat.
			STATE_ON_ALERT,
			// Run away from a threat.
			STATE_RUN_AWAY,
			// Fight against a threat.
			STATE_FIGHTING,
			// At home, doing nothing special.
			// Interact with any obects of interest at random for now.
			STATE_AT_HOME,
			// Sleeping, usually at home.
			// But very lazy NPCs may sleep anywhere.
			STATE_SLEEPING,
			// Socializing with other NPCs.
			STATE_SOCIALIZE,
			// Patrolling along a path in the territory.
			STATE_PATROLLING,
			// Pray, usually after reaching a temple.
			// But very spiritual NPCs may pray anywhere.
			STATE_PRAYING,
			STATE_LAST
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
		 * Categories of places of interest for the NPC.
		 */
		enum PlaceType {
			PLACE_HOME = 0,
			PLACE_SPIRITUAL,
			PLACE_GATHERING,
			PLACE_PATROL_POINT
		};

		/**
		 * A place of interest for the NPC.
		 */
		struct Place {
			const PlaceType type;
			const float radius = 1.0f;
			ref_ptr<ShaderInput3f> pos;

			explicit Place(PlaceType type, float radius = 1.0f) :
				type(type),
				radius(radius),
				pos(createUniform<ShaderInput3f>("pos", Vec3f::zero())) {}
			Place(PlaceType type, const Vec2f &position, float radius = 1.0f) :
				type(type),
				radius(radius),
				pos(createUniform<ShaderInput3f>("pos", Vec3f(position.x, 0.0f, position.y))) {}
			Place(PlaceType type, const Vec3f &position, float radius = 1.0f) :
				type(type),
				radius(radius),
				pos(createUniform<ShaderInput3f>("pos", position)) {}
			Place(PlaceType type, const ref_ptr<ShaderInput3f> &position, float radius = 1.0f) :
				type(type),
				radius(radius),
				pos(position) {}
		};

		/**
		 * Constructor.
		 * @param tf the model transformation.
		 * @param animation the node animation.
		 * @param ranges the animation ranges.
		 */
		NPCController(
			const ref_ptr<ModelTransformation> &tf,
			uint32_t tfIdx,
			const ref_ptr<NodeAnimation> &animation,
			const std::vector<scene::AnimRange> &ranges);

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
		 * Set the max height.
		 * @param maxHeight the max height.
		 */
		void setMaxHeight(float maxHeight) { maxHeight_ = maxHeight; }

		/**
		 * Set the min height.
		 * @param minHeight the min height.
		 */
		void setMinHeight(float minHeight) { minHeight_ = minHeight; }

		/**
		 * Set the base orientation of the mesh around y axis for looking into z direction.
		 * @param baseOrientation the base orientation.
		 */
		void setBaseOrientation(float baseOrientation) { baseOrientation_ = baseOrientation; }

		/**
		 * Set the floor height.
		 * @param floorHeight the floor height.
		 */
		void setFloorHeight(float floorHeight) { floorHeight_ = floorHeight; }

		/**
		 * Set the height map.
		 * @param heightMap the height map.
		 * @param heightMapCenter the center.
		 * @param heightMapSize the size.
		 * @param heightMapFactor the factor.
		 */
		void setHeightMap(
				const ref_ptr<Texture2D> &heightMap,
				const Vec2f &heightMapCenter,
				const Vec2f &heightMapSize,
				float heightMapFactor);

		/**
		 * Add a place of interest for the NPC.
		 * @param place the place.
		 */
		void addPlaceOfInterest(const ref_ptr<Place> &place);

		// override
		void updateController(double dt) override;

		// override
		void updatePose(const TransformKeyFrame &currentFrame, double t) override;

	protected:
		// Base orientation of the mesh around y axis
		float baseOrientation_ = M_PI_2;

		// some high level character traits
		float laziness_ = 0.5f;
		float spirituality_ = 0.5f;
		float alertness_ = 0.5f;
		float bravery_ = 0.5f;

		// Terrain information.
		float floorHeight_ = 0.0f;
		float maxHeight_;
		float minHeight_;
		ref_ptr<Texture2D> heightMap_;
		Bounds<Vec2f> heightMapBounds_;
		float heightMapFactor_ = 8.0f;

		// Walking behavior parameters.
		float walkSpeed_ = 0.05f;
		float runSpeed_ = 0.1f;
		// The current path of the NPC, if any.
		math::Bezier<Vec2f> bezierPath_;
		// All semantic movements the NPC can perform.
		std::map<Motion, std::vector<const scene::AnimRange*>> motionRanges_;
		// Flag is used for continuous movement.
		bool isLastAnimationMovement_ = false;

		// Places of interest.
		std::vector<ref_ptr<Place>> homePlaces_;
		std::vector<ref_ptr<Place>> spiritualPlaces_;
		std::vector<ref_ptr<Place>> gatheringPlaces_;
		std::vector<ref_ptr<Place>> patrolPoints_;

		struct Behaviour {
			// The current state of the NPC.
			State state = STATE_IDLE;
			// The current movement behavior of the NPC.
			Motion motion = MOTION_IDLE;
			// The place in focus of the NPC, if any.
			// e.g. idle does not have a target place, but pray has:
			// the NPC will try to be close to the target place, and oriented towards it.
			ref_ptr<Place> target;
			// The place of origin of the NPC, if any.
			ref_ptr<Place> source;
			// The object in focus of the NPC, if any.
			//ref_ptr<Object> patient;
			// The instrument used by the NPC, if any.
			//ref_ptr<Object> instrument;
		};
		Behaviour currentBehaviour_;

		const WorldTime *worldTime_ = nullptr;

		void startNavigate();

		void startMotion();

		void updateBehavior();

		float getHeight(const Vec2f &pos);

		void setIdle();
	};

	std::ostream &operator<<(std::ostream &out, const NPCController::PlaceType &v);

	std::istream &operator>>(std::istream &in, NPCController::PlaceType &v);
} // namespace

#endif /* REGEN_NPC_CONTROLLER_H_ */
