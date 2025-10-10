#ifndef REGEN_NAVIGATION_CONTROLLER_H_
#define REGEN_NAVIGATION_CONTROLLER_H_

#include "../../animation/transform-animation.h"
#include "regen/math/bezier.h"
#include "regen/states/model-transformation.h"
#include "regen/textures/height-map.h"
#include "regen/utility/indexed.h"

namespace regen {
	class NavigationController : public TransformAnimation {
	public:
		NavigationController(
			const ref_ptr<Mesh> &mesh,
			const Indexed<ref_ptr<ModelTransformation>> &tfIndexed,
			const ref_ptr<WorldModel> &world);

		/**
		 * @param numEntries The number of entries in the Bezier arc length lookup table.
		 */
		void setNumBezierLUTEntries(uint32_t numEntries) {
			numLUTEntries_ = numEntries;
		}

		/**
		 * Set the spatial index for path finding.
		 * @param spatialIndex the spatial index.
		 */
		void setSpatialIndex(const ref_ptr<SpatialIndex> &spatialIndex, std::string_view shapeName);

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
		 * Set the maximum turn speed in degrees per second.
		 * @param deg the degrees per second.
		 */
		void setMaxTurnDegPerSecond(float deg) {
			maxTurnDegPerSec_ = deg;
			maxTurn_ = maxTurnDegPerSec_ * (M_PI / 180.0f);
		}

		/**
		 * set the personal space for avoidance.
		 * @param space the personal space.
		 */
		void setPersonalSpace(float space) { personalSpace_ = space; }

		/**
		 * Set the weight for avoidance steering (0 = no avoidance, 1 = full avoidance).
		 * @param weight the weight.
		 */
		void setAvoidanceWeight(float weight) { avoidanceWeight_ = weight; }

		/**
		 * Set the distance where collisions are ignored when approaching goals.
		 * @param distance the distance.
		 */
		void setPushThroughDistance(float distance) { pushThroughDistance_ = distance; }

		/**
		 * Set the look ahead treshold for dynamic avoidance.
		 * When closer to the goal than this threshold, the look ahead distance
		 * will be reduced to allow better approach to the goal.
		 * @param threshold the threshold.
		 */
		void setLookAheadThreshold(float threshold) { lookAheadThreshold_ = threshold; }

		/**
		 * Set the weight for wall tangent steering (0 = no wall following, 1 = full wall following).
		 * @param weight the weight.
		 */
		void setWallTangentWeight(float weight) { wallTangentWeight_ = weight; }

		/**
		 * Set the weight for velocity orientation (0 = no orientation, 1 = full orientation).
		 * @param weight the weight.
		 */
		void setVelOrientationWeight(float weight) { velOrientationWeight_ = weight; }

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
		 * Set the collision bit for avoiding collisions with other NPCs.
		 * @param bit the collision bit.
		 */
		void setCollisionBit(uint32_t bit) { collisionMask_ = (1 << bit); }

		/**
		 * @return the look ahead distance for the collision shape.
		 */
		float lookAheadDistance() const { return lookAheadDistance_; }

		/**
		 * Set the height map.
		 * @param heightMap the height map.
		 */
		void setHeightMap(const ref_ptr<HeightMap> &heightMap);

		// override
		void updatePose(const TransformKeyFrame &currentFrame, double t) override;

	protected:
		ref_ptr<WorldModel> worldModel_;
		ref_ptr<SpatialIndex> spatialIndex_;
		ref_ptr<BoundingShape> indexedShape_;
		// shape used for collision avoidance, it extends in walk direction
		ref_ptr<BoundingShape> collisionShape_;
		// if set, we will ignore collisions with this shape
		ref_ptr<BoundingShape> patientShape_;
		uint32_t collisionMask_ = 0;
		// Base orientation of the mesh around y axis
		float baseOrientation_ = M_PI_2;

		float personalSpace_ = 4.5f;
		float wallTangentWeight_ = 0.4f;
		float avoidanceWeight_ = 0.7f;
		float velOrientationWeight_ = 0.9f;
		float maxTurnDegPerSec_ = 90.0f;
		float maxTurn_ = maxTurnDegPerSec_ * (M_PI / 180.0f);
		float affordanceSlotRadius_ = 0.75f;
		float pushThroughDistance_ = 1.0f;
		float lookAheadThreshold_ = 6.0f;
		uint32_t numLUTEntries_ = 100;

		// Terrain information.
		float floorHeight_ = 0.0f;
		ref_ptr<HeightMap> heightMap_;

		// Walking behavior parameters.
		float walkSpeed_ = 0.05f;
		float runSpeed_ = 0.1f;
		float lookAheadDistance_ = 15.0f;
		float heightSmoothFactor_ = 12.0f;
		bool isWalking_ = true;
		// The current path of the NPC, if any.
		std::vector<ref_ptr<WayPoint>> currentPath_;
		float distanceToTarget_ = std::numeric_limits<float>::max();
		uint32_t currentPathIndex_ = 0;
		math::Bezier<Vec2f> bezierPath_;
		math::ArcLengthLUT bezierLUT_;

		double lastDT_ = 0.0;

		Vec3f computeNeighborAvoidance();

		float getHeight(const Vec2f &pos);

		void updatePathCurve(const Vec2f &source, const Vec2f &target);

		void updatePathCurve(const Vec2f &source, const Vec2f &target, const Vec3f &orientation);

		static void handleNeighbourStatic(const BoundingShape &b_shape, void *userData);

		void handleNeighbour(const BoundingShape &other, void *userData);

		void updateTransformFrame(const Vec2f &target, const Vec3f &desiredDir);

	private:
		void updateControllerOrientation(double bezierTime);

		void updateControllerVelocity(double bezierTime);

		struct NPCNeighborData {
			NavigationController *npc;
			float lookAhead = 10.0f;
			uint32_t neighborCount = 0;
			Vec3f avoidance = Vec3f::zero();
		};

		void handleCharacterCollision(const BoundingShape &other, NPCNeighborData *data);

		void handleWallCollision(const BoundingShape &other, NPCNeighborData *data);
	};
} // namespace

#endif /* REGEN_NAVIGATION_CONTROLLER_H_ */
