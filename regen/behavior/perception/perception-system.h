#ifndef REGEN_PERCEPTION_SYSTEM_H_
#define REGEN_PERCEPTION_SYSTEM_H_

#include "perception-monitor.h"
#include "regen/shapes/spatial-index.h"
#include "regen/shapes/bounding-shape.h"

namespace regen {
	/**
	 * A perception system for NPCs.
	 * It uses a spatial index to detect nearby objects and notifies registered perception monitors.
	 */
	class PerceptionSystem {
	public:
		/**
		 * Constructor.
		 * @param spatialIndex the spatial index to use for perception.
		 * @param indexedShape the shape of the NPC in the spatial index.
		 */
		PerceptionSystem(
			const ref_ptr<SpatialIndex> &spatialIndex,
			const ref_ptr<BoundingShape> &indexedShape);

		virtual ~PerceptionSystem() = default;

		/**
		 * Add a perception monitor to be notified of perception events.
		 * @param monitor the perception monitor.
		 */
		void addMonitor(PerceptionMonitor *monitor);

		/**
		 * Remove a perception monitor.
		 * @param monitor the perception monitor.
		 */
		void removeMonitor(PerceptionMonitor *monitor);

		/**
		 * Set the collision bit for avoiding collisions with other NPCs.
		 * @param bit the collision bit.
		 */
		void setCollisionBit(uint32_t bit) { collisionMask_ = (1 << bit); }

		/**
		 * Update the perception system.
		 * This should be called periodically to update the perception data and notify monitors.
		 * @param dt_s the time difference in seconds.
		 */
		void update(double dt_s);

		/**
		 * Handle an intersection with another shape.
		 * @param other the other shape.
		 * @param userData user data passed to the handler.
		 */
		void handleIntersection(const BoundingShape &other, void *userData);

	protected:
		// The spatial index used for perception.
		ref_ptr<SpatialIndex> spatialIndex_;
		// The shape of the NPC in the spatial index.
		ref_ptr<BoundingShape> indexedShape_;
		// shape used for collision avoidance, it extends in walk direction
		ref_ptr<BoundingShape> collisionShape_;
		// collision mask to ignore collisions with other NPCs
		uint32_t collisionMask_ = 0;
		// The perception data computed during intersection handling.
		PerceptionData perceptionData_;
		// The list of registered perception monitors.
		std::vector<PerceptionMonitor*> monitors_;
	};
} // namespace

#endif /* REGEN_PERCEPTION_MONITOR_H_ */

