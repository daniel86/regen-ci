#ifndef REGEN_COLLISION_EVENT_H_
#define REGEN_COLLISION_EVENT_H_
#include "regen/shapes/bounding-shape.h"
#include "perception-monitor.h"

namespace regen {
	/**
	 * Data structure for perception events.
	 */
	struct CollisionData {
		const BoundingShape *self = nullptr;
		// The perceived shape.
		const BoundingShape *other = nullptr;
		// The distance between the perceived shape and the perceiver.
		float distance = 0.0f;
		// The vector from the perceived shape to the perceiver.
		Vec3f delta = Vec3f::zero();
		// The normalized direction from the perceived shape to the perceiver.
		Vec3f dir = Vec3f::zero();
		// Whether the perceived shape is behind the perceiver.
		bool isBehind = false;
	};

	/**
	 * A collision event that is generated when a possible collision is detected.
	 * This does not necessarily mean that a collision has occurred, but that
	 * the two objects are close enough to potentially collide.
	 */
	class CollisionEvent : public PerceptionEvent {
	public:
		CollisionData data;
		CollisionEvent() : PerceptionEvent(PerceptionEventType::COLLISION) {}
		~CollisionEvent() override = default;
	};

	/**
	 * A perception monitor that can be registered with a perception system.
	 * It provides callbacks for perception events.
	 */
	class CollisionMonitor :  public PerceptionMonitor {
	public:
		// Callback types.
		using Handler = std::function<void(const CollisionEvent&, void*)>;
		using Init = std::function<void(void*)>;
		using Cleanup = std::function<void(void*)>;

		/**
		 * Constructor.
		 * @param handle the handler function for perception events.
		 * @param init the initialization function called when the monitor is added to a perception system.
		 * @param cleanup the cleanup function called when the monitor is removed from a perception system.
		 */
		CollisionMonitor(Handler handle, Init init, Cleanup cleanup, void *userData = nullptr) :
			PerceptionMonitor(PerceptionEventType::COLLISION),
			onDetection_(handle),
			onFrameInit_(init),
			onFrameCleanup_(cleanup),
			userData_(userData) {}

		~CollisionMonitor() override = default;

		/**
		 * Call the initialization callback.
		 */
		void initializeCollisionFrame() {
			onFrameInit_(userData_);
		}

		/**
		 * Call the cleanup callback.
		 */
		void finalizeCollisionFrame() {
			onFrameCleanup_(userData_);
		}

		/**
		 * Call the perception handler with the given perception data.
		 * @param evt the perception event data.
		 */
		void emitCollisionEvent(const CollisionEvent& evt) {
			onDetection_(evt, userData_);
		}

	private:
		Handler onDetection_ = [](const CollisionEvent&, void*) {};
		Init onFrameInit_ = [](void*) {};
		Cleanup onFrameCleanup_ = [](void*) {};
		void *userData_ = nullptr;
	};
} // namespace

#endif /* REGEN_COLLISION_EVENT_H_ */

