#ifndef REGEN_DETECTION_EVENT_H_
#define REGEN_DETECTION_EVENT_H_
#include "regen/behavior/world/world-object.h"
#include "perception-monitor.h"

namespace regen {
	/**
	 * Data structure for detection events.
	 */
	struct DetectionData {
		const WorldObject *object = nullptr;
	};

	/**
	 * A detection event that is generated when an object is detected.
	 */
	class DetectionEvent : public PerceptionEvent {
	public:
		DetectionData data;
		DetectionEvent() : PerceptionEvent(PerceptionEventType::DETECTION) {}
		~DetectionEvent() override = default;
	};

	/**
	 * A perception monitor that can be registered with a perception system.
	 * It provides callbacks for detection events.
	 */
	class DetectionMonitor :  public PerceptionMonitor {
	public:
		DetectionMonitor() :
			PerceptionMonitor(PerceptionEventType::DETECTION)
		{}
	};
} // namespace

#endif /* REGEN_DETECTION_EVENT_H_ */

