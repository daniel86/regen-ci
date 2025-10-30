#ifndef REGEN_PERCEPTION_MONITOR_H_
#define REGEN_PERCEPTION_MONITOR_H_

namespace regen {
	/**
	 * Types of perception events.
	 */
	enum class PerceptionEventType {
		COLLISION = 0,
		DETECTION
	};

	/**
	 * Base class for perception events.
	 */
	class PerceptionEvent {
	public:
		const PerceptionEventType type;

		explicit PerceptionEvent(PerceptionEventType type) : type(type) {}

		virtual ~PerceptionEvent() = default;
	};

	/**
	 * A perception monitor that can be registered with a perception system.
	 * It provides callbacks for perception events.
	 */
	class PerceptionMonitor {
	public:
		const PerceptionEventType type;

		explicit PerceptionMonitor(PerceptionEventType type) : type(type) {}

		virtual ~PerceptionMonitor() = default;
	};
} // namespace

#endif /* REGEN_PERCEPTION_MONITOR_H_ */

