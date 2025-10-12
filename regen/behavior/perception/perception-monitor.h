#ifndef REGEN_PERCEPTION_MONITOR_H_
#define REGEN_PERCEPTION_MONITOR_H_

#include <functional>
#include "perception-data.h"

namespace regen {
	/**
	 * A perception monitor that can be registered with a perception system.
	 * It provides callbacks for perception events.
	 */
	class PerceptionMonitor {
	public:
		// Callback types.
		using Handler = std::function<void(const PerceptionData&, void*)>;
		using Init = std::function<void(void*)>;
		using Cleanup = std::function<void(void*)>;

		/**
		 * Constructor.
		 * @param handle the handler function for perception events.
		 * @param init the initialization function called when the monitor is added to a perception system.
		 * @param cleanup the cleanup function called when the monitor is removed from a perception system.
		 */
		PerceptionMonitor(Handler handle, Init init, Cleanup cleanup, void *userData = nullptr) :
			onPerception_(handle),
			onPerceptionInit_(init),
			onPerceptionCleanup_(cleanup),
			userData_(userData) {}

		virtual ~PerceptionMonitor() = default;

		/**
		 * Call the initialization callback.
		 */
		void callOnPerceptionInit() {
			onPerceptionInit_(userData_);
		}

		/**
		 * Call the cleanup callback.
		 */
		void callOnPerceptionCleanup() {
			onPerceptionCleanup_(userData_);
		}

		/**
		 * Call the perception handler with the given perception data.
		 * @param perceptionData the perception data.
		 */
		void callOnPerception(const PerceptionData& perceptionData) {
			onPerception_(perceptionData, userData_);
		}

	private:
		Handler onPerception_ = [](const PerceptionData&, void*) {};
		Init onPerceptionInit_ = [](void*) {};
		Cleanup onPerceptionCleanup_ = [](void*) {};
		void *userData_ = nullptr;
	};
} // namespace

#endif /* REGEN_PERCEPTION_MONITOR_H_ */

