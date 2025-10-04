#ifndef REGEN_PERCEPTION_MONITOR_H_
#define REGEN_PERCEPTION_MONITOR_H_

#include "regen/utility/ref-ptr.h"
#include "../blackboard.h"
#include "../world/world-model.h"

namespace regen {
	class PerceptionMonitor {
	public:
		PerceptionMonitor(
			const ref_ptr<Blackboard> &board,
			const ref_ptr<WorldModel> &world);

		void setSensorRate(float rate) { sensorRate_ = rate; }

		virtual ~PerceptionMonitor() = default;

		virtual void initialize() {}

		void update(float dt_s);

	protected:
		ref_ptr<Blackboard> board_;
		ref_ptr<WorldModel> world_;
		float sensorRate_ = 1.0f; // in Hz
		float lastPerceptionTime_ = 0.0f;

		virtual void perceive() = 0;
	};
} // namespace

#endif /* REGEN_PERCEPTION_MONITOR_H_ */

