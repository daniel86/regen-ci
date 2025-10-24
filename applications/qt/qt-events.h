#ifndef REGEN_QT_EVENTS_H_
#define REGEN_QT_EVENTS_H_

#include <regen/camera/camera-controller.h>
#include "regen/behavior/user-controller.h"

namespace regen {
	class QtCameraEventHandler : public EventHandler {
	public:
		QtCameraEventHandler(const ref_ptr<CameraController> &ctrl,
				const std::vector<CameraCommandMapping> &keyMappings);

		~QtCameraEventHandler() override = default;

		void set_sensitivity(float val) { sensitivity_ = val; }

		// Override
		void call(EventObject *evObject, EventData *data) override;

	protected:
		ref_ptr<CameraController> m_;
		std::map<std::string, CameraCommandMapping> keyMappings_;
		bool buttonPressed_;
		float sensitivity_;
	};

	class QtMotionEventHandler : public EventHandler {
	public:
		QtMotionEventHandler(const ref_ptr<UserController> &ctrl,
				const std::vector<MotionCommandMapping> &keyMappings);

		~QtMotionEventHandler() override = default;

		// Override
		void call(EventObject *evObject, EventData *data) override;

	protected:
		ref_ptr<UserController> ctrl_;
		std::map<std::string, MotionCommandMapping> keyMappings_;
		std::vector<int> motionCounter_;
	};
}


#endif /* REGEN_QT_EVENTS_H_ */
