#include "qt-events.h"
#include <qkeysequence.h>
#include <regen/scene/scene.h>

using namespace regen;

QtCameraEventHandler::QtCameraEventHandler(
		const ref_ptr<CameraController> &m,
		const std::vector<CameraCommandMapping> &keyMappings)
		: EventHandler(),
		  m_(m),
		  buttonPressed_(false),
		  sensitivity_(0.0002f) {
	for (const auto &x: keyMappings) {
		keyMappings_[x.key] = x;
	}
}

void QtCameraEventHandler::call(EventObject *evObject, EventData *data) {
	if (data->eventID == Scene::MOUSE_MOTION_EVENT) {
		if (buttonPressed_) {
			auto *ev = (Scene::MouseMotionEvent *) data;
			Vec2f delta((float) ev->dx, (float) ev->dy);
			m_->lookLeft(delta.x * sensitivity_);
			m_->lookUp(delta.y * sensitivity_);
		}
	}
	else if (data->eventID == Scene::KEY_EVENT) {
		auto *ev = (Scene::KeyEvent *) data;
		auto evKey = QKeySequence(ev->key).toString().toStdString();
		auto needle = keyMappings_.find(evKey);

		CameraCommand cmd = CameraCommand::NONE;
		if (needle != keyMappings_.end()) {
			cmd = needle->second.command;
		} else {
			if (ev->key == Qt::Key_W || ev->key == Qt::Key_Up) cmd = CameraCommand::MOVE_FORWARD;
			else if (ev->key == Qt::Key_S || ev->key == Qt::Key_Down) cmd = CameraCommand::MOVE_BACKWARD;
			else if (ev->key == Qt::Key_A || ev->key == Qt::Key_Left) cmd = CameraCommand::MOVE_LEFT;
			else if (ev->key == Qt::Key_D || ev->key == Qt::Key_Right) cmd = CameraCommand::MOVE_RIGHT;
			else if (ev->key == Qt::Key_Space) cmd = CameraCommand::JUMP;
			else if (ev->key == Qt::Key_PageUp) cmd = CameraCommand::MOVE_UP;
			else if (ev->key == Qt::Key_PageDown) cmd = CameraCommand::MOVE_DOWN;
		}

		if (cmd != CameraCommand::NONE) {
			if (cmd == CameraCommand::MOVE_FORWARD) m_->moveForward(!ev->isUp);
			else if (cmd == CameraCommand::MOVE_BACKWARD) m_->moveBackward(!ev->isUp);
			else if (cmd == CameraCommand::MOVE_LEFT) m_->moveLeft(!ev->isUp);
			else if (cmd == CameraCommand::MOVE_RIGHT) m_->moveRight(!ev->isUp);
			else if (cmd == CameraCommand::MOVE_UP) m_->moveUp(!ev->isUp);
			else if (cmd == CameraCommand::MOVE_DOWN) m_->moveDown(!ev->isUp);
			else if (cmd == CameraCommand::JUMP && !ev->isUp) m_->jump();
		}
	}
	else if (data->eventID == Scene::BUTTON_EVENT) {
		auto *ev = (Scene::ButtonEvent *) data;
		if (ev->button == Scene::MOUSE_BUTTON_LEFT) {
			buttonPressed_ = ev->pressed;
		} else if (ev->button == Scene::MOUSE_WHEEL_UP) {
			m_->zoomIn(0.5);
		} else if (ev->button == Scene::MOUSE_WHEEL_DOWN) {
			m_->zoomOut(0.5);
		}
	}
}

QtMotionEventHandler::QtMotionEventHandler(
		const ref_ptr<UserController> &ctrl,
		const std::vector<MotionCommandMapping> &keyMappings)
		: EventHandler(),
		  ctrl_(ctrl) {
	for (const auto &x: keyMappings) {
		keyMappings_[x.key] = x;
	}
	motionCounter_.resize(static_cast<int>(MotionType::MOTION_LAST), 0);
}

void QtMotionEventHandler::call(EventObject *evObject, EventData *data) {
	std::map<std::string, MotionCommandMapping>::iterator needle = keyMappings_.end();
	bool isPressed = false;

	if (data->eventID == Scene::KEY_EVENT) {
		auto *ev = (Scene::KeyEvent *) data;
		auto evStr = QKeySequence(ev->key).toString();
		needle = keyMappings_.find(evStr.toStdString());
		isPressed = !ev->isUp;
	} else if (data->eventID == Scene::BUTTON_EVENT) {
		auto *ev = (Scene::ButtonEvent *) data;
		needle = keyMappings_.find(REGEN_STRING("button" << ev->button));
		isPressed = ev->pressed;
	}
	if (needle == keyMappings_.end()) return;

	MotionType motion = needle->second.command;
	int motionIdx = static_cast<int>(motion);
	int &motionCount = motionCounter_[motionIdx];
	if (needle->second.toggle) {
		if (isPressed) {
			if (motionCount == 0) {
				ctrl_->setMotionActive(motion, true, needle->second.reverse);
				motionCount = 1;
			} else {
				ctrl_->setMotionActive(motion, false, needle->second.reverse);
				motionCount = 0;
			}
		}
	} else if (isPressed) {
		motionCount++;
		if (motionCount == 1) {
			ctrl_->setMotionActive(motion, true, needle->second.reverse);
		}
	} else {
		motionCount--;
		if (motionCount == 0) {
			ctrl_->setMotionActive(motion, false, needle->second.reverse);
		} else if (motionCount < 0) {
			motionCount = 0;
		}
	}
}
