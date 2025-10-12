#include "../behavior/npc-controller.h"

using namespace regen;

NonPlayerCharacterController::NonPlayerCharacterController(
			const Indexed<ref_ptr<ModelTransformation>> &tfIndexed,
			const ref_ptr<BoneAnimationItem> &animItem,
			const ref_ptr<WorldModel> &world) :
		NavigationController(tfIndexed, world),
		tf_(tfIndexed.value),
		animItem_(animItem) {
}

void NonPlayerCharacterController::setTarget(const Vec3f &target, const std::optional<Vec3f> &rotation, double dt) {
	frames_.clear();
	TransformAnimation::push_back(target, rotation, dt);
	if (!isRunning()) { startAnimation(); }
}

void NonPlayerCharacterController::animate(GLdouble dt) {
	updateController(dt);
	if (it_ != frames_.end()) {
		NavigationController::animate(dt);
	}
}
