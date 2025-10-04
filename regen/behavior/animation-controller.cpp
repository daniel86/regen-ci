#include "../behavior/animation-controller.h"

using namespace regen;

AnimationController::AnimationController(
			const ref_ptr<Mesh> &mesh,
			const Indexed<ref_ptr<ModelTransformation>> &tfIndexed,
			const ref_ptr<NodeAnimationItem> &animItem,
			const ref_ptr<WorldModel> &world) :
		MotionController(mesh, tfIndexed, world),
		tf_(tfIndexed.value),
		animItem_(animItem) {
}

void AnimationController::setTarget(
			const Vec3f &target,
			const std::optional<Vec3f> &rotation,
			GLdouble dt) {
	frames_.clear();
	TransformAnimation::push_back(target, rotation, dt);
	if (!isRunning()) { startAnimation(); }
}

void AnimationController::animate(GLdouble dt) {
	updateController(dt);
	if (it_ != frames_.end()) {
		MotionController::animate(dt);
	}
}
