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

void NonPlayerCharacterController::animate(GLdouble dt) {
	updateController(dt);
	NavigationController::animate(dt);
}
