#ifndef REGEN_ANIMATION_CONTROLLER_H_
#define REGEN_ANIMATION_CONTROLLER_H_

#include "../animation/animation-node.h"
#include "regen/states/model-transformation.h"
#include "regen/utility/indexed.h"
#include "motion/motion-controller.h"

namespace regen {
	/**
	 * A controller of a mesh animation.
	 * Has access to the mesh transform and node animation to
	 * control the bone animations.
	 */
	class AnimationController : public MotionController {
	public:
		AnimationController(
			const ref_ptr<Mesh> &mesh,
			const Indexed<ref_ptr<ModelTransformation>> &tfIndexed,
			const ref_ptr<NodeAnimationItem> &animItem,
			const ref_ptr<WorldModel> &world);

		/**
		 * Set the target for movement.
		 * @param target the target position.
		 * @param rotation the target rotation.
		 * @param dt the time difference.
		 */
		void setTarget(
					const Vec3f &target,
					const std::optional<Vec3f> &rotation,
					GLdouble dt);

		/**
		 * Update animation and movement target.
		 * @param dt the time difference.
		 */
		virtual void updateController(double dt) = 0;

		/**
		 * Initialize the controller.
		 * This is called once before the first updateController() call.
		 */
		virtual void initializeController() {}

		// override
		void animate(GLdouble dt) override;

	protected:
		ref_ptr<ModelTransformation> tf_;
		ref_ptr<NodeAnimationItem> animItem_;
		const AnimRange *lastRange_ = nullptr;
	};
} // namespace

#endif /* REGEN_ANIMATION_CONTROLLER_H_ */
