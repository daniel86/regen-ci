#ifndef REGEN_NPC_CONTROLLER_H_
#define REGEN_NPC_CONTROLLER_H_

#include "../animation/animation-node.h"
#include "regen/states/model-transformation.h"
#include "regen/utility/indexed.h"
#include "navigation/navigation-controller.h"

namespace regen {
	/**
	 * A base class for non-player character controllers.
	 */
	class NonPlayerCharacterController : public NavigationController {
	public:
		NonPlayerCharacterController(
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

#endif /* REGEN_NPC_CONTROLLER_H_ */
