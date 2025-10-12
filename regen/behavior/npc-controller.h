#ifndef REGEN_NPC_CONTROLLER_H_
#define REGEN_NPC_CONTROLLER_H_

#include "../animation/bone-tree.h"
#include "regen/states/model-transformation.h"
#include "regen/utility/indexed.h"
#include "navigation/navigation-controller.h"
#include "skeleton/bone-controller.h"

namespace regen {
	/**
	 * A base class for non-player character controllers.
	 */
	class NonPlayerCharacterController : public NavigationController {
	public:
		NonPlayerCharacterController(
			const Indexed<ref_ptr<ModelTransformation>> &tfIndexed,
			const ref_ptr<BoneAnimationItem> &animItem,
			const ref_ptr<WorldModel> &world);

		~NonPlayerCharacterController() override = default;

		/**
		 * Set the target for movement.
		 * @param target the target position.
		 * @param rotation the target rotation.
		 * @param dt the time difference.
		 */
		void setTarget(const Vec3f &target, const std::optional<Vec3f> &rotation, double dt);

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
		ref_ptr<BoneAnimationItem> animItem_;
	};
} // namespace

#endif /* REGEN_NPC_CONTROLLER_H_ */
