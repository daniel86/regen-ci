#include "camera-anchor.h"

using namespace regen;

TransformCameraAnchor::TransformCameraAnchor(const ref_ptr<ModelTransformation> &transform)
		: transform_(transform), mode_(LOOK_AT_FRONT) {
}

Vec3f TransformCameraAnchor::position() {
    auto modelMatrix = transform_->modelMat()->getVertex(0);
	const Mat4f &M = modelMatrix.r;
    return M.position() + M.mul_t30(offset_) / M.scaling();
}

Vec3f TransformCameraAnchor::direction() {
    auto modelMatrix = transform_->modelMat()->getVertex(0);
    auto diff = (modelMatrix.r.position() - position());
    diff.normalize();
    return diff;
}
