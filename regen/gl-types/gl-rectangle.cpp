#include "gl-rectangle.h"

using namespace regen;

GLRectangle::GLRectangle(
		CreateObjectFunc createObjects,
		ReleaseObjectFunc releaseObjects,
		GLuint numObjects)
		: GLObject(createObjects, releaseObjects, numObjects) {
	size_ = ref_ptr<ShaderInput2f>::alloc("rectangleSize");
	sizeInverse_ = ref_ptr<ShaderInput2f>::alloc("rectangleSizeInverse");
	size_->setUniformUntyped();
	sizeInverse_->setUniformUntyped();
	set_rectangleSize(2, 2);
}

void GLRectangle::set_rectangleSize(GLuint width, GLuint height) {
	auto w_f = static_cast<float>(width);
	auto h_f = static_cast<float>(height);
	size_->setVertex(0, Vec2f(w_f, h_f));
	sizeInverse_->setVertex(0, Vec2f(1.0f / w_f, 1.0f / h_f));
}

const ref_ptr<ShaderInput2f> &GLRectangle::sizeInverse() const { return sizeInverse_; }

const ref_ptr<ShaderInput2f> &GLRectangle::size() const { return size_; }

GLuint GLRectangle::width() const { return size_->getVertex(0).r.x; }

GLuint GLRectangle::height() const { return size_->getVertex(0).r.y; }
