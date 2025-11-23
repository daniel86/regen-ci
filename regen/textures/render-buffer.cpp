#include "render-buffer.h"

using namespace regen;

RenderBuffer::RenderBuffer(uint32_t numBuffers)
		: GLRectangle(glCreateRenderbuffers, glDeleteRenderbuffers, numBuffers),
		  format_(GL_RGBA) {
}

void RenderBuffer::set_format(GLenum format) { format_ = format; }

void RenderBuffer::begin(RenderState *rs) { rs->renderBuffer().push(id()); }

void RenderBuffer::end(RenderState *rs) { rs->renderBuffer().pop(); }

void RenderBuffer::storageMS(uint32_t numSamples) const {
	glNamedRenderbufferStorageMultisample(
			id(), numSamples, format_, width(), height());
}

void RenderBuffer::storage() const {
	glNamedRenderbufferStorage(
			id(), format_, width(), height());
}
