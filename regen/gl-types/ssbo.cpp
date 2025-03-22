#include "ssbo.h"

using namespace regen;

SSBO::SSBO(const std::string &name, BufferUsage usage) :
		BufferBlock(SHADER_STORAGE_BUFFER, usage,
		            BufferBlock::BUFFER,
		            BufferBlock::STD430),
		ShaderInput(name, GL_INVALID_ENUM,
		            0, 0, 0, GL_FALSE) {
	enableUniform_ = [this](GLint loc) { enableBufferBlock(loc); };
	isBufferBlock_ = GL_TRUE;
	isVertexAttribute_ = GL_FALSE;
	isVertexAttribute_ = GL_FALSE;
}

void SSBO::write(std::ostream &out) const {
	out << "buffer " << name() << " {\n";
	out << "};";
}
