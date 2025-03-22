#include "buffer-target.h"

using namespace regen;

namespace regen {
	GLenum glBufferTarget(BufferTarget target) {
		switch (target) {
			case ARRAY_BUFFER:
				return GL_ARRAY_BUFFER;
			case ELEMENT_ARRAY_BUFFER:
				return GL_ELEMENT_ARRAY_BUFFER;
			case PIXEL_PACK_BUFFER:
				return GL_PIXEL_PACK_BUFFER;
			case PIXEL_UNPACK_BUFFER:
				return GL_PIXEL_UNPACK_BUFFER;
			case UNIFORM_BUFFER:
				return GL_UNIFORM_BUFFER;
			case TEXTURE_BUFFER:
				return GL_TEXTURE_BUFFER;
			case SHADER_STORAGE_BUFFER:
				return GL_SHADER_STORAGE_BUFFER;
			case TRANSFORM_FEEDBACK_BUFFER:
				return GL_TRANSFORM_FEEDBACK_BUFFER;
			case COPY_READ_BUFFER:
				return GL_COPY_READ_BUFFER;
			case COPY_WRITE_BUFFER:
				return GL_COPY_WRITE_BUFFER;
			case DRAW_INDIRECT_BUFFER:
				return GL_DRAW_INDIRECT_BUFFER;
			case ATOMIC_COUNTER_BUFFER:
				return GL_ATOMIC_COUNTER_BUFFER;
			case DISPATCH_INDIRECT_BUFFER:
				return GL_DISPATCH_INDIRECT_BUFFER;
			case TARGET_LAST:
				break;
		}
		return GL_ARRAY_BUFFER;
	}

	std::ostream &operator<<(std::ostream &out, const BufferTarget &mode) {
		switch (mode) {
			case TARGET_LAST:
			case ARRAY_BUFFER:
				return out << "ARRAY_BUFFER";
			case ELEMENT_ARRAY_BUFFER:
				return out << "ELEMENT_ARRAY_BUFFER";
			case PIXEL_PACK_BUFFER:
				return out << "PIXEL_PACK_BUFFER";
			case PIXEL_UNPACK_BUFFER:
				return out << "PIXEL_UNPACK_BUFFER";
			case UNIFORM_BUFFER:
				return out << "UNIFORM_BUFFER";
			case TEXTURE_BUFFER:
				return out << "TEXTURE_BUFFER";
			case SHADER_STORAGE_BUFFER:
				return out << "SHADER_STORAGE_BUFFER";
			case TRANSFORM_FEEDBACK_BUFFER:
				return out << "TRANSFORM_FEEDBACK_BUFFER";
			case COPY_READ_BUFFER:
				return out << "COPY_READ_BUFFER";
			case COPY_WRITE_BUFFER:
				return out << "COPY_WRITE_BUFFER";
			case DRAW_INDIRECT_BUFFER:
				return out << "DRAW_INDIRECT_BUFFER";
			case ATOMIC_COUNTER_BUFFER:
				return out << "ATOMIC_COUNTER_BUFFER";
			case DISPATCH_INDIRECT_BUFFER:
				return out << "DISPATCH_INDIRECT_BUFFER";
		}
		return out;
	}

	std::istream &operator>>(std::istream &in, BufferTarget &mode) {
		std::string val;
		in >> val;
		boost::to_upper(val);
		if (val == "ARRAY_BUFFER") mode = ARRAY_BUFFER;
		else if (val == "ELEMENT_ARRAY_BUFFER") mode = ELEMENT_ARRAY_BUFFER;
		else if (val == "PIXEL_PACK_BUFFER") mode = PIXEL_PACK_BUFFER;
		else if (val == "PIXEL_UNPACK_BUFFER") mode = PIXEL_UNPACK_BUFFER;
		else if (val == "UNIFORM_BUFFER") mode = UNIFORM_BUFFER;
		else if (val == "TEXTURE_BUFFER") mode = TEXTURE_BUFFER;
		else if (val == "SHADER_STORAGE_BUFFER") mode = SHADER_STORAGE_BUFFER;
		else if (val == "TRANSFORM_FEEDBACK_BUFFER") mode = TRANSFORM_FEEDBACK_BUFFER;
		else if (val == "COPY_READ_BUFFER") mode = COPY_READ_BUFFER;
		else if (val == "COPY_WRITE_BUFFER") mode = COPY_WRITE_BUFFER;
		else if (val == "DRAW_INDIRECT_BUFFER") mode = DRAW_INDIRECT_BUFFER;
		else if (val == "ATOMIC_COUNTER_BUFFER") mode = ATOMIC_COUNTER_BUFFER;
		else if (val == "DISPATCH_INDIRECT_BUFFER") mode = DISPATCH_INDIRECT_BUFFER;
		else {
			REGEN_WARN("Unknown VBO target '" << val << "'. Using default ARRAY_BUFFER.");
			mode = ARRAY_BUFFER;
		}
		return in;
	}
}
