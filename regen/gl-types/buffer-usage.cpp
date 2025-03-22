#include "buffer-usage.h"

using namespace regen;

namespace regen {
	GLenum glBufferUsage(BufferUsage usage) {
		switch (usage) {
			case BufferUsage::USAGE_DYNAMIC:
				return GL_DYNAMIC_DRAW;
			case BufferUsage::USAGE_STATIC:
				return GL_STATIC_DRAW;
			case BufferUsage::USAGE_STREAM:
				return GL_STREAM_DRAW;
			case BufferUsage::USAGE_LAST:
				return GL_DYNAMIC_DRAW;
		}
		return GL_DYNAMIC_DRAW;
	}

	std::ostream &operator<<(std::ostream &out, const BufferUsage &mode) {
		switch (mode) {
			case BufferUsage::USAGE_DYNAMIC:
				return out << "DYNAMIC";
			case BufferUsage::USAGE_STATIC:
				return out << "STATIC";
			case BufferUsage::USAGE_STREAM:
				return out << "STREAM";
			case BufferUsage::USAGE_LAST:
				return out << "DYNAMIC";
		}
		return out;
	}

	std::istream &operator>>(std::istream &in, BufferUsage &mode) {
		std::string val;
		in >> val;
		boost::to_upper(val);
		if (val == "DYNAMIC") mode = BufferUsage::USAGE_DYNAMIC;
		else if (val == "STATIC") mode = BufferUsage::USAGE_STATIC;
		else if (val == "STREAM") mode = BufferUsage::USAGE_STREAM;
		else {
			REGEN_WARN("Unknown buffer usage '" << val << "'. Using default USAGE_DYNAMIC.");
			mode = BufferUsage::USAGE_DYNAMIC;
		}
		return in;
	}
}
