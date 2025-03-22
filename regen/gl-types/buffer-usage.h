#ifndef REGEN_BUFFER_USAGE_H_
#define REGEN_BUFFER_USAGE_H_

#include <regen/gl-types/gl-object.h>

namespace regen {
	/**
	 * \brief Flag indicating the usage of the data in the VBO
	 */
	enum BufferUsage {
		USAGE_DYNAMIC = 0,
		USAGE_STATIC,
		USAGE_STREAM,
		USAGE_LAST
	};

	/**
	 * @param usage the buffer usage.
	 * @return the OpenGL buffer usage.
	 */
	GLenum glBufferUsage(BufferUsage usage);

	std::ostream &operator<<(std::ostream &out, const BufferUsage &v);

	std::istream &operator>>(std::istream &in, BufferUsage &v);
} // namespace

#endif /* REGEN_BUFFER_USAGE_H_ */
