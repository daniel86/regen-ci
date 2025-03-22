#ifndef REGEN_BUFFER_TARGET_H_
#define REGEN_BUFFER_TARGET_H_

#include <regen/gl-types/gl-object.h>

namespace regen {
	/**
	 * \brief Buffer target.
	 */
	enum BufferTarget {
		ARRAY_BUFFER = 0,
		ATOMIC_COUNTER_BUFFER,
		COPY_READ_BUFFER,
		COPY_WRITE_BUFFER,
		DISPATCH_INDIRECT_BUFFER,
		DRAW_INDIRECT_BUFFER,
		ELEMENT_ARRAY_BUFFER,
		PIXEL_PACK_BUFFER,
		PIXEL_UNPACK_BUFFER,
		SHADER_STORAGE_BUFFER,
		TEXTURE_BUFFER,
		TRANSFORM_FEEDBACK_BUFFER,
		UNIFORM_BUFFER,
		TARGET_LAST
	};

	/**
	 * @param target the buffer target.
	 * @return the OpenGL buffer target.
	 */
	GLenum glBufferTarget(BufferTarget target);

	/**
	 * @param out the output stream.
	 * @param v the buffer target.
	 * @return the output stream.
	 */
	std::ostream &operator<<(std::ostream &out, const BufferTarget &v);

	/**
	 * @param in the input stream.
	 * @param v the buffer target.
	 * @return the input stream.
	 */
	std::istream &operator>>(std::istream &in, BufferTarget &v);
} // namespace

#endif /* REGEN_BUFFER_TARGET_H_ */
