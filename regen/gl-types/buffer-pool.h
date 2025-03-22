#ifndef REGEN_BUFFER_POOL_H_
#define REGEN_BUFFER_POOL_H_

#include <regen/utility/memory-allocator.h>

namespace regen {
	/**
	 * \brief Interface for allocating GPU memory.
	 */
	struct BufferAllocator {
		/**
		 * Generate buffer object name and
		 * create and initialize the buffer object's data store to the named size.
		 */
		static GLuint createAllocator(GLuint poolIndex, GLuint size);

		/**
		 * Delete named buffer object.
		 * After a buffer object is deleted, it has no contents,
		 * and its name is free for reuse.
		 */
		static void deleteAllocator(GLuint poolIndex, GLuint ref);
	};

	/**
	 * \brief A pool of VBO memory allocators.
	 */
	typedef AllocatorPool<
			BufferAllocator, GLuint,   // actual allocator and reference type
			BuddyAllocator, GLuint     // virtual allocator and reference type
	> BufferPool;
} // namespace

#endif /* REGEN_BUFFER_POOL_H_ */
