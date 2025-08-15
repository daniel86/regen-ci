#ifndef REGEN_CLIENT_ALLOCATOR_H_
#define REGEN_CLIENT_ALLOCATOR_H_

#include "regen/utility/memory-allocator.h"

namespace regen {
	/**
	 * \brief Reference to a client buffer.
	 * This is a pointer to the allocated memory.
	 */
	using ClientBufferRef = unsigned char*;

	/**
	 * \brief Interface for allocating CPU memory.
	 */
	struct ClientBufferAllocator {
		/**
		 * Generate buffer object name and
		 * create and initialize the buffer object's data store to the named size.
		 */
		static ClientBufferRef createAllocator(GLuint poolIndex, GLuint size);

		/**
		 * Map the buffer object to the CPU memory.
		 * The poolIndex is used to determine the storage mode.
		 * @param poolIndex the index of the pool.
		 * @param size the size of the buffer to map.
		 * @param ref the reference to the buffer object.
		 * @return pointer to mapped memory, or nullptr if mapping is not possible.
		 */
		static void* mapAllocator(GLuint poolIndex, GLuint size, ClientBufferRef ref);

		/**
		 * Unmap the buffer object from the CPU memory.
		 * The poolIndex is used to determine the storage mode.
		 * @param poolIndex the index of the pool.
		 * @param ref the reference to the buffer object.
		 */
		static void unmapAllocator(GLuint poolIndex, ClientBufferRef ref);

		/**
		 * Delete named buffer object.
		 * After a buffer object is deleted, it has no contents,
		 * and its name is free for reuse.
		 */
		static void deleteAllocator(GLuint poolIndex, ClientBufferRef ref);

		/**
		 * Invalidate a buffer range.
		 * @param ref the reference to the buffer object.
		 * @param offset the offset in the buffer object.
		 * @param size the size of the data to invalidate.
		 */
		static void orphanAllocatorRange(ClientBufferRef ref, GLuint offset, GLuint size);
	};

	/**
	 * \brief A pool of memory allocators.
	 */
	typedef AllocatorPool<
			ClientBufferAllocator,
			ClientBufferRef,   // actual allocator and reference type
			BuddyAllocator,
			GLuint     // virtual allocator and reference type
	> ClientBufferPool;
} // namespace

#endif /* REGEN_CLIENT_ALLOCATOR_H_ */
