#include "client-allocator.h"
#include "regen/regen.h"

using namespace regen;

ClientBufferRef ClientBufferAllocator::createAllocator(GLuint /*poolIdx*/, GLuint size) {
	return (byte*)std::malloc(size);
}

void ClientBufferAllocator::deleteAllocator(GLuint /*poolIndex*/, ClientBufferRef ref) {
	if (!ref) return;
	std::free(ref);
}

void* ClientBufferAllocator::mapAllocator(GLuint /*poolIdx*/, GLuint /*size*/, ClientBufferRef ref) {
	return ref;
}

void ClientBufferAllocator::unmapAllocator(GLuint /*poolIdx*/, ClientBufferRef /*ref*/) {
	// nothing to do here.
}

void ClientBufferAllocator::orphanAllocatorRange(ClientBufferRef /*ref*/, GLuint /*offset*/, GLuint /*size*/) {
	// nothing to do here.
}
