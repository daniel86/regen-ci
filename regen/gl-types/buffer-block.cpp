#include "buffer-block.h"
#include "ubo.h"
#include "ssbo.h"
#include "regen/states/state.h"
#include "regen/scene/shader-input-processor.h"

using namespace regen;

BufferBlock::BufferBlock(
		BufferTarget target,
		BufferUsage usage,
		StorageQualifier storageQualifier,
		MemoryLayout memoryLayout)
		: BufferObject(target, usage),
		  storageQualifier_(storageQualifier),
		  memoryLayout_(memoryLayout) {
}

void BufferBlock::addBlockInput(const ref_ptr<ShaderInput> &input, const std::string &name) {
	auto &uboInput = blockInputs_.emplace_back();
	uboInput.input = input;
	uboInput.offset = requiredSize_;
	uboInput.lastStamp = 0;
	inputs_.emplace_back(input, name);
	requiredSize_ += input->inputSize();
	requiresResize_ = true;
}

void BufferBlock::updateBlockInput(const ref_ptr<ShaderInput> &input) {
	for (auto &uboInput: blockInputs_) {
		if (uboInput.input.get() == input.get()) {
			uboInput.lastStamp = 0;
			requiresResize_ = true;
			return;
		}
	}
}

bool BufferBlock::needsUpdate() const {
	if (requiresResize_) { return true; }
	for (auto &uboInput: blockInputs_) {
		if (uboInput.input->stamp() != uboInput.lastStamp) {
			return true;
		}
	}
	return false;
}

void BufferBlock::computePaddedSize() {
	requiredSize_ = 0;
	for (auto &uboInput: blockInputs_) {
		auto &uniform = uboInput.input;
		// note we need to compute the "aligned" offset for std140 layout
		GLuint numElements = uniform->numArrayElements() * uniform->numInstances();
		GLuint baseSize = uniform->inputSize() / numElements;

		// Compute the alignment based on the type
		GLuint baseAlignment = baseSize;
		GLuint alignmentCount = 1;
		if (baseSize == 12) { // vec3
			baseAlignment = 16;
		} else if (baseSize == 48) { // mat3
			baseAlignment = 16;
			alignmentCount = 3;
		} else if (baseSize == 64) { // mat4
			baseAlignment = 16;
			alignmentCount = 4;
		} else if (numElements > 1) {
			if (memoryLayout_ == MemoryLayout::STD140) {
				// NOTE: STD430 does not require alignment for arrays
				baseAlignment = 16;
			}
		}

		// Align the offset to the required alignment
		if (requiredSize_ % baseAlignment != 0) {
			requiredSize_ += baseAlignment - (requiredSize_ % baseAlignment);
		}
		uboInput.offset = requiredSize_;
		if (numElements > 1) {
			requiredSize_ += baseAlignment * alignmentCount * numElements;
		} else {
			requiredSize_ += uniform->inputSize();
		}
	}
}

void BufferBlock::updateAlignedData(BlockInput &uboInput) {
	// the GL specification states that the stride between array elements must be
	// rounded up to 16 bytes.
	// this is quite ugly to do element-wise memcpy below, hence non 16-byte aligned
	// arrays should be avoided in general.
	auto &in = uboInput.input;
	auto numElements = in->numArrayElements() * in->numInstances();
	if (numElements == 1 || memoryLayout_ == MemoryLayout::STD430) {
		return;
	}
	auto elementSizeUnaligned = in->valsPerElement() * in->dataTypeBytes();
	if (elementSizeUnaligned % 16 == 0) {
		return;
	}
	auto elementSizeAligned = elementSizeUnaligned + (16 - elementSizeUnaligned % 16);
	auto dataSizeAligned = elementSizeAligned * numElements;
	if (dataSizeAligned != uboInput.alignedSize) {
		delete[] uboInput.alignedData;
		uboInput.alignedSize = dataSizeAligned;
		uboInput.alignedData = new byte[uboInput.alignedSize];
	}
	auto clientData = in->mapClientDataRaw(ShaderData::READ);
	auto *src = clientData.r;
	auto *dst = uboInput.alignedData;
	for (unsigned int i = 0; i < numElements; ++i) {
		memcpy(dst, src, elementSizeUnaligned);
		src += elementSizeUnaligned;
		dst += elementSizeAligned;
	}
}

void BufferBlock::update(bool forceUpdate) {
	bool needUpdate = forceUpdate || needsUpdate();
	if (!needUpdate) { return; }
	std::unique_lock<std::mutex> lock(mutex_);

	if (requiresResize_) {
		computePaddedSize();
		ref_ = allocBytes(requiredSize_);
		allocatedSize_ = requiredSize_;
		forceUpdate = GL_TRUE;
		requiresResize_ = GL_FALSE;
		GL_ERROR_LOG();
	}

	glBindBuffer(glTarget_, ref_->bufferID());
	void *bufferData = map(ref_, GL_MAP_WRITE_BIT);
	if (bufferData) {
		for (auto &uboInput: blockInputs_) {
			if (forceUpdate || uboInput.input->stamp() != uboInput.lastStamp) {
				if (!uboInput.input->hasClientData()) {
					continue;
				}
				// copy the data to the buffer.
				updateAlignedData(uboInput);
				if (uboInput.alignedData) {
					memcpy(static_cast<char *>(bufferData) + uboInput.offset,
						   uboInput.alignedData, uboInput.alignedSize);
				} else {
					auto mapped = uboInput.input->mapClientDataRaw(ShaderData::READ);
					memcpy(static_cast<char *>(bufferData) + uboInput.offset,
						   mapped.r,
						   uboInput.input->inputSize());
				}
				uboInput.lastStamp = uboInput.input->stamp();
			}
		}
		unmap();
	} else {
		REGEN_WARN("BufferBlock::update: failed to map buffer");
	}
	glBindBuffer(glTarget_, 0);
	stamp_ += 1;
}

void BufferBlock::enableBufferBlock(GLint loc) {
	update();
	bind(loc);
}

ref_ptr<BufferBlock> BufferBlock::load(LoadingContext &ctx, scene::SceneInputNode &input) {
	auto blockType = input.getValue<std::string>("type", "ubo");
	auto usageHint = input.getValue<BufferUsage>("usage", USAGE_DYNAMIC);
	ref_ptr<BufferBlock> block;
	if (blockType == "ubo") {
		block = ref_ptr<UBO>::alloc(input.getName(), usageHint);
	} else if (blockType == "ssbo") {
		block = ref_ptr<SSBO>::alloc(input.getName(), usageHint);
	} else {
		REGEN_WARN("Unknown buffer block type '" << blockType << "'. Using default UBO.");
		block = ref_ptr<UBO>::alloc(input.getName(), usageHint);
	}
	auto dummyState = ref_ptr<State>::alloc();

	for (auto &n: input.getChildren()) {
		if (n->getCategory() == "uniform" || n->getCategory() == "input") {
			auto uniform = scene::ShaderInputProcessor::createShaderInput(
					ctx.scene(), *n.get(), dummyState);
			if (uniform->isVertexAttribute()) {
				REGEN_WARN("UBO cannot contain vertex attributes. In node '" << n->getDescription() << "'.");
				continue;
			}
			auto name = n->getValue("name");
			block->addBlockInput(uniform, name);
		} else {
			REGEN_WARN("Unknown UBO child category '" << n->getCategory() << "'.");
		}
	}
	GL_ERROR_LOG();

	return block;
}

std::ostream &regen::operator<<(std::ostream &out, const BufferBlock::MemoryLayout &v) {
	switch (v) {
		case BufferBlock::MemoryLayout::STD140:
			out << "std140";
			break;
		case BufferBlock::MemoryLayout::STD430:
			out << "std430";
			break;
		case BufferBlock::MemoryLayout::PACKED:
			out << "packed";
			break;
		case BufferBlock::MemoryLayout::SHARED:
			out << "shared";
			break;
	}
	return out;
}

std::ostream &regen::operator<<(std::ostream &out, const BufferBlock::StorageQualifier &v) {
	switch (v) {
		case BufferBlock::StorageQualifier::UNIFORM:
			out << "uniform";
			break;
		case BufferBlock::StorageQualifier::BUFFER:
			out << "buffer";
			break;
		case BufferBlock::StorageQualifier::IN:
			out << "in";
			break;
		case BufferBlock::StorageQualifier::OUT:
			out << "out";
			break;
	}
	return out;
}

std::istream &regen::operator>>(std::istream &in, BufferBlock::MemoryLayout &v) {
	std::string val;
	in >> val;
	boost::to_lower(val);
	if (val == "std140") v = BufferBlock::MemoryLayout::STD140;
	else if (val == "std430") v = BufferBlock::MemoryLayout::STD430;
	else if (val == "packed") v = BufferBlock::MemoryLayout::PACKED;
	else if (val == "shared") v = BufferBlock::MemoryLayout::SHARED;
	else {
		REGEN_WARN("Unknown memory layout '" << val << "'. Using default STD140.");
		v = BufferBlock::MemoryLayout::STD140;
	}
	return in;
}

std::istream &regen::operator>>(std::istream &in, BufferBlock::StorageQualifier &v) {
	std::string val;
	in >> val;
	boost::to_lower(val);
	if (val == "uniform") v = BufferBlock::StorageQualifier::UNIFORM;
	else if (val == "buffer") v = BufferBlock::StorageQualifier::BUFFER;
	else if (val == "in") v = BufferBlock::StorageQualifier::IN;
	else if (val == "out") v = BufferBlock::StorageQualifier::OUT;
	else {
		REGEN_WARN("Unknown storage qualifier '" << val << "'. Using default UNIFORM.");
		v = BufferBlock::StorageQualifier::UNIFORM;
	}
	return in;
}

