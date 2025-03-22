#ifndef REGEN_BUFFER_BLOCK_H_
#define REGEN_BUFFER_BLOCK_H_

#include <regen/gl-types/buffer-object.h>
#include "regen/scene/scene-input.h"

namespace regen {
	/**
	 * \brief A BufferBlock is a Buffer Object that is used to store and retrieve data from within the OpenGL Shading Language.
	 *
	 * This base class is used to create Uniform Buffer Objects (UBO) and Shader Storage Buffer Objects (SSBO).
	 */
	class BufferBlock : public BufferObject {
	public:
		static constexpr const char *TYPE_NAME = "BufferBlock";

		/**
		 * Storage qualifiers for shader storage blocks.
		 */
		enum StorageQualifier {
			// Uniform block
			UNIFORM = 0,
			// Shader storage block
			BUFFER,
			// Input block
			IN,
			// Output block
			OUT
		};
		enum MemoryLayout {
			STD140 = 0,
			STD430,
			PACKED,
			SHARED
		};

		/**
		 * Create a buffer block.
		 * @param target the buffer target.
		 * @param usage the buffer usage.
		 * @param storageQualifier the storage qualifier.
		 * @param memoryLayout the memory layout.
		 */
		BufferBlock(
			BufferTarget target,
			BufferUsage usage,
			StorageQualifier storageQualifier,
			MemoryLayout memoryLayout);

		~BufferBlock() override = default;

		BufferBlock(const BufferBlock &) = delete;

		static ref_ptr<BufferBlock> load(LoadingContext &ctx, scene::SceneInputNode &input);

		/**
		 * @return true if the block is a uniform block.
		 */
		auto isUniformBlock() const { return storageQualifier_ == UNIFORM; }

		/**
		 * @return true if the block is a shader storage block.
		 */
		auto isShaderStorageBlock() const { return storageQualifier_ == BUFFER; }

		/**
		 * @return the storage qualifier of the block.
		 */
		StorageQualifier storageQualifier() const { return storageQualifier_; }

		/**
		 * @return the memory layout of the block.
		 */
		MemoryLayout memoryLayout() const { return memoryLayout_; }

		/**
		 * @return true if the block has a binding index.
		 */
		bool has_bindingIndex() const { return bindingIndex_ >= 0; }

		/**
		 * Set the binding index of the block.
		 * @param index the binding index.
		 */
		void set_bindingIndex(int index) { bindingIndex_ = index; }

		/**
		 * @return the binding index of the block.
		 */
		int bindingIndex() const { return bindingIndex_; }

		/**
		 * Add a uniform to the UBO.
		 * @param input the shader input.
		 */
		void addBlockInput(const ref_ptr<ShaderInput> &input, const std::string &name = "");

		/**
		 * @param input the shader input.
		 */
		void updateBlockInput(const ref_ptr<ShaderInput> &input);

		/**
		 * @return the list of uniforms.
		 */
		auto &blockInputs() const { return inputs_; }

		/**
		 * Update the block buffer.
		 * Should be called each frame, is a no-op if no data has changed.
		 * @param forceUpdate force update.
		 */
		void update(bool forceUpdate = false);

		/**
		 * Binds the uniform block to the given shader location.
		 */
		void enableBufferBlock(GLint loc);

		/**
		 * Lock the UBO, preventing updates.
		 */
		void lock() { mutex_.lock(); }

		/**
		 * Unlock the UBO, allowing updates.
		 */
		void unlock() { mutex_.unlock(); }

	protected:
		const StorageQualifier storageQualifier_;
		const MemoryLayout memoryLayout_;
		int bindingIndex_ = -1;
		std::mutex mutex_;

		struct BlockInput {
			BlockInput() = default;

			BlockInput(const BlockInput &other) {
				input = other.input;
				offset = other.offset;
			}

			~BlockInput() {
				if (alignedData) {
					delete[] alignedData;
				}
			}

			ref_ptr<ShaderInput> input;
			unsigned int offset = 0;
			unsigned int lastStamp = 0;
			unsigned int alignedSize = 0;
			byte *alignedData = nullptr;
		};

		std::vector<BlockInput> blockInputs_;
		std::vector<NamedShaderInput> inputs_;
		ref_ptr<BufferReference> ref_;
		unsigned int requiredSize_ = 0;
		bool requiresResize_ = false;
		unsigned int stamp_ = 0;

		bool needsUpdate() const;

		void computePaddedSize();

		void updateAlignedData(BlockInput &uboInput);
	};

	std::ostream &operator<<(std::ostream &out, const BufferBlock::MemoryLayout &v);
	std::ostream &operator<<(std::ostream &out, const BufferBlock::StorageQualifier &v);

	std::istream &operator>>(std::istream &in, BufferBlock::MemoryLayout &v);
	std::istream &operator>>(std::istream &in, BufferBlock::StorageQualifier &v);
} // namespace

#endif /* REGEN_BUFFER_BLOCK_H_ */
