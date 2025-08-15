#ifndef REGEN_TEXTURE_LOADER_H_
#define REGEN_TEXTURE_LOADER_H_

#include <stdexcept>

#include <regen/textures/texture.h>
#include <regen/utility/ref-ptr.h>
#include <regen/math/vector.h>
#include "texture-config.h"

namespace regen {
	namespace textures {
		/**
		 * \brief An error occurred loading the Texture.
		 */
		class Error : public std::runtime_error {
		public:
			/**
			 * @param message the error message.
			 */
			explicit Error(const std::string &message) : std::runtime_error(message) {}
		};

		uint32_t loadImage(std::string_view file);

		/**
		 * Load a Texture from file. Guess if it is a Texture2D or Texture3D.
		 * Force specified internal format.
		 * Scale to forced size (if forced size != 0).
		 * Setup mipmapping after loading the file.
		 */
		ref_ptr<Texture> load(std::string_view file, const TextureConfig &texCfg = TextureConfig::getDefault());

		/**
		 * Update a Texture from file.
		 * @param tex the Texture to update.
		 * @param file the file to load.
		 */
		void reload(const ref_ptr<Texture> &tex, std::string_view file);

		/**
		 * Load a Texture from RAW data. Guess if it is a Texture2D or Texture3D.
		 * Force specified internal format.
		 * Scale to forced size (if forced size != 0).
		 * Setup mipmapping after loading the file.
		 */
		ref_ptr<Texture> load(
				uint32_t textureType,
				uint32_t numBytes,
				const void *rawData,
				const TextureConfig &texCfg = TextureConfig::getDefault());

		/**
		 * Load a Texture2DArray from file.
		 * Force specified internal format.
		 * Scale to forced size (if forced size != 0).
		 * Setup mipmapping after loading the file.
		 */
		ref_ptr<Texture2DArray> loadArray(
				const std::string &textureDirectory,
				const std::string &textureNamePattern,
				const TextureConfig &texCfg = TextureConfig::getDefault());

		/**
		 * Load a Texture2DArray from file.
		 * Force specified internal format.
		 * Scale to forced size (if forced size != 0).
		 * Setup mipmapping after loading the file.
		 */
		ref_ptr<Texture2DArray> loadArray(
				const std::vector<TextureDescription> &textureDescriptions,
				const TextureConfig &texCfg = TextureConfig::getDefault());

		/**
		 * Load a TextureCube from file.
		 * The file is expected to be a regular 2D image containing
		 * multiple faces arranged next to each other.
		 * Force specified internal format.
		 * Scale to forced size (if forced size != 0).
		 * Setup mipmapping after loading the file.
		 */
		ref_ptr<TextureCube> loadCube(
				std::string_view file,
				bool flipBackFace = false,
				const TextureConfig &texCfg = TextureConfig::getDefault());

		/**
		 * Loads RAW Texture from file.
		 */
		ref_ptr<Texture> loadRAW(
				const std::string &path,
				const Vec3ui &size,
				uint32_t numComponents,
				uint32_t bytesPerComponent);

		/**
		 * 1D texture that contains a color spectrum.
		 */
		ref_ptr<Texture> loadSpectrum(
				double t1,
				double t2,
				int numTexels,
				GLenum mipmapFlag = GL_DONT_CARE);
	}
} // namespace

#endif /* REGEN_TEXTURE_LOADER_H_ */
