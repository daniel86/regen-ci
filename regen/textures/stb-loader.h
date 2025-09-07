#ifndef REGEN_STB_LOADER_H_
#define REGEN_STB_LOADER_H_

#include <regen/config.h>
#ifdef HAS_STB
#include "texture-loader.h"

namespace regen {
	/**
	 * \brief Texture loader using the stb_image library.
	 *
	 * This class implements a texture loader using the stb_image library.
	 * It supports a wide range of image formats, including JPEG, PNG, BMP, TGA, and more.
	 * It can also handle image scaling and format conversion.
	 */
	class STBLoader : public ITextureLoader {
	public:
		STBLoader() = default;

		~STBLoader() override = default;

		bool canLoad(std::string_view fileExt) const override;

		ImageDataArray load(std::string_view file, const TextureConfig& cfg) override;
	};
} // namespace
#endif // HAS_STB

#endif /* REGEN_STB_LOADER_H_ */
