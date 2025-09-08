#ifndef REGEN_DEVIL_LOADER_H_
#define REGEN_DEVIL_LOADER_H_

#include "texture-loader.h"

namespace regen {
	/**
	 * \brief Texture loader using the DevIL library.
	 *
	 * This class implements a texture loader using the DevIL library.
	 * It supports a wide range of image formats, including JPEG, PNG, BMP, TGA, and more.
	 * It can also handle image scaling and format conversion.
	 */
	class DevilLoader : public ITextureLoader {
	public:
		DevilLoader() = default;

		~DevilLoader() override = default;

		bool canLoad(std::string_view fileExt) const override;

		ImageDataArray load(std::string_view file, const TextureConfig& cfg) override;

		/**
		 * Scale the currently bound image to the given size.
		 * If w or h is 0, the image will not be scaled in that dimension.
		 */
		static void scaleImage(GLuint w, GLuint h, GLuint d);

		/**
		 * Convert the currently bound image to the given format and type.
		 * If format or type is GL_NONE, the image will not be converted in that dimension.
		 * @return the actual format of the image after conversion.
		 */
		static GLenum convertImage(GLenum format, GLenum type);
	};
} // namespace

#endif /* REGEN_DEVIL_LOADER_H_ */
