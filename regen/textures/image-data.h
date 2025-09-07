#ifndef REGEN_IMAGE_DATA_H_
#define REGEN_IMAGE_DATA_H_

#include <vector>
#include <GL/glew.h>

namespace regen {
	/**
	 * \brief Image data structure.
	 *
	 * This structure holds the image data loaded from a file.
	 * It contains the width, height, depth, bytes per pixel, format, pixel type, and the pixel data itself.
	 * The pixel data is stored as a contiguous array of bytes.
	 */
	struct ImageData {
		ImageData() = default;
		~ImageData() {
			delete[] pixels;
		}
		ImageData(const ImageData &) = delete;
		int width = 0;
		int height = 0;
		int depth = 1;
		int bpp = 4; // bytes per pixel
		GLenum format = GL_RGBA;
		GLenum pixelType = GL_UNSIGNED_BYTE;
		GLenum wrapping = GL_CLAMP_TO_EDGE;
		byte *pixels = nullptr; // contiguous image data
	};

	/** \brief Array of ImageData pointers. */
	using ImageDataArray = std::vector<ref_ptr<ImageData>>;
}

#endif /* REGEN_IMAGE_DATA_H_ */
