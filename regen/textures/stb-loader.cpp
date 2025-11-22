#include "stb-loader.h"
#ifdef HAS_STB
#include <stb/stb_image.h>
#include <stb/stb_image_resize.h>

using namespace regen;

STBLoader::STBLoader() {
	// Flip all images vertically on load to match OpenGL's coordinate system.
	stbi_set_flip_vertically_on_load(true);
}

bool STBLoader::canLoad(std::string_view fileExt) const {
	if (fileExt == ".png"
		|| fileExt == ".jpg"
		|| fileExt == ".jpeg"
		|| fileExt == ".bmp"
		|| fileExt == ".tga"
		|| fileExt == ".gif"
		|| fileExt == ".psd"
		|| fileExt == ".pic") {
		return true;
	}
	return false;
}

ImageDataArray STBLoader::load(std::string_view file, const TextureConfig& cfg) {
    ImageDataArray result;

    int width, height, channels;
    int desiredChannels = 0; // let stb choose by default

    // Map forcedFormat to stb desired channels
    switch (cfg.forcedFormat) {
        case GL_RED:  desiredChannels = 1; break;
        case GL_RG:   desiredChannels = 2; break;
        case GL_RGB:  desiredChannels = 3; break;
        case GL_RGBA: desiredChannels = 4; break;
        default:      desiredChannels = 0; break; // auto-detect
    }

    stbi_uc* data = stbi_load(std::string(file).c_str(), &width, &height, &channels, desiredChannels);
    if (!data) {
        throw textures::Error(std::string("STBLoader: failed to load image: ") + file.data());
    }

    // If stb auto-detected, use channels it returned
    int actualChannels = (desiredChannels > 0) ? desiredChannels : channels;

    auto img = ref_ptr<ImageData>::alloc();
    img->width     = width;
    img->height    = height;
    img->depth     = 1;
    img->bpp       = actualChannels;
    img->pixelType = GL_UNSIGNED_BYTE;

    // Choose OpenGL format based on channel count (or forcedFormat if valid)
    if (cfg.forcedFormat != GL_NONE) {
        img->format = cfg.forcedFormat;
    } else {
        switch (actualChannels) {
            case 1: img->format = GL_RED;  break;
            case 2: img->format = GL_RG;   break;
            case 3: img->format = GL_RGB;  break;
            case 4: img->format = GL_RGBA; break;
            default:
                stbi_image_free(data);
                throw textures::Error("STBLoader: Unsupported channel count");
        }
    }

    size_t numBytes = static_cast<size_t>(width) * height * img->bpp;
    img->pixels = new byte[numBytes];
    std::memcpy(img->pixels, data, numBytes);

	const int forcedWidth = static_cast<int>(cfg.forcedSize.x);
	const int forcedHeight = static_cast<int>(cfg.forcedSize.y);

    // Optional: scale if forcedSize is set
    if (forcedWidth > 0 && forcedHeight > 0 &&
        (forcedWidth != width || forcedHeight != height)) {

        size_t resizedBytes = forcedWidth * forcedHeight * img->bpp;
        byte* resizedPixels = new byte[resizedBytes];

        stbir_resize_uint8(
            img->pixels, width, height, 0,
            resizedPixels, forcedWidth, forcedHeight, 0,
            img->bpp);

        delete[] img->pixels;
        img->pixels = resizedPixels;
        img->width  = forcedWidth;
        img->height = forcedHeight;
    }

    result.push_back(img);
    stbi_image_free(data);

    return result;
}
#endif
