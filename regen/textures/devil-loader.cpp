#include "devil-loader.h"
#include <IL/il.h>
#include <IL/ilu.h>
#include <boost/filesystem.hpp>

using namespace regen;

bool DevilLoader::canLoad(std::string_view fileExt) const {
	return true; // all formats supported by DevIL
}

void DevilLoader::scaleImage(uint32_t w, uint32_t h, uint32_t d) {
	uint32_t width_ = ilGetInteger(IL_IMAGE_WIDTH);
	// scale image to desired size
	if (w > 0 && h > 0) {
		if (width_ > w) {
			// use bilinear filtering for down scaling
			iluImageParameter(ILU_FILTER, ILU_BILINEAR);
		} else {
			// use triangle filtering for up scaling
			iluImageParameter(ILU_FILTER, ILU_SCALE_TRIANGLE);
		}
		iluScale(w, h, d);
	}
}

GLenum regenImageFormat1() {
	GLenum format = ilGetInteger(IL_IMAGE_FORMAT);
	return (format == GL_COLOR_INDEX ? GL_RGBA : format);
}

GLenum regenImageFormat() {
	GLenum format = ilGetInteger(IL_IMAGE_FORMAT);
	switch (format) {
		// handle deprecated formats
		case GL_LUMINANCE:
			return GL_RED;
		case GL_LUMINANCE_ALPHA:
			return GL_RG;
		case GL_COLOR_INDEX:
			return GL_RGBA;
		case GL_BGR:
			return GL_RGB;
		case GL_BGRA:
			return GL_RGBA;
		default:
			return format;
	}
}

GLenum DevilLoader::convertImage(GLenum format, GLenum type) {
	auto srcFormat = regenImageFormat1();
	auto srcType = static_cast<uint32_t>(ilGetInteger(IL_IMAGE_TYPE));
	auto origSrcFormat = static_cast<uint32_t>(ilGetInteger(IL_IMAGE_FORMAT));
	auto dstFormat = (format == GL_NONE ? srcFormat : format);
	auto dstType = (type == GL_NONE ? srcType : type);
	if (origSrcFormat != dstFormat || srcType != dstType) {
		if (ilConvertImage(dstFormat, dstType) == IL_FALSE) {
			throw textures::Error(REGEN_STRING("ilConvertImage failed between format 0x"
											   << std::hex << origSrcFormat << " to 0x" << dstFormat
											   << " and type 0x" << srcType << " to 0x" << dstType << std::dec));
		}
	}
	return dstFormat;
}

void DevilLoader::save(const ref_ptr<ImageData> &img, std::string_view file) {
	static bool devilInitialized = false;
	if (!devilInitialized) {
		devilInitialized = true;
		ilInit();
	}
	// remove existing file
	if (boost::filesystem::exists(file)) {
		boost::filesystem::remove(file);
	}
	uint32_t ilID;
	ilGenImages(1, &ilID);
	ilBindImage(ilID);
	REGEN_INFO("Saving image to '" << file << "'"
		<< " width=" << img->width
		<< " height=" << img->height
		<< " depth=" << img->depth
		<< " bpp=" << img->bpp
		<< " format=0x" << std::hex << img->format << std::dec
		<< " type=0x" << std::hex << img->pixelType << std::dec);
	auto format = img->format;
	if (format == GL_RED) {
		format = GL_LUMINANCE;
	}
	ilTexImage(img->width, img->height, img->depth, img->bpp, format, img->pixelType, img->pixels);
	if (ilSaveImage(file.data()) == IL_FALSE) {
		ilDeleteImages(1, &ilID);
		throw textures::Error(REGEN_STRING("ilSaveImage failed for file '"
			<< file << "': 0x" << std::hex << ilGetError() << std::dec));
	}
	ilDeleteImages(1, &ilID);
}

ImageDataArray DevilLoader::load(std::string_view file, const TextureConfig &cfg) {
	static bool devilInitialized = false;
	if (!devilInitialized) {
		devilInitialized = true;
		ilInit();
		// Change origin to lower left to match OpenGL texture coordinate system.
		ilEnable(IL_ORIGIN_SET);
		ilOriginFunc(IL_ORIGIN_LOWER_LEFT);
	}
	if (!boost::filesystem::exists(file)) {
		throw textures::Error(REGEN_STRING("Unable to open image file at '" << file << "'."));
	}

	uint32_t ilID;
	ilGenImages(1, &ilID);
	ilBindImage(ilID);
	if (ilLoadImage(file.data()) == IL_FALSE) {
		ilDeleteImages(1, &ilID);
		throw textures::Error("ilLoadImage failed");
	}
	scaleImage(cfg.forcedSize.x, cfg.forcedSize.y, cfg.forcedSize.z);
	auto convertedFormat = convertImage(cfg.forcedFormat, GL_NONE);
	const uint32_t numImages = ilGetInteger(IL_NUM_IMAGES);

	ImageDataArray output;
	auto &first = output.emplace_back(ref_ptr<ImageData>::alloc());
	first->width = ilGetInteger(IL_IMAGE_WIDTH);
	first->height = ilGetInteger(IL_IMAGE_HEIGHT);
	first->depth = ilGetInteger(IL_IMAGE_DEPTH);
	first->bpp = ilGetInteger(IL_IMAGE_BPP);
	first->format = convertedFormat;
	first->pixelType = ilGetInteger(IL_IMAGE_TYPE);

	size_t numBytes = first->width * first->height * first->bpp;
	first->pixels = new byte[numBytes];
	std::memcpy(first->pixels, ilGetData(), numBytes);

	for (uint32_t i = 1; i < numImages; ++i) {
		ilBindImage(ilID);
		ilActiveImage(i);
		auto &img = output.emplace_back(ref_ptr<ImageData>::alloc());
		img->width = first->width;
		img->height = first->height;
		img->depth = first->depth;
		img->bpp = first->bpp;
		img->format = first->format;
		img->pixelType = first->pixelType;
		img->pixels = new byte[numBytes];
		std::memcpy(img->pixels, ilGetData(), numBytes);
	}
	ilDeleteImages(1, &ilID);

	REGEN_DEBUG("Texture '" << file << "'" <<
							" type=" << ilGetInteger(IL_IMAGE_TYPE) <<
							" channels=" << ilGetInteger(IL_IMAGE_CHANNELS) <<
							" format=" << first->format <<
							" bpp=" << first->bpp <<
							" images=" << numImages <<
							" width=" << first->width <<
							" height=" << first->height);

	return output;
}
