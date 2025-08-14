#ifndef REGEN_TEXTURE_FILE_H_
#define REGEN_TEXTURE_FILE_H_

#include <string>
#include <utility>

namespace regen {
	struct TextureFile {
		TextureFile(std::string_view filePath, std::string_view fileName)
				: filePath(filePath), fileName(fileName) {}
		explicit TextureFile(std::string_view fileName)
				: fileName(fileName) {}
		std::string filePath;
		std::string fileName;
	};
} // namespace

#endif /* REGEN_TEXTURE_FILE_H_ */
