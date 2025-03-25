#ifndef REGEN_TEXTURE_FILE_H_
#define REGEN_TEXTURE_FILE_H_

#include <string>
#include <utility>

namespace regen {
	struct TextureFile {
		TextureFile(std::string filePath, std::string fileName)
				: filePath(std::move(filePath)), fileName(std::move(fileName)) {}
		explicit TextureFile(std::string fileName)
				: fileName(std::move(fileName)) {}
		std::string filePath;
		std::string fileName;
	};
} // namespace

#endif /* REGEN_TEXTURE_FILE_H_ */
