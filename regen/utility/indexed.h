#ifndef REGEN_INDEXED_H
#define REGEN_INDEXED_H

#include <cstdint>

namespace regen {
	template <typename T> class Indexed {
	public:
		T value;
		uint32_t index;
		Indexed() : value(), index(0) {}
		Indexed(const T &v, uint32_t i) : value(v), index(i) {}
		bool operator==(const Indexed<T> &other) const {
			return value == other.value && index == other.index;
		}
		bool operator!=(const Indexed<T> &other) const {
			return !(*this == other);
		}
	};
} // knowrob

#endif //REGEN_INDEXED_H
