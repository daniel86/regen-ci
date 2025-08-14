#ifndef REGEN_ATOM_H
#define REGEN_ATOM_H

#include <optional>
#include <string_view>
#include <map>
#include <memory>

namespace regen {
	/**
	 * An atom is an atomic term that represents a constant.
	 * This is thought as an replacement of string, where the string data
	 * is globally allocated and strings are immutable.
	 * The main advantage is that we can do string equality checking by comparison
	 * of pointers, also atoms can safely be used as keys in maps without copy of string
	 * data and without risk of memory corruption.
	 */
	class Atom {
	public:
		/**
		 * @param stringForm the string form of the atom
		 * @return a shared pointer to an atom
		 */
		static std::shared_ptr<Atom> Tabled(std::string_view stringForm);

		/**
		 * @param other another atom
		 * @return true if both atoms are equal
		 */
		bool isSameAtom(const Atom &other) const { return this == &other; }

		/**
		 * @return the string form of this atom.
		 */
		std::string_view stringForm() const { return stringForm_; }

	protected:
		std::string_view stringForm_;

		using AtomTable = std::map<std::string, std::optional<std::weak_ptr<Atom>>, std::less<>>;

		static AtomTable &table();

		explicit Atom(std::string_view stringForm) : stringForm_(stringForm) {}
	};

	using AtomPtr = std::shared_ptr<Atom>;

} // knowrob

#endif //REGEN_ATOM_H