#include "atom.h"

using namespace regen;

namespace regen {
	class concrete_Atom : public Atom {
	public:
		explicit concrete_Atom(std::string_view v) : Atom(v) {}
	};
}

Atom::AtomTable &Atom::table() {
	static Atom::AtomTable theTable;
	return theTable;
}

std::shared_ptr<Atom> Atom::Tabled(std::string_view stringForm) {
	auto it = table().find(stringForm);
	if (it != table().end()) {
		if (auto atomPtr = it->second.value().lock()) {
			// Atom still exists, return it
			return atomPtr;
		}
		table().erase(it);
	}
	// Atom does not exist or was destroyed, create a new one
	auto inserted = table().emplace(stringForm, std::nullopt);
	auto &jt = inserted.first;
	auto atom = std::make_shared<concrete_Atom>(jt->first);
	jt->second = atom;
	auto locked = jt->second.value().lock();
	if (!locked) {
		throw std::runtime_error("Failed to lock Atom");
	}
	return locked;
}