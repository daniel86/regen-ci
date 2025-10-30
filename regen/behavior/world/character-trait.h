#ifndef REGEN_MODEL_TRAITS_H_
#define REGEN_MODEL_TRAITS_H_

#include <iostream>

namespace regen {
	/**
	 * The personality traits of a character.
	 * Each trait is a normalized value between 0.0 and 1.0.
	 */
	enum class Trait {
		// The (normalized) laziness (1 = very lazy, 0 = not at all).
		LAZINESS = 0,
		// The (normalized) spirituality (1 = very spiritual, 0 = not at all).
		SPIRITUALITY,
		// The (normalized) alertness (1 = very alert, 0 = not at all).
		ALERTNESS,
		// The (normalized) bravery (1 = very brave, 0 = not at all).
		BRAVERY,
		// The (normalized) sociability (1 = very sociable, 0 = not at all).
		SOCIALABILITY,
		LAST_TRAIT
	};

	std::ostream &operator<<(std::ostream &out, const Trait &v);

	std::istream &operator>>(std::istream &in, Trait &v);
} // namespace

#endif /* REGEN_MODEL_TRAITS_H_ */
