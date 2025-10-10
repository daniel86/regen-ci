#ifndef REGEN_MODEL_CHARACTER_ATTRIBUTE_H_
#define REGEN_MODEL_CHARACTER_ATTRIBUTE_H_

#include <iostream>

namespace regen {
	/**
	 * The attributes of a character.
	 * Each attribute is a normalized value between 0.0 and 1.0.
	 */
	enum class CharacterAttribute {
		// The normalized health of the character, from 0.0 (dead) to 1.0 (full health).
		HEALTH = 0,
		// The normalized hunger of the character, from 0.0 (full) to 1.0 (starving).
		HUNGER,
		// The normalized stamina of the character, from 0.0 (exhausted) to 1.0 (full stamina).
		STAMINA,
		// The normalized fear of the character, from 0.0 (no fear) to 1.0 (terrified).
		FEAR,
		LAST
	};

	std::ostream &operator<<(std::ostream &out, const CharacterAttribute &v);

	std::istream &operator>>(std::istream &in, CharacterAttribute &v);
} // namespace

#endif /* REGEN_MODEL_CHARACTER_ATTRIBUTE_H_ */
