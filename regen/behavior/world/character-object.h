#ifndef REGEN_CHARACTER_OBJECT_GROUP_H_
#define REGEN_CHARACTER_OBJECT_GROUP_H_
#include "world-object.h"

namespace regen {
	class Blackboard;
	class NonPlayerCharacterController;

	class CharacterObject : public WorldObject {
	public:
		explicit CharacterObject(std::string_view name) :
			WorldObject(name) {
			objectType_ = ObjectType::CHARACTER;
			isStatic_ = false;
		}
		~CharacterObject() override = default;

		/**
		 * Set the knowledge base for the character.
		 * @param kb the knowledge base.
		 */
		void setKnowledgeBase(Blackboard *kb) {
			knowledgeBase_ = kb;
		}

		/**
		 * Get the knowledge base of the character.
		 * @return the knowledge base.
		 */
		Blackboard* knowledgeBase() const {
			return knowledgeBase_;
		}

		/**
		 * Check if the character has a knowledge base.
		 * @return true if the character has a knowledge base, false otherwise.
		 */
		bool hasKnowledgeBase() const {
			return knowledgeBase_ != nullptr;
		}

		/**
		 * Set the NPC controller for the character.
		 * @param controller the NPC controller.
		 */
		void setNPCController(NonPlayerCharacterController *controller) {
			npcController_ = controller;
		}

		/**
		 * Get the NPC controller of the character.
		 * @return the NPC controller.
		 */
		NonPlayerCharacterController* npcController() const {
			return npcController_;
		}

		/**
		 * Check if the character has an NPC controller.
		 * @return true if the character has an NPC controller, false otherwise.
		 */
		bool hasNPCController() const {
			return npcController_ != nullptr;
		}

	protected:
		Blackboard *knowledgeBase_ = nullptr;
		NonPlayerCharacterController *npcController_ = nullptr;
	};

	class NPCObject : public CharacterObject {
	public:
		explicit NPCObject(std::string_view name) : CharacterObject(name) {}

		~NPCObject() override = default;
	};

	class AnimalObject : public NPCObject {
	public:
		explicit AnimalObject(std::string_view name) :
			NPCObject(name) {
			objectType_ = ObjectType::ANIMAL;
		}
		~AnimalObject() override = default;
	};

	class PersonObject : public NPCObject {
	public:
		explicit PersonObject(std::string_view name) :
			NPCObject(name) {
			objectType_ = ObjectType::CHARACTER;
		}
		~PersonObject() override = default;
	};

	class PlayerObject : public CharacterObject {
	public:
		explicit PlayerObject(std::string_view name) :
			CharacterObject(name) {
			objectType_ = ObjectType::PLAYER;
		}
		~PlayerObject() override = default;
	};
} // namespace

#include "../blackboard.h"
#include "../npc-controller.h"

#endif /* REGEN_CHARACTER_OBJECT_GROUP_H_ */
