#ifndef REGEN_MODEL_LOCATION_H_
#define REGEN_MODEL_LOCATION_H_

#include "world-object.h"

namespace regen {
	/**
	 * A location is a special type of world object that can contain other objects
	 * and characters. It represents a place in the world where characters can
	 * gather, e.g. a room, a building, a park, etc.
	 */
	class Location : public WorldObject {
	public:
		explicit Location(std::string_view name);

		~Location() override = default;

		/**
		 * Check if the given object is currently located in this location.
		 * @param obj the object to check.
		 * @return true if the object is in this location, false otherwise.
		 */
		bool isLocationOf(const WorldObject &obj) const {
			return obj.currentLocation().get() == this;
		}

		/**
		 * Add a character to the list of visitors.
		 * @param character the character to add.
		 */
		void addVisitor(WorldObject *character);

		/**
		 * Remove a character from the list of visitors.
		 * @param character the character to remove.
		 */
		void removeVisitor(WorldObject *character);

		/**
		 * Get the number of visitors currently in the location.
		 * @return the number of visitors.
		 */
		int32_t numVisitors() const { return numVisitors_; }

		/**
		 * Get the visitor at the given index.
		 * @param idx the index of the visitor.
		 * @return the visitor, or nullptr if the index is out of range.
		 */
		WorldObject* visitor(uint32_t idx) const { return visitors_[idx]; }

		/**
		 * Add a social group to the list of groups currently in the location.
		 * @param group the social group to add.
		 */
		void addGroup(ObjectGroup *group);

		/**
		 * Remove a social group from the list of groups currently in the location.
		 * @param group the social group to remove.
		 */
		void removeGroup(ObjectGroup *group);

		/**
		 * Get the number of social groups currently in the location.
		 * @return the number of social groups.
		 */
		int32_t numGroups() const { return numGroups_; }

		/**
		 * Get the social group at the given index.
		 * @param idx the index of the social group.
		 * @return the social group, or nullptr if the index is out of range.
		 */
		ObjectGroup* group(uint32_t idx) const { return groups_[idx]; }

	protected:
		// list of characters currently in the location
		std::vector<WorldObject*> visitors_;
		int32_t numVisitors_ = 0;
		// list of groups currently in the location
		std::vector<ObjectGroup*> groups_;
		int32_t numGroups_ = 0;
	};
} // namespace

#endif /* REGEN_MODEL_LOCATION_H_ */
