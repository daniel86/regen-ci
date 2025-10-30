#include "location.h"

using namespace regen;

Location::Location(std::string_view name) : WorldObject(name) {
	objectType_ = ObjectType::LOCATION;
	isStatic_ = true;
}

void Location::addVisitor(WorldObject *character) {
	if (numVisitors_ >= static_cast<int32_t>(visitors_.size())) {
		visitors_.resize(visitors_.size() + 4, nullptr);
	}
	character->setCurrentLocationIndex(numVisitors_);
	visitors_[numVisitors_++] = character;
}

void Location::removeVisitor(WorldObject *visitor) {
	auto locationIdx = visitor->currentLocationIndex();
	if (locationIdx < 0) return;
	// Remove by shifting elements down.
	for (int32_t i = locationIdx; i + 1 < numVisitors_; ++i) {
		visitors_[i] = visitors_[i + 1];
		visitors_[i]->setCurrentLocationIndex(i);
	}
	numVisitors_--;
	visitor->setCurrentLocationIndex(-1);
}

void Location::addGroup(ObjectGroup *group) {
	if (numGroups_ >= static_cast<int32_t>(groups_.size())) {
		groups_.resize(groups_.size() + 4, nullptr);
	}
	group->setCurrentLocationIndex(numGroups_);
	groups_[numGroups_++] = group;
}

void Location::removeGroup(ObjectGroup *group) {
	auto locationIdx = group->currentLocationIndex();
	if (locationIdx < 0) return;
	// Remove by shifting elements down.
	for (int32_t i = locationIdx; i + 1 < numGroups_; ++i) {
		groups_[i] = groups_[i + 1];
		groups_[i]->setCurrentLocationIndex(i);
	}
	numGroups_--;
	group->setCurrentLocationIndex(-1);
}
