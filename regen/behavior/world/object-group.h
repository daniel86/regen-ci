#ifndef REGEN_OBJECT_GROUP_H_
#define REGEN_OBJECT_GROUP_H_
#include <vector>
#include "action-type.h"
#include "world-object.h"

namespace regen {
	/**
	 * A social group of characters that can perform group activities together.
	 * The group has a maximum size and can keep track of its members.
	 * It can also manage a speaker among its members.
	 */
	class ObjectGroup : public WorldObject {
	public:
		/**
		 * Create a new social group.
		 * @param groupActivity the activity of the social group.
		 * @param maxGroupSize the maximum size of the social group, default is 6.
		 */
		explicit ObjectGroup(ActionType groupActivity, int32_t maxGroupSize = 6);

		~ObjectGroup() override = default;

		/**
		 * @return the center position of the social group.
		 */
		const Vec3f &groupCenter() const { return groupCenter_; }

		/**
		 * @return the center position of the social group in 2D (x,z).
		 */
		const Vec2f &groupCenter2D() const { return groupCenter2D_; }

		/**
		 * Add a member to the social group.
		 * @param member the member to add.
		 * @return the index of the member in the group, or -1 if the group is full or the member is already in the group.
		 */
		int32_t addMember(WorldObject *member);

		/**
		 * Remove a member from the social group.
		 * @param member the member to remove.
		 */
		void removeMember(WorldObject *member);

		/**
		 * Check if a member is part of the social group.
		 * @param member the member to check.
		 * @return true if the member is part of the group, false otherwise.
		 */
		bool hasMember(const WorldObject *member) const;

		/**
		 * Get the number of members in the social group.
		 * @return the number of members.
		 */
		uint32_t numMembers() const { return numMembers_; }

		/**
		 * Get a member of the social group by index.
		 * @param idx the index of the member.
		 * @return the member at the given index.
		 */
		WorldObject* member(uint32_t idx) const { return members_[idx]; }

		/**
		 * Check if the social group is full.
		 * @return true if the group is full, false otherwise.
		 */
		bool isFull() const;

		/**
		 * @return true if there is a current speaker, false otherwise.
		 */
		bool hasSpeaker() const { return speakerIdx_ >= 0; }

		/**
		 * Check if a member is the current speaker of the social group.
		 * @param member the member to check.
		 * @return true if the member is the current speaker, false otherwise.
		 */
		bool isSpeaker(const WorldObject *member) const;

		/**
		 * Set a member as the current speaker of the social group.
		 * @param member the member to set as speaker.
		 * @param time_s the time in seconds the member will speak.
		 */
		void setSpeaker(const WorldObject *member, float time_s);

		/**
		 * Advance the speaker time by a given delta time.
		 * If the speaker time reaches zero, the speaker is unset.
		 * @param dt_s the delta time in seconds.
		 * @param randomize if true, the remaining speaker time is randomized between 0 and the current speaker time.
		 */
		void advanceSpeaker(float dt_s, bool randomize = false);

		/**
		 * Unset the current speaker of the social group.
		 */
		void unsetSpeaker();

	protected:
		const ActionType groupActivity_;
		const int32_t maxGroupSize_;

		// Current members of the social group, size of vector is maxGroupSize
		std::vector<WorldObject*> members_;
		// Center position of the group
		Bounds<Vec3f> groupBounds_;
		Vec3f groupCenter_ = Vec3f::zero();
		Vec2f groupCenter2D_ = Vec2f::zero();
		// Actual number of members in the group
		int32_t numMembers_ = 0;
		// Index of the current speaker in members_, -1 if no current speaker
		int32_t speakerIdx_ = -1;
		// Time the current speaker has left to speak unless unsetSpeaker() is called
		float speakerTime_ = 0.0f;

		void updateGroupPosition();
	};
} // namespace

#endif /* REGEN_OBJECT_GROUP_H_ */
