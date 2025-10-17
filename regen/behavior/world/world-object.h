#ifndef REGEN_WORLD_OBJECT_H_
#define REGEN_WORLD_OBJECT_H_
#include "action-type.h"
#include "regen/shader/shader-input.h"
#include "regen/shapes/bounding-shape.h"

namespace regen {
	enum class ObjectType {
		THING = 0,
		PLACE,
		LOCATION,
		WAYPOINT,
		CHARACTER,
		ANIMAL,
		PLAYER,
		COLLECTION
	};

	// forward declarations, include happens below.
	class ObjectGroup;
	class Location;
	struct Affordance;

	class WorldObject : public Resource {
	public:
		explicit WorldObject(std::string_view name);

		~WorldObject() override;

		const std::string& name() const { return name_; }

		uint32_t instanceIdx() const { return instanceIdx_; }

		void setPosition(const Vec3f &position);

		void setPosition(const ref_ptr<ShaderInput3f> &position);

		void setShape(const ref_ptr<BoundingShape> &shape, uint32_t instanceIdx);

		bool hasShape() const { return shape_.get() != nullptr; }

		const ref_ptr<BoundingShape> &shape() const { return shape_; }

		ObjectType objectType() const { return objectType_; }

		void setObjectType(ObjectType type) { objectType_ = type; }

		uint32_t factionMask() const { return factionMask_; }

		bool hasFaction(uint32_t faction) const { return (factionMask_ & faction) != 0; }

		bool isFriendly(const WorldObject &other) const;

		void addFaction(uint32_t faction) { factionMask_ |= faction; }

		void removeFaction(uint32_t faction) { factionMask_ &= ~faction; }

		const ref_ptr<WorldObject> &placeOfObject() const { return placeOfObject_; }

		void setPlaceOfObject(const ref_ptr<WorldObject> &place) { placeOfObject_ = place; }

		bool isStatic() const { return isStatic_; }

		bool isDynamic() const { return !isStatic_; }

		void setStatic(bool isStatic) { isStatic_ = isStatic; }

		float danger() const { return danger_; }

		void setDanger(float danger) { danger_ = danger; }

		void setRadius(float radius) { radius_ = radius; }

		float radius() const { return radius_; }

		Vec2f position2D() const;

		Vec3f position3D() const;

		const std::vector<ref_ptr<Affordance>> &affordances() const { return affordances_; }

		void addAffordance(const ref_ptr<Affordance> &affordance);

		bool hasAffordance(ActionType actionType) const;

		ref_ptr<Affordance> getAffordance(ActionType actionType) const;

		void setCurrentLocation(const ref_ptr<Location> &location);

		void unsetCurrentLocation();

		void setCurrentLocationIndex(int32_t locIdx);

		const ref_ptr<Location> &currentLocation() const { return currentLocation_; }

		int32_t currentLocationIndex() const { return currentLocationIdx_; }

		const ref_ptr<ObjectGroup>& currentGroup() const { return currentGroup_; }

		bool isPartOfGroup() const { return currentGroup_.get() != nullptr; }

		int32_t currentGroupIndex() const { return currentGroupIdx_; }

		void setCurrentGroupIndex(int32_t idx) { currentGroupIdx_ = idx; }

		bool hasCurrentGroup() const { return currentGroup_.get() != nullptr; }

		void leaveCurrentGroup();

		void joinGroup(const ref_ptr<ObjectGroup> &group);

		static  ref_ptr<WorldObject> load(LoadingContext &ctx, scene::SceneInputNode &n);

	protected:
		const std::string name_;
		uint32_t instanceIdx_ = 0;
		float radius_ = 1.0f;

		ref_ptr<BoundingShape> shape_;
		ref_ptr<ShaderInput3f> pos_;

		// The broad type of the object.
		ObjectType objectType_ = ObjectType::THING;
		// Normalized danger level [0,1], where 1 is very dangerous
		// and 0 is safe.
		float danger_ = 0.0f;
		// Flag indicating if the object is static (not moving)
		// or dynamic (moving).
		bool isStatic_ = true;
		// Bitfield for faction membership, up to 32 factions supported.
		// Default is 0, meaning no faction.
		// Factions can be used to determine friend/foe relationships.
		// Two objects are considered friends if they share at least
		// one faction.
		uint32_t factionMask_ = 0;
		// The current location the object is in, if any.
		ref_ptr<Location> currentLocation_;
		// Index of the location the object is currently in, if any.
		int32_t currentLocationIdx_ = -1;
		// The current social group the object is part of, if any.
		// Only one social group membership is supported at the moment.
		// The meaning is rather that the character currently actively participates
		// in the social group, e.g. talking to other group members.
		ref_ptr<ObjectGroup> currentGroup_;
		int32_t currentGroupIdx_ = -1;

		std::vector<ref_ptr<Affordance>> affordances_;
		ref_ptr<WorldObject> placeOfObject_;

		// function pointer for reading position
		Vec3f (WorldObject::*getPosition3D_)() const;
		Vec2f (WorldObject::*getPosition2D_)() const;

		Vec2f position2D_shape() const;
		Vec2f position2D_local() const;

		Vec3f position3D_shape() const;
		Vec3f position3D_local() const;
	};
} // namespace

// include forward declarations
#include "affordance.h"
#include "location.h"
#include "object-group.h"

namespace regen {
	class WorldObjectVec : public std::vector<ref_ptr<WorldObject>>, public Resource {
	public:
		static constexpr const char *TYPE_NAME = "WorldObject";

		WorldObjectVec() = default;

		explicit WorldObjectVec(uint32_t numInstances) :
			std::vector<ref_ptr<WorldObject>>(numInstances) {}

		WorldObjectVec &operator=(const std::vector<ref_ptr<WorldObject>> &rhs) {
			std::vector<ref_ptr<WorldObject>>::operator=(rhs);
			return *this;
		}

		/**
		 * Load world objects from scene input.
		 * Note that each item may create multiple world objects,
		 * e.g. when a shape with multiple instances is used.
		 * @param ctx the loading context.
		 * @param input the scene input node.
		 * @return a vector of loaded world objects.
		 */
		static ref_ptr<WorldObjectVec> load(LoadingContext &ctx, scene::SceneInputNode &input);
	};

	struct Patient {
		ref_ptr<WorldObject> object;
		ref_ptr<Affordance> affordance;
		int affordanceSlot = -1;
		float currentDistance = std::numeric_limits<float>::max();
	};

	class CharacterObject : public WorldObject {
	public:
		explicit CharacterObject(std::string_view name) :
			WorldObject(name) {
			objectType_ = ObjectType::CHARACTER;
			isStatic_ = false;
		}
		~CharacterObject() override = default;
	};

	class AnimalObject : public WorldObject {
	public:
		explicit AnimalObject(std::string_view name) :
			WorldObject(name) {
			objectType_ = ObjectType::ANIMAL;
			isStatic_ = false;
		}
		~AnimalObject() override = default;
	};

	class PlayerObject : public WorldObject {
	public:
		explicit PlayerObject(std::string_view name) :
			WorldObject(name) {
			objectType_ = ObjectType::PLAYER;
			isStatic_ = false;
		}
		~PlayerObject() override = default;
	};

	std::ostream &operator<<(std::ostream &out, const ObjectType &v);

	std::istream &operator>>(std::istream &in, ObjectType &v);
} // namespace

#endif /* REGEN_WORLD_OBJECT_H_ */
