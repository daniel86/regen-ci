#ifndef REGEN_WORLD_OBJECT_H_
#define REGEN_WORLD_OBJECT_H_
#include "action-type.h"
#include "regen/shader/shader-input.h"
#include "regen/shapes/bounding-shape.h"

namespace regen {
	enum class SlotLayout {
		CIRCULAR = 0,
		GRID,
		CIRCULAR_GRID
	};

	enum class ObjectType {
		THING = 0,
		PLACE,
		WAYPOINT,
		CHARACTER,
		ANIMAL,
		PLAYER
	};

	class WorldObject;

	struct Affordance {
		ActionType type = ActionType::IDLE;
		SlotLayout layout = SlotLayout::CIRCULAR;
		// number of NPCs that can use this affordance at the same time
		int slotCount = 1;
		int freeSlots = 1;
		int numRings = 2; // for concentric grid layout
		// Inner exclusion radius around center
		float minDistance = 0.0f;
		// Radius for circular layout
		float radius = 1.0f;
		// Spacing between slots in grid layout
		float spacing = 3.0f;
		// The owner of the affordance
		ref_ptr<WorldObject> owner;
		// Current users of this affordance, size of vector is slotCount
		std::vector<const WorldObject*> users;
		Vec3f baseOffset = Vec3f::zero();
		// Precomputed slot positions in world space.
		std::vector<Vec3f> slotPositions;

		explicit Affordance(const ref_ptr<WorldObject> &owner);

		void initialize();

		bool hasFreeSlot() const;

		int reserveSlot(const WorldObject *user, bool randomizeSlot=false);

		void releaseSlot(int slotIdx);

		const Vec3f& slotPosition(int idx) const { return slotPositions[idx]; }

	protected:
		Vec3f computeSlotPosition(int idx) const;
	};

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

		bool isFriendly(const WorldObject &other) const {
			return (factionMask_ & other.factionMask_) != 0;
		}

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

	std::ostream &operator<<(std::ostream &out, const SlotLayout &v);

	std::istream &operator>>(std::istream &in, SlotLayout &v);
} // namespace

#endif /* REGEN_WORLD_OBJECT_H_ */
