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
		std::vector<WorldObject*> users;
		Vec3f baseOffset = Vec3f::zero();
		// Precomputed slot positions in world space.
		std::vector<Vec3f> slotPositions;

		explicit Affordance(const ref_ptr<WorldObject> &owner);

		void initialize();

		bool hasFreeSlot() const;

		int reserveSlot(WorldObject *user, bool randomizeSlot=false);

		void releaseSlot(int slotIdx);

		const Vec3f& slotPosition(int idx) const { return slotPositions[idx]; }

	protected:
		Vec3f computeSlotPosition(int idx) const;
	};

	class WorldObject : public Resource {
	public:
		static constexpr const char *TYPE_NAME = "WorldObject";

		WorldObject(std::string_view name, const ref_ptr<BoundingShape> &shape, uint32_t instanceIdx);

		WorldObject(std::string_view name, const Vec3f &position);

		WorldObject(std::string_view name, const ref_ptr<ShaderInput3f> &position);

		~WorldObject() override;

		uint32_t instanceIdx() const { return instanceIdx_; }

		const std::string& name() const { return name_; }

		bool hasShape() const { return shape_.get() != nullptr; }

		const ref_ptr<BoundingShape> &shape() const { return shape_; }

		void setRadius(float radius) { radius_ = radius; }

		float radius() const { return radius_; }

		Vec2f position2D() const;

		Vec3f position3D() const;

		const std::vector<ref_ptr<Affordance>> &affordances() const { return affordances_; }

		void addAffordance(const ref_ptr<Affordance> &affordance);

		bool hasAffordance(ActionType actionType) const;

		ref_ptr<Affordance> getAffordance(ActionType actionType) const;

	protected:
		const std::string name_;
		ref_ptr<BoundingShape> shape_;
		uint32_t instanceIdx_ = 0;
		ref_ptr<ShaderInput3f> pos_;
		float radius_ = 1.0f;
		std::vector<ref_ptr<Affordance>> affordances_;

		// function pointer for reading position
		Vec3f (WorldObject::*getPosition3D_)() const;
		Vec2f (WorldObject::*getPosition2D_)() const;

		Vec2f position2D_shape() const;
		Vec2f position2D_local() const;

		Vec3f position3D_shape() const;
		Vec3f position3D_local() const;
	};

	struct Patient {
		ref_ptr<WorldObject> object;
		ref_ptr<Affordance> affordance;
		int affordanceSlot = -1;
		float currentDistance = std::numeric_limits<float>::max();
	};

	std::ostream &operator<<(std::ostream &out, const SlotLayout &v);

	std::istream &operator>>(std::istream &in, SlotLayout &v);
} // namespace

#endif /* REGEN_WORLD_OBJECT_H_ */
