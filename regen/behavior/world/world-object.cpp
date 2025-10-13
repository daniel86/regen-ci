#include "world-object.h"
#include <regen/objects/mesh-state.h>

#include "regen/scene/resource-manager.h"

using namespace regen;

Affordance::Affordance(const ref_ptr<WorldObject> &owner) : owner(owner) {
	// TODO: Also support dynamic world objects here, e.g. chairs that can be moved.
	//         Then it makes sense to compute affordance slot in local space instead.
}

void Affordance::initialize() {
	if (slotCount < 1) slotCount = 1;
	freeSlots = slotCount;
	users.resize(slotCount);
	slotPositions.resize(slotCount);
	for (int32_t i = 0; i < slotCount; i++) {
		users[i] = {};
		slotPositions[i] = computeSlotPosition(i);
	}
}

Vec3f Affordance::computeSlotPosition(int idx) const {
	Vec3f pos, center;
	if (owner->hasShape()) {
		auto tf = owner->shape()->transform();
		if (tf.get() && tf->hasModelMat()) {
			auto mat = tf->modelMat()->getVertex(owner->shape()->instanceID());
			pos = (mat.r ^ baseOffset).xyz_();
			center = mat.r.position();
		} else {
			pos = center + baseOffset;
			center = owner->shape()->tfOrigin();
		}
	} else {
		center = owner->position3D();
		pos = center + baseOffset;
	}

	switch (layout) {
		case SlotLayout::CIRCULAR: {
			float angle = (2.0f * M_PIf * static_cast<float>(idx)) /
				static_cast<float>(std::max(1, slotCount));
			pos.x += std::cos(angle) * radius;
			pos.z += std::sin(angle) * radius;
			return pos;
		}

		case SlotLayout::CIRCULAR_GRID: {
			// Concentric circular layers
			int perRing = slotCount / numRings;
			auto ring = static_cast<float>(idx / perRing);
			auto i = static_cast<float>(idx % perRing);

			float layoutRadius = minDistance + ring * spacing;
			float angle = (2.0f * M_PIf * i) / static_cast<float>(perRing);

			pos.x += std::cos(angle) * layoutRadius;
			pos.z += std::sin(angle) * layoutRadius;
			return pos;
		}


		case SlotLayout::GRID: {
			// Compute rows & columns
			int cols = std::max(std::ceil(std::sqrt(slotCount)), 2.0);
			auto row = static_cast<float>(idx / cols);
			auto col = static_cast<float>(idx % cols);

			// Orientation
			Vec3f forward = pos - center;
			forward.y = 0.0;
			float l = forward.length();
			if (l < 1e-3f) {
				forward = Vec3f(0.0f, 0.0f, 1.0f);
			} else {
				forward /= l;
			}
			Vec3f right = forward.cross(Vec3f::up());
			right.normalize();

			// Grid center offset
			float gridWidth = static_cast<float>(cols - 1) * spacing;
			float gridHeight = std::ceil(static_cast<float>(slotCount) /
				static_cast<float>(cols) - 1.0f) * spacing;

			pos = pos - (right * gridWidth) * 0.5f + (forward * gridHeight) * 0.5f;
			return pos + right * (col * spacing) - forward * (row * spacing);
		}
	}
	return center;
}

bool Affordance::hasFreeSlot() const {
	return freeSlots > 0;
}

int Affordance::reserveSlot(WorldObject *user, bool randomizeSlot) {
	int randomOffset = randomizeSlot ? math::randomInt() % slotCount : 0;
	for (int i = 0; i < slotCount; ++i) {
		int idx = (i + randomOffset) % slotCount;
		if (!users[idx]) {
			users[idx] = user;
			--freeSlots;
			return idx;
		}
	}
	return -1;
}

void Affordance::releaseSlot(int slotIdx) {
	if (slotIdx < 0 || slotIdx >= slotCount) return;
	if (users[slotIdx]) {
		users[slotIdx] = {};
		++freeSlots;
	}
}

WorldObject::WorldObject(std::string_view name) : name_(name) {
	getPosition2D_ = nullptr;
	getPosition3D_ = nullptr;
}

WorldObject::~WorldObject() {
	if (shape_.get()) {
		shape_->unsetWorldObject(this);
	}
}

void WorldObject::setShape(const ref_ptr<BoundingShape> &shape, uint32_t instanceIdx) {
	shape_ = shape;
	instanceIdx_ = instanceIdx;
	getPosition2D_ = &WorldObject::position2D_shape;
	getPosition3D_ = &WorldObject::position3D_shape;
	shape_->setWorldObject(this);
	for (auto &aff: affordances_) {
		aff->initialize();
	}
}

void WorldObject::setPosition(const ref_ptr<ShaderInput3f> &position) {
	pos_ = position;
	getPosition2D_ = &WorldObject::position2D_local;
	getPosition3D_ = &WorldObject::position3D_local;
	if (shape_.get()) {
		shape_->unsetWorldObject(this);
		shape_ = {};
	}
	for (auto &aff: affordances_) {
		aff->initialize();
	}
}

void WorldObject::setPosition(const Vec3f &position) {
	if (!pos_) {
		pos_ = createUniform<ShaderInput3f>("pos", position);
	} else {
		pos_->setVertex(0, position);
	}
	getPosition2D_ = &WorldObject::position2D_local;
	getPosition3D_ = &WorldObject::position3D_local;
	if (shape_.get()) {
		shape_->unsetWorldObject(this);
		shape_ = {};
	}
	for (auto &aff: affordances_) {
		aff->initialize();
	}
}

Vec2f WorldObject::position2D() const {
	return (this->*getPosition2D_)();
}

Vec3f WorldObject::position3D() const {
	return (this->*getPosition3D_)();
}

Vec2f WorldObject::position2D_shape() const {
	const Vec3f &p = shape_->tfOrigin();
	return { p.x, p.z };
}

Vec3f WorldObject::position3D_shape() const {
	return shape_->tfOrigin();
}

Vec2f WorldObject::position2D_local() const {
	const Vec3f &p = pos_->getVertex(0).r;
	return { p.x, p.z };
}

Vec3f WorldObject::position3D_local() const {
	return pos_->getVertex(0).r;
}

void WorldObject::addAffordance(const ref_ptr<Affordance> &affordance) {
	affordances_.push_back(affordance);
	if (getPosition2D_) {
		affordance->initialize();
	}
}

bool WorldObject::hasAffordance(ActionType actionType) const {
	for (const auto &aff: affordances_) {
		if (aff->type == actionType) {
			return true;
		}
	}
	return false;
}

ref_ptr<Affordance> WorldObject::getAffordance(ActionType actionType) const {
	for (const auto &aff: affordances_) {
		if (aff->type == actionType) {
			return aff;
		}
	}
	return {};
}

ref_ptr<WorldObject> WorldObject::load(LoadingContext &ctx, scene::SceneInputNode &n) {
	ref_ptr<WorldObject> wo = ref_ptr<WorldObject>::alloc(n.getName());
	if (n.hasAttribute("type")) {
		wo->setObjectType(n.getValue<ObjectType>("type", ObjectType::THING));
	}
	wo->setStatic(n.getValue<bool>("static", true));
	wo->setDanger(n.getValue<float>("danger", 0.0f));
	wo->setRadius(n.getValue<float>("radius", 1.0f));

	// load affordances
	for (const auto &a: n.getChildren("affordance")) {
		ref_ptr<Affordance> affordance = ref_ptr<Affordance>::alloc(wo);
		affordance->type = a->getValue<ActionType>("type", ActionType::IDLE);
		affordance->minDistance = a->getValue<float>("min-distance", 0.0f);
		affordance->slotCount = a->getValue<int>("num-slots", 1);
		affordance->spacing = a->getValue<float>("spacing", 2.0f);
		affordance->layout = a->getValue<SlotLayout>("slot-layout", SlotLayout::CIRCULAR);
		affordance->radius = a->getValue<float>("radius", 1.0f);
		affordance->baseOffset = a->getValue<Vec3f>("base-offset", Vec3f(0.0f, 0.0f, 0.0f));
		wo->addAffordance(affordance);
	}

	// load factions
	if (n.hasAttribute("factions")) {
		const auto indicesAtt = n.getValue<std::string>("factions", "0");
		std::vector<std::string> indicesStr;
		boost::split(indicesStr, indicesAtt, boost::is_any_of(","));
		for (auto & it : indicesStr) {
			int i = atoi(it.c_str());
			if (i >= 0 && i < 32) {
				wo->addFaction(1u << i);
			} else {
				REGEN_WARN("Ignoring " << n.getDescription() << ", invalid faction index '" << it << "'.");
			}
		}
	} else if (n.hasAttribute("faction")) {
		int i = n.getValue<int>("faction", -1);
		if (i >= 0 && i < 32) {
			wo->addFaction(1u << i);
		} else {
			REGEN_WARN("Ignoring " << n.getDescription() << ", invalid faction index '" << i << "'.");
		}
	}

	ctx.scene()->getResources()->getWorldModel()->addWorldObject(wo);
	return wo;
}

ref_ptr<WorldObjectVec> WorldObjectVec::load(LoadingContext &ctx, scene::SceneInputNode &n) {
	ref_ptr<WorldObjectVec> objVec = ref_ptr<WorldObjectVec>::alloc();

	if (n.hasAttribute("mesh")) {
		uint32_t meshIdx = n.getValue<uint32_t>("mesh-index", 0u);
		auto meshes = ctx.scene()->getResources()->getMesh(ctx.scene(), n.getValue("mesh"));
		if (!meshes || meshes->empty()) {
			REGEN_WARN("Cannot find mesh in `" << n.getDescription() << "`.");
		} else if (meshIdx >= meshes->size()) {
			REGEN_WARN("Invalid mesh index in `" << n.getDescription() << "`.");
		} else {
			auto mesh = (*meshes.get())[meshIdx];
			auto shape = mesh->indexedShape(0);
			uint32_t numInstances = shape->transform()->numInstances();
			// load range of instances
			std::list<scene::IndexRange> instanceRange = n.getIndexSequence(numInstances);
			for (const auto &r: instanceRange) {
				for (uint32_t instance = r.from; instance <= r.to; instance += r.step) {
					auto wo = WorldObject::load(ctx, n);
					wo->setShape(mesh->indexedShape(instance), instance);
					objVec->push_back(wo);
				}
			}
		}
	}
	if (objVec->empty()) {
		return {};
	}
	ctx.scene()->putResource<WorldObjectVec>(n.getName(), objVec);
	return objVec;
}

std::ostream &regen::operator<<(std::ostream &out, const ObjectType &v) {
	switch (v) {
		case ObjectType::THING:
			return out << "THING";
		case ObjectType::PLACE:
			return out << "PLACE";
		case ObjectType::WAYPOINT:
			return out << "WAYPOINT";
		case ObjectType::CHARACTER:
			return out << "CHARACTER";
		case ObjectType::ANIMAL:
			return out << "ANIMAL";
		case ObjectType::PLAYER:
			return out << "PLAYER";
	}
	return out;
}

std::istream &regen::operator>>(std::istream &in, ObjectType &v) {
	std::string val;
	in >> val;
	boost::to_upper(val);
	if (val == "THING") v = ObjectType::THING;
	else if (val == "PLACE") v = ObjectType::PLACE;
	else if (val == "WAYPOINT") v = ObjectType::WAYPOINT;
	else if (val == "CHARACTER") v = ObjectType::CHARACTER;
	else if (val == "ANIMAL") v = ObjectType::ANIMAL;
	else if (val == "PLAYER") v = ObjectType::PLAYER;
	else {
		REGEN_WARN("Unknown object type '" << val << "'. Using THING.");
		v = ObjectType::THING;
	}
	return in;
}

std::ostream &regen::operator<<(std::ostream &out, const SlotLayout &v) {
	switch (v) {
		case SlotLayout::CIRCULAR:
			return out << "CIRCULAR";
		case SlotLayout::GRID:
			return out << "GRID";
		case SlotLayout::CIRCULAR_GRID:
			return out << "CIRCULAR_GRID";
	}
	return out;
}

std::istream &regen::operator>>(std::istream &in, SlotLayout &v) {
	std::string val;
	in >> val;
	boost::to_upper(val);
	if (val == "CIRCULAR") v = SlotLayout::CIRCULAR;
	else if (val == "GRID") v = SlotLayout::GRID;
	else if (val == "CIRCULAR_GRID" || val == "CONCENTRIC") v = SlotLayout::CIRCULAR_GRID;
	else {
		REGEN_WARN("Unknown slot layout '" << val << "'. Using CIRCULAR.");
		v = SlotLayout::CIRCULAR;
	}
	return in;
}
