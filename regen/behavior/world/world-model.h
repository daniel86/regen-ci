#ifndef REGEN_WORLD_MODEL_H_
#define REGEN_WORLD_MODEL_H_
#include "regen/shader/shader-input.h"
#include "world-object.h"

namespace regen {
	enum class PathWayType {
		STROLL = 0,
		PATROL,
	};

	class WayPoint : public WorldObject {
	public:
		WayPoint(std::string_view name, const Vec3f &position) :
			WorldObject(name, position) {}

		WayPoint(std::string_view name, const ref_ptr<ShaderInput3f> &position) :
			WorldObject(name, position) {}

		explicit WayPoint(const ref_ptr<WorldObject> &obj) :
			WorldObject(obj->name(), obj->position3D()) {
			setRadius(obj->radius());
		}
	};

	/**
	 * Categories of places of interest for the NPC.
	 */
	enum PlaceType {
		PLACE_HOME = 0,
		PLACE_SPIRITUAL,
		PLACE_GATHERING,
		PLACE_PATROL_POINT
	};

	/**
	 * A place of interest for the NPC.
	 */
	class Place : public WorldObject {
	public:
		Place(std::string_view name, PlaceType type) :
			WorldObject(name, Vec3f::zero()),
			type(type) {}
		Place(std::string_view name, PlaceType type, const Vec3f &position) :
			WorldObject(name, position),
			type(type) {}
		Place(std::string_view name, PlaceType type, const ref_ptr<ShaderInput3f> &position) :
			WorldObject(name, position),
			type(type) {}

		PlaceType placeType() const { return type; }

		const std::vector<ref_ptr<WorldObject>> &worldObjects() const {
			return objects_;
		}

		void addWorldObject(const ref_ptr<WorldObject> &obj) {
			objects_.push_back(obj);
			for (const auto &aff: obj->affordances()) {
				if (aff->type != AffordanceType::NONE) {
					affordanceMap_[aff->type].push_back(obj);
				}
			}
		}

		bool hasAffordance(AffordanceType affordanceType) const {
			auto it = affordanceMap_.find(affordanceType);
			return (it != affordanceMap_.end() && !it->second.empty());
		}

		const std::vector<ref_ptr<WorldObject>>& getAffordanceObjects(AffordanceType affordanceType) {
			return affordanceMap_[affordanceType];
		}

		void addPathWay(PathWayType pathWayType, const std::vector<ref_ptr<WayPoint>> &pathWay) {
			pathWays_[pathWayType].push_back(pathWay);
		}

		bool hasPathWay(PathWayType pathWayType) const {
			auto it = pathWays_.find(pathWayType);
			return (it != pathWays_.end() && !it->second.empty());
		}

		const std::vector<std::vector<ref_ptr<WayPoint>>>& getPathWays(PathWayType pathWayType) {
			return pathWays_[pathWayType];
		}
	protected:
		const PlaceType type;
		std::vector<ref_ptr<WorldObject>> objects_;

		std::map<PathWayType, std::vector<std::vector<ref_ptr<WayPoint>>>> pathWays_;
		std::map<AffordanceType, std::vector<ref_ptr<WorldObject>>> affordanceMap_;
	};

	class WorldModel {
	public:
		WorldModel() = default;

		void addWorldObject(const ref_ptr<WorldObject> &obj) {
			objects_.push_back(obj);
		}

		const std::vector<ref_ptr<WorldObject>> &worldObjects() const {
			return objects_;
		}

		std::vector<ref_ptr<Place> > places;
		std::vector<ref_ptr<WayPoint> > wayPoints;
		std::map<std::string, ref_ptr<WayPoint> > wayPointMap;
		std::vector<std::pair<ref_ptr<WayPoint>, ref_ptr<WayPoint> > > wayPointConnections;
	protected:
		std::vector<ref_ptr<WorldObject>> objects_;
	};

	std::ostream &operator<<(std::ostream &out, const PlaceType &v);

	std::istream &operator>>(std::istream &in, PlaceType &v);

	std::ostream &operator<<(std::ostream &out, const PathWayType &v);

	std::istream &operator>>(std::istream &in, PathWayType &v);
} // namespace

#endif /* REGEN_WORLD_MODEL_H_ */
