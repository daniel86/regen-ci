#ifndef REGEN_WORLD_MODEL_H_
#define REGEN_WORLD_MODEL_H_
#include "place.h"
#include "pathway.h"

namespace regen {
	class WorldModel {
	public:
		WorldModel() = default;

		void addWorldObject(const ref_ptr<WorldObject> &obj) {
			objects_.push_back(obj);
		}

		const std::vector<ref_ptr<WorldObject>> &worldObjects() const {
			return objects_;
		}

		WayPoint* getClosestTo(const WayPoint &other, const std::vector<const WayPoint*> &excludeWayPoints);

		using Connection = std::pair<ref_ptr<WayPoint>, ref_ptr<WayPoint> >;

		std::vector<ref_ptr<Place>> places;
		std::vector<ref_ptr<WayPoint>> wayPoints;
		std::map<std::string, ref_ptr<WayPoint>> wayPointMap;
		std::vector<Connection> wayPointConnections;
	protected:
		std::vector<ref_ptr<WorldObject>> objects_;
	};
} // namespace

#endif /* REGEN_WORLD_MODEL_H_ */
