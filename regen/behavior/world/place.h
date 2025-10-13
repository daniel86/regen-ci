#ifndef REGEN_MODEL_PLACE_TYPE_H_
#define REGEN_MODEL_PLACE_TYPE_H_

#include <iostream>
#include "world-object.h"
#include "pathway.h"

namespace regen {
	/**
	 * The type of a place of interest for NPCs.
	 */
	enum class PlaceType {
		HOME = 0,
		SPIRITUAL,
		GATHERING,
		PATROL_POINT,
		LAST
	};

	/**
	 * A place of interest for the NPC.
	 */
	class Place : public WorldObject {
	public:
		Place(std::string_view name, PlaceType type) :
			WorldObject(name),
			placeType_(type) {
			objectType_ = ObjectType::PLACE;
		}

		~Place() override = default;

		PlaceType placeType() const { return placeType_; }

		const std::vector<ref_ptr<WorldObject>> &worldObjects() const {
			return objects_;
		}

		void addWorldObject(const ref_ptr<WorldObject> &obj) {
			objects_.push_back(obj);
			for (const auto &aff: obj->affordances()) {
				affordanceMap_[aff->type].push_back(obj);
			}
		}

		bool hasAffordance(ActionType actionType) const {
			auto it = affordanceMap_.find(actionType);
			return (it != affordanceMap_.end() && !it->second.empty());
		}

		const std::vector<ref_ptr<WorldObject>>& getAffordanceObjects(ActionType actionType) {
			return affordanceMap_[actionType];
		}

		void addPathWay(PathwayType pathWayType, const std::vector<ref_ptr<WayPoint>> &pathWay) {
			pathWays_[pathWayType].push_back(pathWay);
		}

		bool hasPathWay(PathwayType pathWayType) const {
			auto it = pathWays_.find(pathWayType);
			return (it != pathWays_.end() && !it->second.empty());
		}

		const std::vector<std::vector<ref_ptr<WayPoint>>>& getPathWays(PathwayType pathWayType) {
			return pathWays_[pathWayType];
		}
	protected:
		const PlaceType placeType_;
		std::vector<ref_ptr<WorldObject>> objects_;

		std::map<PathwayType, std::vector<std::vector<ref_ptr<WayPoint>>>> pathWays_;
		std::map<ActionType, std::vector<ref_ptr<WorldObject>>> affordanceMap_;
	};

	std::ostream &operator<<(std::ostream &out, const PlaceType &v);

	std::istream &operator>>(std::istream &in, PlaceType &v);
} // namespace

#endif /* REGEN_MODEL_PLACE_TYPE_H_ */
