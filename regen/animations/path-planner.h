#ifndef REGEN_PATH_PLANNER_H_
#define REGEN_PATH_PLANNER_H_

#include <map>
#include <vector>
#include <regen/math/vector.h>
#include <regen/utility/ref-ptr.h>
#include "world-model.h"

namespace regen {
	/**
	 * A simple path planner using A* search on a graph of way points.
	 */
	class PathPlanner {
	public:
		PathPlanner() = default;

		void addWayPoint(const ref_ptr<WayPoint> &wp);

		void addConnection(
				const ref_ptr<WayPoint> &from,
				const ref_ptr<WayPoint> &to,
				bool bidirectional = true);

		std::vector<ref_ptr<WayPoint>> findPath(
				const Vec3f &from, const Vec3f &to) const;

		std::vector<ref_ptr<WayPoint>> findPath(
				const Vec3f &from, const ref_ptr<WayPoint> &to) const;

	protected:
		std::vector<ref_ptr<WayPoint>> wayPoints_;
		std::map<WayPoint*, std::vector<ref_ptr<WayPoint>>> connections_;

		std::vector<ref_ptr<WayPoint>> findPath(
			const ref_ptr<WayPoint> &start,
			const ref_ptr<WayPoint> &goal) const;

		ref_ptr<WayPoint> getClosest(const Vec3f &pos) const;

		float distance(const ref_ptr<WayPoint> &a, const ref_ptr<WayPoint> &b) const;

		float heuristic(const ref_ptr<WayPoint> &a,  const ref_ptr<WayPoint> &b) const;
	};
} // namespace

#endif /* REGEN_PATH_PLANNER_H_ */
