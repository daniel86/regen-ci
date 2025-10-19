#include "path-planner.h"
#include <queue>
#include <unordered_map>
#include <limits>
#include <cmath>

namespace regen {
	struct NodeRecord {
		ref_ptr<WayPoint> node;
		ref_ptr<WayPoint> parent;
		float gCost = 0.0f; // cost so far
		float fCost = 0.0f; // g + h
	};

	// For priority queue
	struct NodeRecordCompare {
		bool operator()(const NodeRecord &a, const NodeRecord &b) const {
			return a.fCost > b.fCost; // min-heap
		}
	};
}

using namespace regen;

void PathPlanner::addWayPoint(const ref_ptr<WayPoint> &wp) {
	wayPoints_.push_back(wp);
	connections_[wp.get()]; // ensure entry exists
}

void PathPlanner::addConnection(
	const ref_ptr<WayPoint> &from,
	const ref_ptr<WayPoint> &to,
	bool bidirectional) {
	connections_[from.get()].push_back(to);
	if (bidirectional) {
		connections_[to.get()].push_back(from);
	}
}

std::vector<ref_ptr<WayPoint>> PathPlanner::findPath(const Vec2f &from, const Vec2f &to) const {
	// Find a waypoint near the target first.
	// But only consider waypoints that are closer to the source than the target.
	float maxDistanceSq = (from - to).lengthSquared();
	auto closest = getClosest(to, from, maxDistanceSq);
	if (!closest) {
		// Directly go to target if no waypoint could be found
		return {};
	} else {
		return findPath(from, closest);
	}
}

std::vector<ref_ptr<WayPoint> > PathPlanner::findPath(
	const Vec2f &from, const ref_ptr<WayPoint> &lastWP) const {
	auto posLastWP = lastWP->position2D();
	float maxDistanceSq = (from - posLastWP).lengthSquared();
	auto firstWP = getClosest(from, posLastWP, maxDistanceSq);
	if (!firstWP) {
		// Directly go to target if no waypoint could be found
		return {lastWP};
	} else {
		return findPath(firstWP, lastWP);
	}
}

ref_ptr<WayPoint> PathPlanner::getClosest(const Vec2f &pos, const Vec2f &target, float maxDistanceSq) const {
	ref_ptr<WayPoint> best = {};
	float bestDist2 = std::numeric_limits<float>::max();
	for (auto &wp: wayPoints_) {
		Vec2f d = wp->position2D() - pos;
		float dist2 = d.lengthSquared();
		if (dist2 < bestDist2) {
			// Check if waypoint is closer to target than source
			Vec2f dt = wp->position2D() - target;
			float distToTarget2 = dt.lengthSquared();
			if (distToTarget2 < maxDistanceSq) {
				bestDist2 = dist2;
				best = wp;
			}
		}
	}
	return best;
}

static std::vector<ref_ptr<WayPoint> > reconstructPath(
	const std::unordered_map<WayPoint *, NodeRecord> &records,
	const NodeRecord &goalRec) {
	std::vector<ref_ptr<WayPoint> > path;
	NodeRecord current = goalRec;
	while (current.node.get()) {
		path.push_back(current.node);
		if (!current.parent) break;
		current = records.at(current.parent.get());
	}
	std::reverse(path.begin(), path.end());
	return path;
}

std::vector<ref_ptr<WayPoint> > PathPlanner::findPath(
	const ref_ptr<WayPoint> &start,
	const ref_ptr<WayPoint> &goal) const {
	if (!start || !goal) return {};
	if (start == goal) return {start};

	std::priority_queue<NodeRecord, std::vector<NodeRecord>, NodeRecordCompare> open;
	std::unordered_map<WayPoint *, NodeRecord> allRecords;
	std::unordered_map<WayPoint *, bool> closed;

	// Init start
	NodeRecord startRec{
		start, {}, 0.0f,
		heuristic(start, goal)
	};
	open.push(startRec);
	allRecords[start.get()] = startRec;

	while (!open.empty()) {
		NodeRecord current = open.top();
		open.pop();

		if (current.node == goal) {
			return reconstructPath(allRecords, current);
		}

		closed[current.node.get()] = true;

		// Expand neighbors
		for (auto &neighbor: connections_.at(current.node.get())) {
			if (closed[neighbor.get()]) continue;

			float gNew = current.gCost + distance(current.node, neighbor);
			auto it = allRecords.find(neighbor.get());

			if (it == allRecords.end() || gNew < it->second.gCost) {
				NodeRecord rec;
				rec.node = neighbor;
				rec.parent = current.node;
				rec.gCost = gNew;
				rec.fCost = gNew + heuristic(neighbor, goal);

				allRecords[neighbor.get()] = rec;
				open.push(rec);
			}
		}
	}

	// No path, return target only
	return {goal};
}

float PathPlanner::distance(const ref_ptr<WayPoint> &a, const ref_ptr<WayPoint> &b) const {
	Vec2f d = a->position2D() - b->position2D();
	return d.lengthSquared();
}

float PathPlanner::heuristic(const ref_ptr<WayPoint> &a, const ref_ptr<WayPoint> &b) const {
	// Euclidean heuristic (admissible)
	return distance(a, b);
}
