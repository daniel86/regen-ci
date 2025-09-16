#include "path-planner.h"
#include <queue>
#include <unordered_map>
#include <limits>
#include <cmath>

namespace regen {
	struct NodeRecord {
		ref_ptr<WayPoint> node;
		ref_ptr<WayPoint> parent;
		float gCost; // cost so far
		float fCost; // g + h
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

std::vector<ref_ptr<WayPoint>> PathPlanner::findPath(
	const Vec3f &from, const Vec3f &to) const {
	auto start = getClosest(from);
	auto goal = getClosest(to);
	return findPath(start, goal);
}

std::vector<ref_ptr<WayPoint> > PathPlanner::findPath(
	const Vec3f &from, const ref_ptr<WayPoint> &to) const {
	auto start = getClosest(from);
	return findPath(start, to);
}

ref_ptr<WayPoint> PathPlanner::getClosest(const Vec3f &pos) const {
	ref_ptr<WayPoint> best = {};
	float bestDist2 = std::numeric_limits<float>::max();
	for (auto &wp: wayPoints_) {
		Vec3f d = wp->pos()->getVertex(0).r - pos;
		float dist2 = d.lengthSquared();
		if (dist2 < bestDist2) {
			bestDist2 = dist2;
			best = wp;
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

	// No path
	return {};
}

float PathPlanner::distance(const ref_ptr<WayPoint> &a, const ref_ptr<WayPoint> &b) const {
	Vec3f d = a->pos()->getVertex(0).r - b->pos()->getVertex(0).r;
	return d.length();
}

float PathPlanner::heuristic(const ref_ptr<WayPoint> &a, const ref_ptr<WayPoint> &b) const {
	// Euclidean heuristic (admissible)
	return distance(a, b);
}
