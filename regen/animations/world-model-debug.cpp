#include "world-model-debug.h"

using namespace regen;

WorldModelDebug::WorldModelDebug(const ref_ptr<WorldModel> &world)
		: StateNode(),
		  HasShader("regen.models.lines"),
		  world_(world),
		  lineLocation_(-1),
		  vbo_(0) {
	lineColor_ = ref_ptr<ShaderInput3f>::alloc("lineColor");
	lineColor_->setUniformData(Vec3f(1.0f));
	state()->setInput(lineColor_);
	state()->joinStates(shaderState_);
	lineVertices_ = ref_ptr<ShaderInput3f>::alloc("lineVertices");
	lineVertices_->setVertexData(2);
	// Create and set up the VAO and VBO
	vao_ = ref_ptr<VAO>::alloc();
	bufferSize_ = sizeof(GLfloat) * 3 * lineVertices_->numVertices();
	glGenBuffers(1, &vbo_);
}

void WorldModelDebug::drawLine(const Vec3f &from, const Vec3f &to, const Vec3f &color) {
	auto rs = RenderState::get();
	lineColor_->setUniformData(color);
	if (lineLocation_ < 0) {
		lineLocation_ = shaderState_->shader()->uniformLocation(lineColor_->name());
	}
	lineColor_->enableUniform(lineLocation_);
	// update cpu-side vertex data
	lineVertices_->setVertex(0, from);
	lineVertices_->setVertex(1, to);
	// update gpu-side vertex data
	{
		auto mappedClientData = lineVertices_->mapClientDataRaw(BUFFER_GPU_READ);
		glBufferData(GL_ARRAY_BUFFER, bufferSize_, mappedClientData.r, GL_DYNAMIC_DRAW);
	}
	// draw the line
	rs->vao().apply(vao_->id());
	lineVertices_->enableAttribute(0);
	glDrawArrays(GL_LINES, 0, 2);
}

void WorldModelDebug::drawCircle(const Vec3f &center, float radius, const Vec3f &color) {
	// draw a circle
	const int segments = 32;
	const float angleStep = 2.0f * M_PI / segments;
	for (int i = 0; i < segments; i++) {
		float angle = i * angleStep;
		float nextAngle = (i + 1) * angleStep;
		Vec3f from = center + Vec3f(std::cos(angle) * radius, 0, std::sin(angle) * radius);
		Vec3f to = center + Vec3f(std::cos(nextAngle) * radius, 0, std::sin(nextAngle) * radius);
		drawLine(from, to, color);
	}
}

inline Vec3f toVec3(const Vec2f &v, float y) {
	return {v.x, y, v.y};
}

static Vec3f getAffordanceColor(AffordanceType type) {
	switch (type) {
		case AffordanceType::CONVERSE:
			return Vec3f(1.0f, 1.0f, 0.0f); // yellow
		case AffordanceType::SIT:
			return Vec3f(1.0f, 0.0f, 0.0f);
		case AffordanceType::SLEEP:
			return Vec3f(0.0f, 0.0f, 1.0f);
		case AffordanceType::PRAY:
			return Vec3f(1.0f, 0.0f, 1.0f); // magenta
		case AffordanceType::OBSERVE:
			return Vec3f(0.0f, 1.0f, 1.0f); // cyan
		default:
			return Vec3f(1.0f, 1.0f, 1.0f); // white
	}
}

void WorldModelDebug::traverse(regen::RenderState *rs) {
	state()->enable(rs);
	rs->arrayBuffer().apply(vbo_);
	for (auto &shape: world_->worldObjects()) {
		auto pos = shape->pos()->getVertex(0).r;
		drawCircle(pos, shape->radius(), Vec3f(0.0f, 1.0f, 0.0f));
		// Draw affordances
		for (auto &aff : shape->affordances()) {
			auto affPos = aff->target.get() ? aff->target->pos()->getVertex(0).r : pos;
			Vec3f from = pos;
			Vec3f to = affPos;
			to.y += 0.1f; // lift target a bit
			Vec3f color = getAffordanceColor(aff->type);
			drawLine(from, to, color);
			if (aff->minDistance > 0.0f) {
				drawCircle(pos, aff->minDistance, color);
			}
		}
	}
	// Draw global navigation paths
	for (auto &path : world_->wayPointConnections) {
		auto from = path.first->pos()->getVertex(0).r;
		auto to = path.second->pos()->getVertex(0).r;
		drawLine(from, to, Vec3f(1.0f, 0.0f, 0.0f));
	}
	// Draw place-specific things
	for (auto &place : world_->places) {
		// Draw patrol and stroll paths
		for (auto &patrolPath : place->getPathWays(PathWayType::PATROL)) {
			for (size_t i = 0; i < patrolPath.size(); i++) {
				auto from = patrolPath[i]->pos()->getVertex(0).r;
				auto to = patrolPath[(i + 1) % patrolPath.size()]->pos()->getVertex(0).r;
				drawLine(from, to, Vec3f(0.0f, 0.0, 1.0f));
			}
		}
		for (auto &strollPath : place->getPathWays(PathWayType::STROLL)) {
			for (size_t i = 0; i < strollPath.size(); i++) {
				auto from = strollPath[i]->pos()->getVertex(0).r;
				auto to = strollPath[(i + 1) % strollPath.size()]->pos()->getVertex(0).r;
				drawLine(from, to, Vec3f(1.0f, 0.5f, 0.0f));
			}
		}
	}
	state()->disable(rs);
}
