#include "bounding-sphere.h"
#include "regen/objects/primitives/sphere.h"

using namespace regen;

static Vec3f computeBasePosition(const ref_ptr<Mesh> &mesh, const std::vector<ref_ptr<Mesh>> &parts) {
	if (mesh.get() && parts.empty()) {
		return mesh->centerPosition();
	}
	Vec3f min, max;
	if (mesh.get()) {
		min = mesh->minPosition();
		max = mesh->maxPosition();
	}
	for (const auto &part : parts) {
		if (part.get()) {
			min.setMin(part->minPosition());
			max.setMax(part->maxPosition());
		}
	}
	return (min + max) * 0.5;
}

BoundingSphere::BoundingSphere(const Vec3f &basePosition, GLfloat radius)
		: BoundingShape(BoundingShapeType::SPHERE),
		  basePosition_(basePosition),
		  radius_(radius) {
	radiusSquared_ = radius_ * radius_;
}

BoundingSphere::BoundingSphere(const ref_ptr<Mesh> &mesh, const std::vector<ref_ptr<Mesh>> &parts, float radius)
		: BoundingShape(BoundingShapeType::SPHERE, mesh, parts),
		  basePosition_(computeBasePosition(mesh, parts)),
		  radius_(radius > 0.0f ? radius : computeRadius(mesh, parts)) {
	if (auto sphereMesh = dynamic_cast<Sphere *>(mesh.get())) {
		radius_ = sphereMesh->radius();
	}
	radiusSquared_ = radius_ * radius_;
}

void BoundingSphere::updateBaseBounds(const Vec3f &min, const Vec3f &max) {
	basePosition_ = (min + max) * 0.5 + baseOffset_;
	if (auto sphereMesh = dynamic_cast<Sphere *>(mesh_.get())) {
		radius_ = sphereMesh->radius();
	} else {
		radius_ = (max - basePosition_).length();
	}
	radiusSquared_ = radius_ * radius_;
}

float BoundingSphere::computeRadius(const ref_ptr<Mesh> &mesh, const std::vector<ref_ptr<Mesh>> &parts) const {
	Vec3f min, max;
	if (mesh_.get()) {
		if (parts.empty()) {
			if (auto sphereMesh = dynamic_cast<Sphere *>(mesh.get())) {
				return sphereMesh->radius();
			}
		}
		min = mesh_->minPosition();
		max = mesh_->maxPosition();
	}
	for (const auto &part : parts) {
		if (part.get()) {
			min.setMin(part->minPosition());
			max.setMax(part->maxPosition());
		}
	}
	return (max - basePosition_).length();
}

bool BoundingSphere::updateTransform(bool forceUpdate) {
	if (!forceUpdate && tfStamp() == lastTransformStamp_) {
		return false;
	} else {
		lastTransformStamp_ = tfStamp();
		updateShapeOrigin();
		return true;
	}
}

void BoundingSphere::updateShapeOrigin() {
	tfOrigin_ = basePosition_ + translation();
}

Vec3f BoundingSphere::closestPointOnSurface(const Vec3f &point) const {
	const Vec3f &p = tfOrigin();
	Vec3f d = point - p;
	if (d.lengthSquared() < 1e-6f) {
		return p + Vec3f(0, 0, radius());
	} else {
		d.normalize();
		return p + d * radius();
	}
}

bool BoundingSphere::hasIntersectionWithShape(const BoundingShape &other) const {
	const Vec3f &p_this = tfOrigin();
	const Vec3f &p_other = other.tfOrigin();
	if ((p_this - p_other).lengthSquared() <= radiusSquared_) {
		return true;
	}
	Vec3f closestPoint = other.closestPointOnSurface(p_this);
	return (closestPoint - p_this).lengthSquared() <= radiusSquared_;
}

bool BoundingSphere::hasIntersectionWithSphere(const BoundingSphere &other) const {
	const Vec3f &p_this = tfOrigin();
	const Vec3f &p_other = other.tfOrigin();
	const float r_sum = radiusSquared_ + other.radiusSquared_;
	return (p_this - p_other).lengthSquared() <= (r_sum * r_sum);
}

/**
bool hasHalfSphereIntersection(const Vec3f &center, GLfloat radius) const {
	// get the distance from the camera to the center of the sphere
	auto d = Plane(
			position_[0].xyz_(),
			direction_[0].xyz_()).distance(center);
	// check if the sphere is outside the far plane
	if (d - radius > projParams_[0].far) return false;
	// check if the sphere is inside the near plane
	if (d + radius < projParams_[0].near) return false;
	// check if the sphere is inside the half sphere
	auto halfSphereRadius = projParams_[0].far;
	auto halfSphereNormal = direction_[0];
	auto halfSphereCenter = position_[0].xyz_() + halfSphereNormal.xyz_() * halfSphereRadius;
	return Plane(halfSphereCenter, halfSphereNormal.xyz_()).distance(center) < radius;
}

bool hasHalfSphereIntersection(const Vec3f &center, const Vec3f *points) const {
	// get the distance from the camera to the center of the sphere
	auto d = Plane(
			position_[0].xyz_(),
			direction_[0].xyz_()).distance(center);
	// check if the sphere is outside the far plane
	if (d > projParams_[0].far) return false;
	// check if the sphere is inside the near plane
	if (d < projParams_[0].near) return false;
	// check if the sphere is inside the half sphere
	auto halfSphereRadius = projParams_[0].far;
	auto halfSphereNormal = direction_[0];
	auto halfSphereCenter = position_[0].xyz_() + halfSphereNormal.xyz_() * halfSphereRadius;
	auto halfSphere = Plane(halfSphereCenter, halfSphereNormal.xyz_());
	for (int i = 0; i < 8; ++i) {
		if (halfSphere.distance(center + points[i]) < 0) {
			return false;
		}
	}
	return true;
}
 */
