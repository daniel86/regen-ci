#include "height-map.h"

#define SMOOTH_HEIGHT

using namespace regen;

HeightMap::HeightMap() : Texture2D() {
}

HeightMap::HeightMap(const Texture2D &other) : Texture2D(other) {
}

void HeightMap::setMapCenter(const Vec2f &center) {
	mapCenter_ = center;
	mapBounds_.min = mapCenter_ - mapSize_ * 0.5f;
	mapBounds_.max = mapCenter_ + mapSize_ * 0.5f;
}

void HeightMap::setMapSize(const Vec2f &size) {
	mapSize_ = size;
	mapBounds_.min = mapCenter_ - mapSize_ * 0.5f;
	mapBounds_.max = mapCenter_ + mapSize_ * 0.5f;
}

void HeightMap::setMapFactor(float factor) {
	mapFactor_ = factor;
}

float HeightMap::sampleHeight(const Vec2f &pos) {
	// compute UV for height map sampling
	auto uv = (pos - mapCenter_) / mapSize_ + Vec2f(0.5f);
#ifdef SMOOTH_HEIGHT
	auto texelSize = Vec2f(1.0f / width(), 1.0f / height());
	auto regionTS = texelSize * 8.0f;
	auto mapValue = sampleAverage<float>(uv, regionTS, textureData());
#else
	auto mapValue = sampleLinear<float>(uv, textureData());
#endif
	mapValue *= mapFactor_;
	// increase by small bias to avoid intersection with the floor
	mapValue += 0.02f;
	return mapValue - mapFactor_ * 0.5f;
}
