#include <regen/utility/logging.h>
#include <algorithm>

#include "spatial-index.h"
#include "quad-tree.h"
#include "cull-shape.h"
#include "spatial-index-debug.h"
#include "regen/utility/conversion.h"

// NOTE: this piece of code is performance critical! For many execution paths:
// - avoid the use of std::set, std::unordered_set, std::map, std::unordered_map, etc. here
// 		- also iteration over these containers is expensive!
// - avoid lambda functions
// - avoid alloc/free

using namespace regen;

SpatialIndex::SpatialIndex() {
}

void SpatialIndex::addToIndex(const ref_ptr<BoundingShape> &shape) {
	auto it = nameToShape_.find(shape->name());
	if (it == nameToShape_.end()) {
		auto shapeVector = ref_ptr<std::vector<ref_ptr<BoundingShape>>>::alloc();
		shapeVector->push_back(shape);
		nameToShape_[shape->name()] = shapeVector;
	} else {
		it->second->push_back(shape);
	}
	for (auto &ic: indexCameras_) {
		createIndexShape(ic, shape);
	}
	if (shape->spatialIndexData_.size() < indexCameras_.size()) {
		shape->spatialIndexData_.resize(math::nextPow2(indexCameras_.size()));
	}
}

void SpatialIndex::removeFromIndex(const ref_ptr<BoundingShape> &shape) {
	auto it = nameToShape_.find(shape->name());
	if (it != nameToShape_.end()) {
		auto jt = std::find(it->second->begin(), it->second->end(), shape);
		if (jt != it->second->end()) {
			it->second->erase(jt);
		}
	}
}

void SpatialIndex::addCamera(
		const ref_ptr<Camera> &cullCamera,
		const ref_ptr<Camera> &sortCamera,
		SortMode sortMode,
		Vec4i lodShift) {
	cameraToIndexCamera_[cullCamera.get()] = indexCameras_.size();
	auto &data = indexCameras_.emplace_back();
	data.cullCamera = cullCamera;
	data.sortCamera = sortCamera;
	data.sortMode = sortMode;
	data.lodShift = lodShift;
	data.index = this;
	data.camIdx = static_cast<uint32_t>(indexCameras_.size() - 1);
	for (auto &pair: nameToShape_) {
		for (auto &shape: *pair.second.get()) {
			createIndexShape(data, shape);
			if (shape->spatialIndexData_.size() < indexCameras_.size()) {
				shape->spatialIndexData_.resize(math::nextPow2(indexCameras_.size()));
			}
		}
	}
}

void SpatialIndex::createIndexShape(IndexCamera &ic, const ref_ptr<BoundingShape> &shape) {
	auto needle = ic.nameToShape_.find(shape->name());
	if (needle != ic.nameToShape_.end()) {
		// already created
		needle->second->boundingShapes_.push_back(shape);
		return;
	}
	const uint32_t numLayer = ic.cullCamera->numLayer();
	const uint32_t numInstances = shape->numInstances();
	const uint32_t numIndices = numInstances * numLayer;

	auto is = ref_ptr<IndexedShape>::alloc(ic.cullCamera, ic.sortCamera, shape);
	const uint32_t numLOD = is->numLODs();
	is->idVec_ = ref_ptr<ShaderInput1ui>::alloc("instanceIDs", 1);
	is->idVec_->setInstanceData(numIndices, 1, nullptr);
	is->countVec_ = ref_ptr<ShaderInput1ui>::alloc("instanceCounts", numLayer * numLOD);
	is->countVec_->setInstanceData(1, 1, nullptr);
	is->baseVec_ = ref_ptr<ShaderInput1ui>::alloc("baseInstances", numLayer * numLOD);
	is->baseVec_->setInstanceData(1, 1, nullptr);
	is->setLODShift(ic.lodShift);

	auto mapped_ids = is->idVec_->mapClientData<uint32_t>(BUFFER_GPU_WRITE);
	auto mapped_count = is->countVec_->mapClientData<uint32_t>(BUFFER_GPU_WRITE);
	auto mapped_base = is->baseVec_->mapClientData<uint32_t>(BUFFER_GPU_WRITE);
	for (unsigned int instanceIdx = 0; instanceIdx < numInstances; ++instanceIdx) {
		// write instance data for every layer
		for (unsigned int layerIdx = 0; layerIdx < numLayer; ++layerIdx) {
			mapped_ids.w[instanceIdx + layerIdx * numInstances] = instanceIdx;
		}
	}
	for (unsigned int layerIdx = 0; layerIdx < numLayer; ++layerIdx) {
		for (unsigned int lodLevel = 0; lodLevel < numLOD; ++lodLevel) {
			uint32_t binIdx = lodLevel * numLayer + layerIdx;
			if (lodLevel==0) {
				mapped_count.w[binIdx] = numInstances;
				mapped_base.w[binIdx] = layerIdx * numInstances;
			} else {
				mapped_count.w[binIdx] = 0;
				mapped_base.w[binIdx] = (layerIdx+1) * numInstances;
			}
		}
	}
	mapped_base.unmap();
	mapped_count.unmap();
	mapped_ids.unmap();

	is->shapeIdx_ = static_cast<uint16_t>(ic.indexShapes_.size());
	ic.nameToShape_[shape->name()] = is;
	ic.indexShapes_.push_back(is.get());
	is->boundingShapes_.push_back(shape);
	// mark camera as dirty as the camera buffers must be resized.
	// We do this lazy to avoid multiple resizes when adding many shapes.
	ic.isDirty = true;
}

bool SpatialIndex::hasCamera(const Camera &camera) const {
	return cameraToIndexCamera_.find(&camera) != cameraToIndexCamera_.end();
}

std::vector<const Camera *> SpatialIndex::cameras() const {
	std::vector<const Camera *> result;
	for (auto &ic: indexCameras_) {
		result.push_back(ic.cullCamera.get());
	}
	return result;
}

ref_ptr<IndexedShape> SpatialIndex::getIndexedShape(const ref_ptr<Camera> &camera, std::string_view shapeName) {
	auto it = cameraToIndexCamera_.find(camera.get());
	if (it == cameraToIndexCamera_.end()) {
		return {};
	}
	return indexCameras_[it->second].nameToShape_[shapeName];
}

ref_ptr<std::vector<ref_ptr<BoundingShape>>> SpatialIndex::getShapes(std::string_view shapeName) const {
	auto it = nameToShape_.find(shapeName);
	if (it != nameToShape_.end()) {
		return it->second;
	}
	return {};
}

bool SpatialIndex::isVisible(const Camera &camera, uint32_t layerIdx, std::string_view shapeID) {
	auto it = cameraToIndexCamera_.find(&camera);
	if (it == cameraToIndexCamera_.end()) {
		return true;
	}
	const IndexCamera &ic = indexCameras_[it->second];
	auto it2 = ic.nameToShape_.find(shapeID);
	if (it2 == ic.nameToShape_.end()) {
		return true;
	}
	return it2->second->isVisibleInLayer(layerIdx);
}

GLuint SpatialIndex::numInstances(std::string_view shapeID) const {
	auto shape = getShape(shapeID);
	if (!shape.get()) {
		return 0u;
	}
	return shape->numInstances();
}

ref_ptr<BoundingShape> SpatialIndex::getShape(std::string_view shapeID) const {
	auto it = nameToShape_.find(shapeID);
	if (it != nameToShape_.end() && !it->second->empty()) {
		return it->second->front();
	}
	return {};
}

ref_ptr<BoundingShape> SpatialIndex::getShape(std::string_view shapeID, uint32_t instance) const {
	auto it = nameToShape_.find(shapeID);
	if (it != nameToShape_.end() && !it->second->empty()) {
		for (auto &shape: *it->second.get()) {
			if (instance == shape->instanceID()) {
				return shape;
			}
		}
	}
	return {};
}

static inline uint32_t getLODLevel(
			const BoundingShape &b_shape,
			const IndexedShape *i_shape,
			float lodDistance) {
	return b_shape.baseMesh()->getLODLevel(lodDistance, i_shape->lodShift());
}

inline uint16_t floatToHalf(float distance, SortMode sortMode) {
	uint32_t x = std::bit_cast<uint32_t>(distance);
	uint16_t q = ((x>>16)&0x8000) |                     // sign bit
		   ((((x&0x7f800000)-0x38000000)>>13)&0x7c00) | // exponent bits
		   	((x>>13)&0x03ff);                           // mantissa bits
	// Flip if back-to-front sorting
	if (sortMode == BACK_TO_FRONT) {
		q = 0xFFFF - q;
	}
	return q;
}

inline uint64_t packKey_64bit(uint16_t shapeIdx, uint32_t layerIdx, float distance, SortMode sortMode) {
	// Clamp layerIdx to 16 bits
	const uint16_t layer16 = static_cast<uint16_t>(std::min(layerIdx, 0xFFFFu));
	// Convert distance to uint32_t. For FRONT_TO_BACK, `floatBitsToUint(distance)` is used.
	// In case of BACK_TO_FRONT, we use `0xFFFFFFFFu - floatBitsToUint(distance)` to invert the order.
	int backToFront = int(sortMode == BACK_TO_FRONT);
	const uint32_t distance_ui = (0xFFFFFFFFu * backToFront) +
		conversion::floatBitsToUint(distance) * (1 - 2*backToFront);
	// Pack: upper 16 bits = shapeIdx, next 16 bits = layerIdx, lower 32 bits = distance
	return static_cast<uint64_t>(shapeIdx) << 48 | static_cast<uint64_t>(layer16) << 32 | static_cast<uint64_t>(distance_ui);
}

void SpatialIndex::handleIntersection(const BoundingShape& b_shape, void* userData) {
    auto* data = (SpatialIndex::TraversalData*)(userData);
	auto* i_cam = data->indexCamera;
    auto* i_shape = (IndexedShape*) b_shape.spatialIndexData_[i_cam->camIdx];
    const uint32_t L = i_shape->camera()->numLayer();
	const uint32_t l = data->layerIdx;
	const SortMode sortMode = i_shape->instanceSortMode();

    // toggle visibility for this layer
    i_shape->tmp_layerVisibility_[l] = true;

    // compute LOD level for this shape by distance to camera.
    // each mesh may have its own thresholds for switching LOD levels, so we need
    // to let the (base) mesh decide which LOD level to use.
	const float lodDistance = (b_shape.tfOrigin() - *data->camPos).lengthSquared();
	const uint32_t k = getLODLevel(b_shape, i_shape, lodDistance);

	// Finally bin the shape into the (lod, layer) bin
	const uint32_t b = CullShape::binIdx(k, l, L);
	i_shape->tmp_binCounts_[b] += 1;
	// Total visibility count of the shape across all layers
	i_shape->tmp_totalCount_ += 1;
	i_cam->tmp_layerInstances_.push_back(b_shape.instanceID());
	i_cam->tmp_layerShapes_.push_back(i_cam->tmp_layerShapes_.size());
	i_cam->tmp_sortKeys_.push_back(packKey_64bit(i_shape->shapeIdx_, l, lodDistance, sortMode));
}

void SpatialIndex::updateLayerVisibility(
		IndexCamera &ic, uint32_t layerIdx,
		const BoundingShape &camera_shape) {
	// Collect all intersections for this layer into (lod,layer) bins
	TraversalData traversalData{ this, &ic, nullptr, layerIdx };
	if (ic.sortCamera->position().size()>1) {
		traversalData.camPos = &ic.sortCamera->position(layerIdx);
	} else {
		traversalData.camPos = &ic.sortCamera->position(0);
	}
	foreachIntersection(camera_shape,
		handleIntersection,
		&traversalData,
		ic.traversalMask);
}

static void visibilityJobFunc(void *arg) {
	VisibilityJob::run(arg);
}

void SpatialIndex::resetCamera(IndexCamera *indexCamera, uint32_t traversalMask) {
	if (indexCamera->isDirty) {
		// Compute the number of keys which is the sum of L * I for all shapes
		// assigned to this camera.
		const uint32_t numLayer = indexCamera->cullCamera->numLayer();
		indexCamera->numKeys = 0;
		for (auto &indexShape: indexCamera->indexShapes_) {
			indexCamera->numKeys += indexShape->shape()->numInstances() * numLayer;
		}
		indexCamera->tmp_layerInstances_.reserve(indexCamera->numKeys);
		indexCamera->tmp_layerShapes_.reserve(indexCamera->numKeys);
		indexCamera->tmp_sortKeys_.reserve(indexCamera->numKeys);
		indexCamera->radixSort.resize(indexCamera->numKeys);
		indexCamera->isDirty = false;
	}
	indexCamera->tmp_layerInstances_.clear();
	indexCamera->tmp_layerShapes_.clear();
	indexCamera->tmp_sortKeys_.clear();
	indexCamera->traversalMask = traversalMask;
	auto &frustumShapes = indexCamera->cullCamera->frustum();
	for (auto &shape: frustumShapes) {
		shape.updateOrthogonalProjection();
	}
}

void SpatialIndex::updateVisibility(uint32_t traversalMask) {
	uint32_t numIndexedCameras = indexCameras_.size();
	if (!jobPool_) {
		uint32_t numThreads = numIndexedCameras - 1; // leave one for the local thread
		numThreads = std::max(1u, std::min(numThreads, maxNumThreads_));
		jobPool_ = std::make_unique<JobPool>(numThreads);
		REGEN_DEBUG("Created job pool with " << numThreads << " threads.");
	}

	// Schedule jobs for all index cameras, but keep the one with the most keys for the local thread.
	IndexCamera *localIndexCamera = &indexCameras_.front();
	resetCamera(localIndexCamera, traversalMask);
	for (uint32_t i = 1; i < numIndexedCameras; ++i) {
		IndexCamera *nextIndexCamera = &indexCameras_[i];
		resetCamera(nextIndexCamera, traversalMask);
		if (nextIndexCamera->numKeys > localIndexCamera->numKeys) {
			std::swap(localIndexCamera, nextIndexCamera);
		}
		jobPool_->addJobPreFrame(Job{ .fn = visibilityJobFunc, .arg = nextIndexCamera });
	}

	// Execute jobs
	jobPool_->beginFrame(1u); // one local job
	Job localJob { .fn = visibilityJobFunc, .arg = localIndexCamera };
	do {
		jobPool_->performJob(localJob);
	} while (jobPool_->stealJob(localJob));
	jobPool_->endFrame();
}

void SpatialIndex::updateVisibility(IndexCamera *indexCamera) {
	const uint32_t L = indexCamera->cullCamera->numLayer();

	for (auto &indexShape: indexCamera->indexShapes_) {
		indexShape->tmp_layerVisibility_.assign(L, false);
		// Reset the per-bin counts, we accumulate them during traversal, so they
		// need to be reset first.
		std::memset(indexShape->tmp_binCounts_.data(), 0,
			sizeof(uint32_t) * indexShape->tmp_binCounts_.size());
		// Reset total count
		indexShape->tmp_totalCount_ = 0;
		// Remember the index shape to bounding shape mapping such that we can
		// obtain index shape from bounding shape directly (else a hash lookup would be required).
		for (auto &bs: indexShape->boundingShapes_) {
			// FIXME: This would break in a multi-index scenario where multiple indices
			//        are processed in parallel that contain the same bounding shape.
			//        Would need something like thread_local member variable.
			bs->spatialIndexData_[indexCamera->camIdx] = indexShape;
		}
	}

	// Process each layer
	auto &frustumShapes = indexCamera->cullCamera->frustum();
	for (uint32_t layerIdx = 0; layerIdx < frustumShapes.size(); ++layerIdx) {
		updateLayerVisibility(*indexCamera, layerIdx, frustumShapes[layerIdx]);
	}

	// TODO: With some changes it should be possible to sort instance IDs directly, and to remove tmp_layerShapes_.
	//       Basically we need to store the keys in a way such that we can access them by instance ID.
	std::vector<uint32_t>& vec = indexCamera->tmp_layerShapes_;
	if (vec.size() < SMALL_ARRAY_SIZE) {
		std::ranges::sort(vec, [indexCamera](uint32_t itemIdx1, uint32_t itemIdx2) {
			return indexCamera->tmp_sortKeys_[itemIdx1] < indexCamera->tmp_sortKeys_[itemIdx2];
		});
	} else {
		indexCamera->radixSort.sort(vec, indexCamera->tmp_sortKeys_);
	}

	uint32_t shapeBase = 0;
	for (auto &indexShape: indexCamera->indexShapes_) {
		if (indexShape->tmp_totalCount_ == 0) {
			// No visible instances for this shape
			// Mark all layers as invisible
			indexShape->visible_.assign(L, false);
			indexShape->isVisibleInAnyLayer_ = false;
			continue;
		}
	    const uint32_t K = indexShape->numLODs();

	    // Compute base offsets for all bins in LOD-major order
	    uint32_t runningBase = 0;
	    for (uint32_t b = 0; b < indexShape->tmp_binBase_.size(); ++b) {
			indexShape->tmp_binBase_[b] = runningBase;
			runningBase += indexShape->tmp_binCounts_[b];
		}

		indexShape->mapInstanceData_internal();
	    auto mapped_ids = indexShape->mappedInstanceIDs();
	    auto mapped_counts = indexShape->mappedInstanceCounts();
	    auto mapped_base = indexShape->mappedBaseInstance();

		// Copy over the visibility flags from tmp_layerVisibility_ into visible_
	    bool isVisible = false;
		for (uint32_t layer = 0; layer < L; ++layer) {
			bool visibleInLayer = indexShape->tmp_layerVisibility_[layer];
			indexShape->visible_[layer] = visibleInLayer;
			if (!isVisible && visibleInLayer) { isVisible = true; }
		}
		indexShape->isVisibleInAnyLayer_ = isVisible;

		// Copy over the counts and base offsets from tmp_ arrays into the mapped arrays
		// TODO: Rather directly write into mapped arrays during traversal to avoid this copy.
		//       With frame-locking and double buffer this should not be slower.
		const uint32_t numBytes = sizeof(uint32_t) * indexShape->tmp_binCounts_.size();
		std::memcpy(mapped_counts, indexShape->tmp_binCounts_.data(), numBytes);
		std::memcpy(mapped_base, indexShape->tmp_binBase_.data(), numBytes);

	    // Write IDs to mapped buffer
		// Each shape has a fixed contiguous region in tmp_layerShapes_ starting at some offset.
		runningBase = shapeBase;
		for (uint32_t l = 0; l < L; ++l) {
			for (uint32_t k = 0; k < K; ++k) {
				const uint32_t b = CullShape::binIdx(k, l, L);
				const uint32_t base = indexShape->tmp_binBase_[b];
				const uint32_t cnt = indexShape->tmp_binCounts_[b];
				if (cnt == 0) continue;

    			std::transform(
    				vec.begin() + runningBase,
    				vec.begin() + runningBase + cnt,
    				mapped_ids + base,
    				[&](uint32_t shapeIdx) { return indexCamera->tmp_layerInstances_[shapeIdx]; });
				runningBase += cnt;
			}
		}

		indexShape->unmapInstanceData_internal();
		shapeBase += indexShape->tmp_totalCount_;
	}
}

void SpatialIndex::addDebugShape(const ref_ptr<BoundingShape> &shape) {
	debugShapes_.push_back(shape);
}

void SpatialIndex::removeDebugShape(const ref_ptr<BoundingShape> &shape) {
	auto it = std::find(debugShapes_.begin(), debugShapes_.end(), shape);
	if (it != debugShapes_.end()) {
		debugShapes_.erase(it);
	}
}

void SpatialIndex::debugBoundingShape(DebugInterface &debug, const BoundingShape &shape) const {
	SpatialIndexDebug &sid = static_cast<SpatialIndexDebug &>(debug);
	switch (shape.shapeType()) {
		case BoundingShapeType::BOX: {
			auto box = static_cast<const BoundingBox *>(&shape);
			sid.drawBox(*box);
			break;
		}
		case BoundingShapeType::SPHERE: {
			auto sphere = static_cast<const BoundingSphere *>(&shape);
			sid.drawSphere(*sphere);
			break;
		}
		case BoundingShapeType::FRUSTUM:
			auto frustum = static_cast<const Frustum *>(&shape);
			sid.drawFrustum(*frustum);
			break;
	}
}

void SpatialIndex::debugDraw(DebugInterface &debug) const {
	SpatialIndexDebug &sid = static_cast<SpatialIndexDebug &>(debug);
	for (auto &shape: shapes()) {
		for (auto &instance: *shape.second.get()) {
			if (instance->traversalMask() == 0) continue;
			debugBoundingShape(debug, *instance.get());
		}
	}
	for (auto &shape : debugShapes_) {
		if (shape->traversalMask() == 0) continue;
		debugBoundingShape(debug, *shape.get());
	}
	for (auto &camera: cameras()) {
		for (auto &frustum: camera->frustum()) {
			sid.debugFrustum(frustum, Vec3f(1.0f, 0.0f, 1.0f));
		}
	}
}

ref_ptr<SpatialIndex> SpatialIndex::load(LoadingContext &ctx, scene::SceneInputNode &input) {
	auto indexType = input.getValue<std::string>("type", "quadtree");
	ref_ptr<SpatialIndex> index;

	if (indexType == "quadtree") {
		auto quadTree = ref_ptr<QuadTree>::alloc();
		//quadTree->setMaxObjectsPerNode(input.getValue<GLuint>("max-objects-per-node", 4u));
		quadTree->setMinNodeSize(input.getValue<float>("min-node-size", 0.1f));

		if (input.hasAttribute("test-mode-3d")) {
			auto testMode = input.getValue("test-mode-3d");
			if (testMode == "closest") {
				quadTree->setTestMode3D(QuadTree::QUAD_TREE_3D_TEST_CLOSEST);
			} else if (testMode == "all") {
				quadTree->setTestMode3D(QuadTree::QUAD_TREE_3D_TEST_ALL);
			} else {
				quadTree->setTestMode3D(QuadTree::QUAD_TREE_3D_TEST_NONE);
			}
		}
		if (input.hasAttribute("close-distance")) {
			auto dst = input.getValue<float>("close-distance", 20.0f);
			dst = std::max(0.0f, dst);
			quadTree->setCloseDistanceSquared(dst * dst);
		}
		if (input.hasAttribute("subdivision-threshold")) {
			quadTree->setSubdivisionThreshold(input.getValue<GLuint>("subdivision-threshold", 4u));
		}

		index = quadTree;
	}

	return index;
}
