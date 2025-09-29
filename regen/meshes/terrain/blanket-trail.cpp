#include "blanket-trail.h"

#include "regen/meshes/mesh-vector.h"

using namespace regen;

static uint32_t computeMaxInstances(const BlanketTrail::TrailConfig &cfg) {
	uint32_t maxInstances = cfg.numTrails;
	if (cfg.blanketLifetime > 0.0f && cfg.blanketFrequency > 0.0f) {
		// compute number of instances based on walk speed and lifetime
		maxInstances += static_cast<uint32_t>(cfg.numTrails *
			std::ceil(2.0 * cfg.blanketLifetime / cfg.blanketFrequency));
	}
	return maxInstances;
}

BlanketTrail::BlanketTrail(
	const ref_ptr<Ground> &groundMesh,
	const TrailConfig &cfg) : Blanket(cfg, computeMaxInstances(cfg)) {
	groundMesh_ = groundMesh;
	insertHeight_ = groundMesh_->mapCenter()->getVertex(0).r.y -
			0.5f*groundMesh_->mapSize()->getVertex(0).r.y;
}

void BlanketTrail::setBlanketMask(const ref_ptr<Texture> &tex) {
	blanketMask_ = tex;
	if (!blanketMask_) {
		return;
	}
	blanketMaskState_ = ref_ptr<TextureState>::alloc(blanketMask_, "blanketMask");
	blanketMaskState_->set_mapTo(TextureState::MAP_TO_CUSTOM);
	blanketMaskState_->set_mapping(TextureState::MAPPING_CUSTOM);
	joinStates(blanketMaskState_);

	uint32_t numMaskTextures = tex->depth();
	if (numMaskTextures > 1) {
		maskIndex_.resize(numBlankets_, 0);
		u_maskIndex_ = ref_ptr<ShaderInput1ui>::alloc("maskIndex");
		u_maskIndex_->setInstanceData(numBlankets_, 1, (byte*)maskIndex_.data());
		setInput(u_maskIndex_);
	} else {
		maskIndex_.clear();
		u_maskIndex_ = {};
	}
}

void BlanketTrail::setModelTransform(const ref_ptr<ModelTransformation> &tf) {
	tf_ = tf;

	// initialize TF to num instances
	if (tf_.get()) {
		auto modelMat = tf_->modelMat();
		modelMat->setInstanceData(numBlankets_, 1, nullptr);
		// distribute initially along z axis
		auto modelData = (Mat4f*)modelMat->clientData();
		for (uint32_t i = 0; i < numBlankets_; ++i) {
			modelData[i] = Mat4f::identity();
			modelData[i].translate(Vec3f(0.0f, -32.0f, static_cast<float>(i) * 3.0f));
		}
		tf_->set_numInstances(numBlankets_);
	}

	joinStates(tf);
}

void BlanketTrail::insertBlanket(const Vec3f &pos, const Vec3f &dir, uint32_t maskIndex) {
	uint32_t instanceIdx = reviveBlanket();

	// update TF
	Quaternion q(0.0, 0.0, 0.0, 1.0);
	q.setEuler(dir.x, dir.y, dir.z);
	tmpMat_ = q.calculateMatrix();
	tmpMat_.translate(Vec3f(pos.x, insertHeight_, pos.z));
	tf_->setModelMat(instanceIdx, tmpMat_);

	// update mask index
	if (!maskIndex_.empty() && maskIndex_[instanceIdx] != maskIndex) {
		maskIndex_[instanceIdx] = maskIndex;
		u_maskIndex_->setVertex(instanceIdx, maskIndex);
	}
}

ref_ptr<BlanketTrail> BlanketTrail::load(LoadingContext &ctx,
			scene::SceneInputNode &input,
			const std::vector<GLuint> &lodLevels) {
	auto scene = ctx.scene();
	ref_ptr<ModelTransformation> tf;
	ref_ptr<State> dummy = ref_ptr<State>::alloc();

	std::vector<ref_ptr<scene::SceneInputNode>> handledChildren;
	for (auto &n: input.getChildren()) {
		if (n->getCategory() == "transform") {
			handledChildren.push_back(n);
			// load the model transformation
			tf = ModelTransformation::load(ctx, *n.get(), dummy);
			scene->putResource("ModelTransformation", n->getValue("id"), tf);
		}
	}
	for (auto &n: handledChildren) {
		// make sure mesh loading does not attempt to load materials again (this will cause a warning)
		input.removeChild(n);
	}
	if (!tf.get()) {
		tf = ref_ptr<ModelTransformation>::alloc();
	}

	ref_ptr<Ground> groundMesh;
	auto meshVec = ctx.scene()->getResource<MeshVector>(input.getValue("ground-mesh"));
	if (meshVec.get() != nullptr && meshVec->size() > 0) {
		auto mesh = (*meshVec.get())[0];
		groundMesh = ref_ptr<Ground>::dynamicCast(mesh);
	}
	if (!groundMesh) {
		REGEN_ERROR("No valid ground mesh found for blanket trail in " << input.getDescription() << ".");
		return {};
	}

	BufferUpdateFlags updateFlags;
	updateFlags.frequency = input.getValue<BufferUpdateFrequency>("update-frequency", BUFFER_UPDATE_NEVER);
	updateFlags.scope = input.getValue<BufferUpdateScope>("update-scope", BUFFER_UPDATE_FULLY);
	BlanketTrail::TrailConfig meshCfg;
	meshCfg.levelOfDetails = lodLevels;
	meshCfg.texcoScale = input.getValue<Vec2f>("texco-scaling", Vec2f(1.0f));
	meshCfg.blanketSize = input.getValue<Vec2f>("blanket-size", Vec2f(1.0f, 1.0f));
	meshCfg.isInitiallyDead = input.getValue<bool>("initially-dead", false);
	meshCfg.blanketLifetime = input.getValue<GLfloat>("blanket-lifetime", 0.0f);
	meshCfg.updateHint = updateFlags;
	meshCfg.mapMode = input.getValue<BufferMapMode>("map-mode", BUFFER_MAP_DISABLED);
	meshCfg.accessMode = input.getValue<ClientAccessMode>("access-mode", BUFFER_CPU_WRITE);
	meshCfg.numTrails = input.getValue<GLuint>("blanket-trails", 1u);
	meshCfg.blanketFrequency = input.getValue<GLfloat>("blanket-frequency", 0.5f);
	auto blanket = ref_ptr<BlanketTrail>::alloc(groundMesh, meshCfg);
	blanket->setModelTransform(tf);

	auto maskNode = input.getFirstChild("blanket-mask");
	if (maskNode.get()) {
		ref_ptr<Texture> blanketMask;
		// avoid re-processing
		input.removeChild(maskNode);

		if (maskNode->hasAttribute("fbo")) {
			auto fboName = maskNode->getValue<std::string>("fbo", "");
			auto fbo = ctx.scene()->getResource<FBO>(fboName);
			if (fbo.get()) {
				uint32_t attachmentIdx = maskNode->getValue<uint32_t>("attachment", 0);
				blanketMask = fbo->colorTextures()[attachmentIdx];
			}
			if (!blanketMask.get()) {
				REGEN_WARN("No valid FBO/attachment found for blanket mask in " << maskNode->getDescription() << ".");
			}
		}
		if (blanketMask.get()) {
			blanket->setBlanketMask(blanketMask);
		}
	}
	groundMesh->setupGroundBlanket(*blanket.get());
	blanket->updateAttributes();
	return blanket;
}
