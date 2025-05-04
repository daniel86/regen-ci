#include <iostream>

#include "mesh-viewer-widget.h"
#include <QtWidgets/QFileDialog>
#include <QtCore/QList>
#include <QtCore/QDirIterator>
#include <QtEvents>
#include <boost/filesystem/path.hpp>
#include <assimp/postprocess.h>

#include <regen/states/shader-state.h>
#include <regen/states/fbo-state.h>
#include <regen/states/blit-state.h>
#include <regen/states/state-configurer.h>
#include <regen/utility/filesystem.h>
#include <regen/animations/animation-manager.h>
#include <regen/meshes/lod/mesh-simplifier.h>
#include <regen/states/direct-shading.h>
#include <applications/qt/qt-camera-events.h>
#include <applications/qt/ColorWidget.h>

using namespace std;

// Resizes Framebuffer texture when the window size changed
class FBOResizer : public EventHandler {
public:
	explicit FBOResizer(const ref_ptr<FBOState> &fbo) : EventHandler(), fboState_(fbo) {}
	void call(EventObject *evObject, EventData *) override {
		auto *app = (Application *) evObject;
		auto winSize = app->windowViewport()->getVertex(0);
		fboState_->resize(winSize.r.x, winSize.r.y);
	}
protected:
	ref_ptr<FBOState> fboState_;
};

class RotateAnimation : public Animation {
public:
	explicit RotateAnimation(MeshViewerWidget *widget)
			: Animation(false, true), widget_(widget) {}
	void animate(GLdouble dt) override { widget_->transformMesh(dt); }
	MeshViewerWidget *widget_;
};

template<class WidgetType>
QWidget *createParameterWidget(QtApplication *app, QWidget *parent,
		const ref_ptr<StateNode> &node,
		const ref_ptr<ShaderInput> &input) {
	auto *widget = new WidgetType({app, node, input}, parent);
	widget->show();
	return widget;
}

static void hideLayout(QLayout *layout) {
	for (GLint i = 0; i < layout->count(); ++i) {
		QLayoutItem *item = layout->itemAt(i);
		if (item->widget()) { item->widget()->hide(); }
		if (item->layout()) { hideLayout(item->layout()); }
	}
}

static void showLayout(QLayout *layout) {
	for (GLint i = 0; i < layout->count(); ++i) {
		QLayoutItem *item = layout->itemAt(i);
		if (item->widget()) { item->widget()->show(); }
		if (item->layout()) { showLayout(item->layout()); }
	}
}

MeshViewerWidget::MeshViewerWidget(QtApplication *app)
		: QMainWindow(),
		  app_(app),
		  wereControlsShown_(false),
		  settings_("regen-mesh-viewer") {
	setMouseTracking(true);
	setAcceptDrops(true);
	controlsShown_ = true;
	sceneRoot_ = ref_ptr<StateNode>::alloc();
	meshRoot_ = ref_ptr<StateNode>::alloc();

	ui_.setupUi(this);
	ui_.loadMeshButton->setEnabled(false);
	ui_.meshIndexCombo->setEnabled(false);
	ui_.splitter->setSizes(QList<int>({INT_MAX, INT_MAX}));
	// load initial settings
	ui_.simplifyCheckBox->setChecked(settings_.value("simplify", true).toBool());
	ui_.simplify1Spin->setValue(settings_.value("simplify1", 0.7).toFloat());
	ui_.simplify2Spin->setValue(settings_.value("simplify2", 0.4).toFloat());
	ui_.simplify3Spin->setValue(settings_.value("simplify3", 0.1).toFloat());
	ui_.norMaxAngleSpin->setValue(settings_.value("norMaxAngle", 0.6).toFloat());
	ui_.norPenaltySpin->setValue(settings_.value("norPenalty", 0.1).toFloat());
	ui_.valencePenaltySpin->setValue(settings_.value("valencePenalty", 0.1).toFloat());
	ui_.areaPenaltySpin->setValue(settings_.value("areaPenalty", 0.1).toFloat());
	ui_.genNorCheck->setCheckState(
			settings_.value("genNorCheck", false).toBool() ? Qt::Checked : Qt::Unchecked);
	ui_.fixNorCheck->setCheckState(
			settings_.value("fixNorCheck", false).toBool() ? Qt::Checked : Qt::Unchecked);
	ui_.genUvCheck->setCheckState(
			settings_.value("genUvCheck", true).toBool() ? Qt::Checked : Qt::Unchecked);
	ui_.flipUvCheck->setCheckState(
			settings_.value("flipUvCheck", true).toBool() ? Qt::Checked : Qt::Unchecked);
	ui_.deboneCheck->setCheckState(
			settings_.value("deboneCheck", false).toBool() ? Qt::Checked : Qt::Unchecked);
	ui_.limitBonesCheck->setCheckState(
			settings_.value("limitBonesCheck", false).toBool() ? Qt::Checked : Qt::Unchecked);
	ui_.optimizeMeshesCheck->setCheckState(
			settings_.value("optimizeMeshesCheck", true).toBool() ? Qt::Checked : Qt::Unchecked);
	ui_.optimizeGraphCheck->setCheckState(
			settings_.value("optimizeGraphCheck", true).toBool() ? Qt::Checked : Qt::Unchecked);
	auto lastPath = settings_.value("lastPath", "").toString();
	if (!lastPath.isEmpty()) {
		assetFilePath_ = lastPath.toStdString();
		auto assetFilename = boost::filesystem::path(assetFilePath_).filename().string();
		ui_.assetNameEdit->setText(QString::fromStdString(assetFilename));
		ui_.loadMeshButton->setEnabled(true);
	}
	ui_.glWidgetLayout->addWidget(app_->glWidgetContainer(), 0, 0, 1, 1);

	fullscreenLayout_ = new QVBoxLayout();
	fullscreenLayout_->setObjectName(QString::fromUtf8("fullscreenLayout"));
	ui_.gridLayout_2->addLayout(fullscreenLayout_, 0, 0, 1, 1);
	hideLayout(fullscreenLayout_);

	resize(1600, 1200);
	updateSize();
	app_->withGLContext([this]() { gl_loadScene(); });
}

void MeshViewerWidget::simplifyMesh_GL(const Vec3f &thresholds) {
	for (auto &mesh: meshes_) {
		MeshSimplifier simplifier(mesh);
		simplifier.setThresholds(thresholds.x, thresholds.y);
		simplifier.setNormalMaxAngle(static_cast<float>(ui_.norMaxAngleSpin->value()));
		simplifier.setNormalPenalty(static_cast<float>(ui_.norPenaltySpin->value()));
		simplifier.setValencePenalty(static_cast<float>(ui_.valencePenaltySpin->value()));
		simplifier.setAreaPenalty(static_cast<float>(ui_.areaPenaltySpin->value()));
		simplifier.simplifyMesh();
	}
}

void MeshViewerWidget::updateLoDButtons() {
	if (meshes_.empty()) {
		ui_.lodControls->setEnabled(false);
		return;
	}
	ui_.lodControls->setEnabled(true);
	auto &firstMesh = meshes_[0];
	auto numLODs = firstMesh->numLODs();
	ui_.lod0Button->setEnabled(true);
	ui_.lod1Button->setEnabled(numLODs > 1);
	ui_.lod2Button->setEnabled(numLODs > 2);
	ui_.lod3Button->setEnabled(numLODs > 3);
}

void MeshViewerWidget::loadMeshes_GL(const std::string &assetPath) {
	static const BufferUsage vboUsage = BUFFER_USAGE_STATIC_DRAW;
	auto p = resourcePath(assetPath);
	// Read values from UI spinner
	bool simplify = ui_.simplifyCheckBox->isChecked();
	// Apply transform to model on import
	Mat4f transform = Mat4f::identity();
	asset_ = ref_ptr<AssetImporter>::alloc(p);
	asset_->setImportFlag(AssetImporter::IGNORE_NORMAL_MAP);
	asset_->importAsset();
	meshes_ = asset_->loadAllMeshes(transform, vboUsage);
	if (meshes_.empty()) {
		REGEN_WARN("No meshes loaded from " << p);
		// remove all elements from the mesh index combo box and disable it
		ui_.meshIndexCombo->clear();
		ui_.meshIndexCombo->setEnabled(false);
	} else {
		// compute mesh scale based on its bounding box. scale such that it is 1 unit in y direction.
		auto &firstMesh = meshes_[0];
		Bounds<Vec3f> bounds(firstMesh->minPosition(), firstMesh->maxPosition());
		for (uint32_t i = 1; i < meshes_.size(); ++i) {
			auto &mesh = meshes_[i];
			bounds.min.setMin(mesh->minPosition());
			bounds.max.setMax(mesh->maxPosition());
		}
		meshScale_ = 1.0f / (bounds.max.y - bounds.min.y);
		// Add mesh indexes to the combo box, select "0" by default
		ui_.meshIndexCombo->clear();
		for (uint32_t i = 0; i < meshes_.size(); ++i) {
			ui_.meshIndexCombo->addItem(QString::fromStdString(REGEN_STRING(i)));
		}
		ui_.meshIndexCombo->setEnabled(true);
		ui_.meshIndexCombo->setCurrentIndex(0);

		if (simplify) {
			auto lod0 = static_cast<float>(ui_.simplify1Spin->value());
			auto lod1 = static_cast<float>(ui_.simplify2Spin->value());
			auto lod2 = static_cast<float>(ui_.simplify3Spin->value());
			simplifyMesh_GL(Vec3f(
					lod0,
					lod1 < lod0 ? lod1 : 0.0f,
					lod2 < lod1 ? lod2 : 0.0f));
		}
		loadResources_GL();
		updateLoDButtons();
		selectMesh(0, 0);
		transformMesh(0.0f);
	}
}

void MeshViewerWidget::loadResources_GL() {
	meshRoot_->clear();
	meshNodes_.clear();

	for (auto &mesh: meshes_) {
		auto material = asset_->getMeshMaterial(mesh.get());
		if (material.get() != nullptr) {
			mesh->joinStates(material);
		}

		auto shaderState = ref_ptr<ShaderState>::alloc();
		mesh->joinStates(shaderState);
		auto meshNode = ref_ptr<StateNode>::alloc(mesh);
		meshNode->set_isHidden(true);
		meshRoot_->addChild(meshNode);
		meshNodes_.push_back(meshNode);

		StateConfigurer shaderConfigurer;
		shaderConfigurer.define("OUTPUT_TYPE", "DIRECT");
		shaderConfigurer.addNode(meshNode.get());
		shaderState->createShader(shaderConfigurer.cfg(), "regen.models.mesh");
		mesh->updateVAO(RenderState::get(), shaderConfigurer.cfg(), shaderState->shader());
	}
}

void MeshViewerWidget::selectMesh(uint32_t meshIndex, uint32_t lodIndex) {
	// TODO: Support mesh animations, play the animation from begin to end in a loop.
	for (auto &meshNode: meshNodes_) {
		meshNode->set_isHidden(true);
	}
	if (meshIndex < meshNodes_.size()) {
		auto &mesh = meshes_[meshIndex];
		auto numLODs = mesh->numLODs();
		if (numLODs > 1) {
			if (lodIndex < numLODs) {
				mesh->activateLOD(lodIndex);
			} else {
				REGEN_WARN("Invalid LOD index " << lodIndex << " for mesh " << meshIndex);
				mesh->activateLOD(0);
			}
		}
		meshNodes_[meshIndex]->set_isHidden(false);
	}
	currentMeshIndex_ = meshIndex;
	currentLodLevel_ = lodIndex;
}

static ref_ptr<Camera> createUserCamera(const Vec2i &viewport) {
	auto cam = ref_ptr<Camera>::alloc(1);
	float aspect = (GLfloat) viewport.x / (GLfloat) viewport.y;
	cam->set_isAudioListener(false);
	cam->position()->setVertex(0, Vec3f(0.0f, 0.0f, -3.0f));
	cam->direction()->setVertex(0, Vec3f(0.0f, 0.0f, 1.0f));
	cam->setPerspective(aspect, 45.0f, 0.1f, 100.0f);
	cam->updateCamera();
	return cam;
}

void MeshViewerWidget::createCameraController() {
	cameraController_ = ref_ptr<CameraController>::alloc(userCamera_);
	cameraController_->set_moveAmount(0.01f);
	cameraController_->setHorizontalOrientation(0.0);
	cameraController_->setVerticalOrientation(0.0);
	cameraController_->setCameraMode(CameraController::FIRST_PERSON);
	cameraController_->startAnimation();

	std::vector<CameraCommandMapping> keyMappings;
	ref_ptr<QtFirstPersonEventHandler> cameraEventHandler =
			ref_ptr<QtFirstPersonEventHandler>::alloc(cameraController_, keyMappings);
	cameraEventHandler->set_sensitivity(0.005f);
	app_->connect(Application::KEY_EVENT, cameraEventHandler);
	app_->connect(Application::BUTTON_EVENT, cameraEventHandler);
	app_->connect(Application::MOUSE_MOTION_EVENT, cameraEventHandler);
}

void MeshViewerWidget::gl_loadScene() {
	AnimationManager::get().pause(GL_TRUE);
	AnimationManager::get().setRootState(app_->renderTree()->state());
	// TODO: why needed? seems it is initialized once and then GL context is switched?
	RenderState::reset();

	// create render target
	auto fbo = ref_ptr<FBO>::alloc(ui_.glWidget->width(), ui_.glWidget->height());
	fbo->addTexture(1, GL_TEXTURE_2D, GL_RGBA, GL_RGBA, GL_UNSIGNED_BYTE);
	fbo->createDepthTexture(GL_TEXTURE_2D, GL_DEPTH_COMPONENT24, GL_UNSIGNED_BYTE);
	auto fboState = ref_ptr<FBOState>::alloc(fbo);
	fboState->setClearDepth();
	fboState->setClearColor({
		Vec4f(0.26, 0.26, 0.36, 1.0),
		GL_COLOR_ATTACHMENT0 });
	fboState->setDrawBuffers({GL_COLOR_ATTACHMENT0});

	// create a root node
	sceneRoot_->state()->joinStates(fboState);
	sceneRoot_->addChild(meshRoot_);
	app_->renderTree()->addChild(sceneRoot_);
	// enable user camera
	userCamera_ = createUserCamera(app_->windowViewport()->getVertex(0).r);
	sceneRoot_->state()->joinStates(userCamera_);
	// enable model transformation
	modelTransform_ = ref_ptr<ModelTransformation>::alloc();
	sceneRoot_->state()->joinStates(modelTransform_);
	// enable wireframe mode
	wireframeState_ = ref_ptr<FillModeState>::alloc(GL_LINE);
	//wireframeState_->set_isHidden(true);
	sceneRoot_->state()->joinStates(wireframeState_);
	// enable light, and use it in direct shading
	// TODO: better use deferred shading, and also allow to display the normals
	auto shadingState = ref_ptr<DirectShading>::alloc();
	shadingState->ambientLight()->setVertex(0, Vec3f(0.3f));
	sceneLight_ = ref_ptr<Light>::alloc(Light::DIRECTIONAL);
	sceneLight_->set_isAttenuated(false);
	sceneLight_->direction()->setVertex(0, Vec3f(1.0f, 0.0f, 0.0f).normalize());
	sceneLight_->diffuse()->setVertex(0, Vec3f(0.6f, 0.6f, 0.6f));
	sceneLight_->specular()->setVertex(0, Vec3f(0.2f));
	shadingState->addLight(sceneLight_);
	sceneRoot_->state()->joinStates(shadingState);
	// finally, enable a blit state to copy the framebuffer to the screen
	sceneRoot_->state()->joinStates(ref_ptr<BlitToScreen>::alloc(
			fbo, app_->windowViewport(), GL_COLOR_ATTACHMENT0));
	GL_ERROR_LOG();

	// resize fbo with window
	app_->connect(Application::RESIZE_EVENT, ref_ptr<FBOResizer>::alloc(fboState));
	// Update frustum when window size changes
	app_->connect(Application::RESIZE_EVENT,
			ref_ptr<ProjectionUpdater>::alloc(userCamera_, app_->windowViewport()));

	AnimationManager::get().resume();
	REGEN_INFO("Scene Loaded.");
	createCameraController();
}

void MeshViewerWidget::transformMesh(GLdouble dt) {
	auto &tf = modelTransform_->get();
	// rotate the mesh around the Y axis
	meshOrientation_ += dt * 0.001f;
	if (meshOrientation_ > M_PI * 2.0f) {
		meshOrientation_ -= M_PI * 2.0f;
	}
	meshQuaternion_.setAxisAngle(Vec3f::up(), meshOrientation_);
	auto mat = meshQuaternion_.calculateMatrix();
	mat.scale(Vec3f(meshScale_));
	tf->setVertex(0, mat);
}

void MeshViewerWidget::toggleInputsDialog() {
	if (inputDialog_ == nullptr) {
		inputDialog_ = new QDialog(this);
		inputDialog_->setWindowTitle("ShaderInput Editor");
		inputDialog_->resize(1000, 800);

		auto *gridLayout = new QGridLayout(inputDialog_);
		gridLayout->setContentsMargins(0, 0, 0, 0);
		gridLayout->setObjectName(QString::fromUtf8("gridLayout"));

		inputWidget_ = new ShaderInputWidget(app_, inputDialog_);
		gridLayout->addWidget(inputWidget_);
	}
	if (inputDialog_->isVisible()) {
		inputDialog_->hide();
	} else {
		inputWidget_->setNode(app_->renderTree());
		inputDialog_->show();
	}
}

//////////////////////////////
//////// Slots
//////////////////////////////

void MeshViewerWidget::loadMeshes() {
	app_->withGLContext([this]() {
		loadMeshes_GL(assetFilePath_);
	});
	// store the settings
	settings_.setValue("lastPath", QString::fromStdString(assetFilePath_));
	settings_.setValue("simplify", ui_.simplifyCheckBox->isChecked());
	settings_.setValue("simplify1", ui_.simplify1Spin->value());
	settings_.setValue("simplify2", ui_.simplify2Spin->value());
	settings_.setValue("simplify3", ui_.simplify3Spin->value());
	settings_.setValue("norMaxAngle", ui_.norMaxAngleSpin->value());
	settings_.setValue("norPenalty", ui_.norPenaltySpin->value());
	settings_.setValue("valencePenalty", ui_.valencePenaltySpin->value());
	settings_.setValue("areaPenalty", ui_.areaPenaltySpin->value());
	settings_.setValue("genNorCheck", ui_.genNorCheck->checkState() == Qt::Checked);
	settings_.setValue("fixNorCheck", ui_.fixNorCheck->checkState() == Qt::Checked);
	settings_.setValue("genUvCheck", ui_.genUvCheck->checkState() == Qt::Checked);
	settings_.setValue("flipUvCheck", ui_.flipUvCheck->checkState() == Qt::Checked);
	settings_.setValue("deboneCheck", ui_.deboneCheck->checkState() == Qt::Checked);
	settings_.setValue("limitBonesCheck", ui_.limitBonesCheck->checkState() == Qt::Checked);
	settings_.setValue("optimizeMeshesCheck", ui_.optimizeMeshesCheck->checkState() == Qt::Checked);
	settings_.setValue("optimizeGraphCheck", ui_.optimizeGraphCheck->checkState() == Qt::Checked);
	settings_.sync();
}

void MeshViewerWidget::openAssetFile() {
	QFileDialog dialog(nullptr);
	dialog.setFileMode(QFileDialog::AnyFile);
	dialog.setViewMode(QFileDialog::Detail);
	if (!dialog.exec()) { return; }
	auto fileNames = dialog.selectedFiles();
	assetFilePath_ = fileNames.first().toStdString();
	auto assetFilename = boost::filesystem::path(assetFilePath_).filename().string();
	ui_.assetNameEdit->setText(QString::fromStdString(assetFilename));
	ui_.loadMeshButton->setEnabled(true);
}

void MeshViewerWidget::activateLoD(int lodLevel) {
	if (meshes_.empty()) {
		REGEN_WARN("No meshes loaded.");
		return;
	}
	auto &firstMesh = meshes_[currentMeshIndex_];
	auto numLODs = firstMesh->numLODs();
	if (lodLevel > numLODs) {
		REGEN_WARN("Invalid LOD level " << lodLevel);
		lodLevel = 0;
	}
	firstMesh->activateLOD(lodLevel);
}

void MeshViewerWidget::activateLoD_0() {
	activateLoD(0);
}

void MeshViewerWidget::activateLoD_1() {
	activateLoD(1);
}

void MeshViewerWidget::activateLoD_2() {
	activateLoD(2);
}

void MeshViewerWidget::activateLoD_3() {
	activateLoD(3);
}

void MeshViewerWidget::activateMeshIndex(int index) {
	if (index < 0 || index >= meshes_.size()) {
		REGEN_WARN("Invalid mesh index " << index);
		return;
	}
	REGEN_INFO("Activating mesh " << index);
	selectMesh(index, currentLodLevel_);
}

void MeshViewerWidget::updateSize() {
	auto w = ui_.blackBackground->width();
	auto h = ui_.blackBackground->height();
	ui_.glWidget->setMinimumSize(QSize(max(2, w), max(2, h)));
}

void MeshViewerWidget::toggleControls() {
	controlsShown_ = !controlsShown_;
	if (controlsShown_) {
		ui_.buttonFrameBar->show();
		ui_.statusbar->show();

		hideLayout(fullscreenLayout_);
		ui_.splitter->addWidget(ui_.blackBackground);
		ui_.splitter->addWidget(ui_.controls);
		showLayout(ui_.mainLayout);
		ui_.blackBackground->show();
		ui_.splitter->setSizes(splitterSizes_);
	} else {
		splitterSizes_ = ui_.splitter->sizes();

		ui_.buttonFrameBar->hide();
		ui_.statusbar->hide();

		hideLayout(ui_.mainLayout);
		fullscreenLayout_->addWidget(ui_.blackBackground);
		showLayout(fullscreenLayout_);
		ui_.blackBackground->show();
	}
}

void MeshViewerWidget::toggleFullscreen() {
	app_->toggleFullscreen();
	// hide controls when the window is fullscreen
	if (isFullScreen()) {
		wereControlsShown_ = controlsShown_;
		if (controlsShown_) toggleControls();
	} else {
		if (wereControlsShown_) toggleControls();
	}
}

void MeshViewerWidget::toggleWireframe(bool isEnabled) {
	wireframeState_->set_isHidden(!isEnabled);
}

void MeshViewerWidget::toggleRotate(bool isEnabled) {
	if (isEnabled) {
		rotateAnim_ = ref_ptr<RotateAnimation>::alloc(this);
		rotateAnim_->startAnimation();
	} else {
		rotateAnim_->stopAnimation();
	}
}

void MeshViewerWidget::setAssImpFlags() {
	if (ui_.genNorCheck->checkState() == Qt::Checked) {
		asset_->setAiProcessFlag(aiProcess_GenNormals);
	} else {
		asset_->unsetAiProcessFlag(aiProcess_GenNormals);
	}
	if (ui_.fixNorCheck->checkState() == Qt::Checked) {
		asset_->setAiProcessFlag(aiProcess_FixInfacingNormals);
	} else {
		asset_->unsetAiProcessFlag(aiProcess_FixInfacingNormals);
	}
	if (ui_.genUvCheck->checkState() == Qt::Checked) {
		asset_->setAiProcessFlag(aiProcess_GenUVCoords);
	} else {
		asset_->unsetAiProcessFlag(aiProcess_GenUVCoords);
	}
	if (ui_.flipUvCheck->checkState() == Qt::Checked) {
		asset_->setAiProcessFlag(aiProcess_FlipUVs);
	} else {
		asset_->unsetAiProcessFlag(aiProcess_FlipUVs);
	}
	if (ui_.deboneCheck->checkState() == Qt::Checked) {
		asset_->setAiProcessFlag(aiProcess_Debone);
	} else {
		asset_->unsetAiProcessFlag(aiProcess_Debone);
	}
	if (ui_.limitBonesCheck->checkState() == Qt::Checked) {
		asset_->setAiProcessFlag(aiProcess_LimitBoneWeights);
	} else {
		asset_->unsetAiProcessFlag(aiProcess_LimitBoneWeights);
	}
	if (ui_.optimizeMeshesCheck->checkState() == Qt::Checked) {
		asset_->setAiProcessFlag(aiProcess_OptimizeMeshes);
	} else {
		asset_->unsetAiProcessFlag(aiProcess_OptimizeMeshes);
	}
	if (ui_.optimizeGraphCheck->checkState() == Qt::Checked) {
		asset_->setAiProcessFlag(aiProcess_OptimizeGraph);
	} else {
		asset_->unsetAiProcessFlag(aiProcess_OptimizeGraph);
	}
}
