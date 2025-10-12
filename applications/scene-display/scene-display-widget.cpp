#include <regen/camera/camera-controller.h>
#include <regen/animation/animation-manager.h>
#include <regen/utility/filesystem.h>
#include <regen/objects/text/texture-mapped-text.h>
#include <regen/textures/texture-binder.h>
#include <QLineEdit>

//#include <regen/scene/scene-xml.h>
#include <regen/scene/scene-loader.h>
#include <regen/scene/scene-processors.h>
#include <regen/scene/resource-processor.h>
#include <regen/scene/resource-manager.h>
#include <regen/scene/scene-input-xml.h>

#include "../../regen/behavior/person-controller.h"
#include "regen/behavior/behavior-tree.h"
#include "regen/objects/terrain/blanket-trail.h"
#include "regen/textures/height-map.h"

using namespace regen::scene;
using namespace std;

#include "scene-display-widget.h"
#include "view-node.h"
#include "fps-widget.h"
#include "animation-events.h"
#include "interaction-manager.h"
#include "interactions/video-toggle.h"
#include "interactions/node-activation.h"
#include "../../regen/simulation/impulse-controller.h"
#include "regen/behavior/character-controller.h"
#include "../../regen/behavior/animal-controller.h"
#include "regen/av/video-recorder.h"
#include "regen/states/blit-state.h"

#define CONFIG_FILE_NAME ".regen-scene-display.cfg"

/////////////////
////// Scene Loader Animation
/////////////////

class SceneLoaderAnimation : public Animation {
public:
	SceneLoaderAnimation(SceneDisplayWidget *widget, const string &sceneFile)
		: Animation(true, false),
		  widget_(widget), sceneFile_(sceneFile) {
	}

	void glAnimate(RenderState *rs, GLdouble dt) override {
		widget_->loadSceneGraphicsThread(sceneFile_);
	}

	SceneDisplayWidget *widget_;
	const string sceneFile_;
};

/////////////////
////// Game Time Animation
/////////////////

class GameTimeAnimation : public Animation {
public:
	explicit GameTimeAnimation(SceneDisplayWidget *widget)
		: Animation(false, true), widget_(widget) {
	}

	void animate(GLdouble dt) override { widget_->updateGameTimeWidget(); }

protected:
	SceneDisplayWidget *widget_;
};

/////////////////
////// Mouse event handler
/////////////////

class SceneDisplayMouseHandler : public EventHandler, public Animation {
public:
	explicit SceneDisplayMouseHandler(Scene *app)
		: EventHandler(), Animation(true, false), app_(app) {
	}

	~SceneDisplayMouseHandler() override = default;

	void call(EventObject *evObject, EventData *data) override {
		if (data->eventID == Scene::BUTTON_EVENT) {
			auto *ev = (Scene::ButtonEvent *) data;
			boost::posix_time::ptime evtTime(boost::posix_time::microsec_clock::local_time());

			if (ev->button == Scene::MOUSE_BUTTON_LEFT && app_->hasHoveredObject()) {
				auto timeDiff = evtTime - buttonClickTime_[ev->button];
				if (timeDiff.total_milliseconds() < 200) {
					auto interaction = app_->getInteraction(app_->hoveredObject()->name());
					if (interaction.get() != nullptr) {
						boost::lock_guard<boost::mutex> lock(animationLock_);
						interactionQueue_.emplace(app_->hoveredObject(), interaction);
					}
				}
			}

			if (ev->pressed) {
				buttonClickTime_[ev->button] = evtTime;
			} else {
				buttonClickTime_.erase(ev->button);
			}
		}
	}

	void glAnimate(RenderState *rs, GLdouble dt) override {
		while (!interactionQueue_.empty()) {
			boost::lock_guard<boost::mutex> lock(animationLock_);
			auto interaction = interactionQueue_.front();
			if (!interaction.second->interactWith(interaction.first)) {
				REGEN_WARN("interaction failed with " << app_->hoveredObject()->name());
			}
			interactionQueue_.pop();
		}
	}

protected:
	Scene *app_;
	std::map<int, boost::posix_time::ptime> buttonClickTime_;
	std::queue<std::pair<ref_ptr<StateNode>, ref_ptr<SceneInteraction> > > interactionQueue_;
	boost::mutex animationLock_;
};

/////////////////
/////////////////

SceneDisplayWidget::SceneDisplayWidget(QtApplication *app)
	: QMainWindow(),
	  anchorIndex_(0),
	  inputDialog_(nullptr),
	  inputWidget_(nullptr),
	  app_(app),
	  lastUpdateTime_(boost::posix_time::microsec_clock::local_time()),
	  settings_("regen-scene-display") {
	setMouseTracking(true);
	anchorEaseInOutIntensity_ = 1.0;
	anchorPauseTime_ = 2.0;
	anchorTimeScale_ = 1.0;

	InteractionManager::registerInteraction(
		"video-toggle", ref_ptr<VideoToggleInteration>::alloc());

	// load window width/height from Qt settings
	int width = settings_.value("width", 1280).toInt();
	int height = settings_.value("height", 960).toInt();
	REGEN_INFO("Initial window size: " << width << "x" << height);

	ui_.setupUi(this);
	ui_.glWidgetLayout->addWidget(app_->glWidgetContainer(), 0, 0, 1, 1);
	ui_.worldTimeFactor->setValue(app_->worldTime().scale);
	resize(width, height);
	readConfig();
}

void SceneDisplayWidget::init() {
	if (activeFile_.empty()) {
		openFile();
	} else {
		loadScene(activeFile_);
	}
}

SceneDisplayWidget::~SceneDisplayWidget() {
	settings_.setValue("width", width());
	settings_.setValue("height", height());
	settings_.sync();
	if (inputDialog_ != nullptr) {
		delete inputDialog_;
		delete inputWidget_;
	}
}

void SceneDisplayWidget::resetFile() {
	activeFile_ = "";
	openFile();
}

void SceneDisplayWidget::readConfig() {
	// just read in the fluid file for now
	boost::filesystem::path p(userDirectory());
	p /= CONFIG_FILE_NAME;
	if (!boost::filesystem::exists(p)) return;
	ifstream cfgFile;
	cfgFile.open(p.c_str());
	cfgFile >> activeFile_;
	cfgFile.close();
}

void SceneDisplayWidget::writeConfig() {
	// just write out the fluid file for now
	boost::filesystem::path p(userDirectory());
	p /= CONFIG_FILE_NAME;
	ofstream cfgFile;
	cfgFile.open(p.c_str());
	cfgFile << activeFile_ << endl;
	cfgFile.close();
}

void SceneDisplayWidget::nextView() {
	if (viewNodes_.empty()) return;
	ViewNode &active0 = *activeView_;
	active0.node->set_isHidden(GL_TRUE);

	activeView_++;
	if (activeView_ == viewNodes_.end()) {
		activeView_ = viewNodes_.begin();
	}

	ViewNode &active1 = *activeView_;
	active1.node->set_isHidden(GL_FALSE);
	app_->toplevelWidget()->setWindowTitle(QString(active1.name.c_str()));

	if (videoRecorder_.get()) {
		auto blitState = active1.node->findStateWithType<BlitToScreen>();
		if (blitState) {
			app_->withGLContext([&] {
				videoRecorder_->setFrameBuffer(blitState->fbo(), blitState->attachment());
			});
		}
	}
}

void SceneDisplayWidget::previousView() {
	if (viewNodes_.empty()) return;
	ViewNode &active0 = *activeView_;
	active0.node->set_isHidden(GL_TRUE);

	if (activeView_ == viewNodes_.begin()) {
		activeView_ = viewNodes_.end();
	}
	activeView_--;

	ViewNode &active1 = *activeView_;
	active1.node->set_isHidden(GL_FALSE);
	app_->toplevelWidget()->setWindowTitle(QString(active1.name.c_str()));

	if (videoRecorder_.get()) {
		auto blitState = active1.node->findStateWithType<BlitToScreen>();
		if (blitState) {
			app_->withGLContext([&] {
				videoRecorder_->setFrameBuffer(blitState->fbo(), blitState->attachment());
			});
		}
	}
}

void SceneDisplayWidget::toggleOffCameraTransform() {
	if (cameraController_.get()) {
		cameraController_->stopAnimation();
	}
}

void SceneDisplayWidget::toggleOnCameraTransform() {
	if (cameraController_.get()) {
		// update the camera position
		cameraController_->setTransform(
			mainCamera_->position(0),
			mainCamera_->direction(0));
		cameraController_->startAnimation();
		cameraController_->animate(0.0);
	}
}

double SceneDisplayWidget::getAnchorTime(
	const Vec3f &fromPosition, const Vec3f &toPosition) {
	double linearDistance = (fromPosition - toPosition).length();
	// travel with ~6m per second
	return std::max(2.0, linearDistance / 6.0);
}

void SceneDisplayWidget::activateAnchor() {
	auto &anchor = anchors_[anchorIndex_];
	auto &camPos = mainCamera_->position(0);
	auto &camDir = mainCamera_->direction(0);
	auto cameraAnchor = ref_ptr<FixedCameraAnchor>::alloc(camPos, camDir);
	double dt = getAnchorTime(anchor->position(), camPos);

	anchorAnim_ = ref_ptr<KeyFrameController>::alloc(mainCamera_);
	anchorAnim_->setRepeat(GL_FALSE);
	anchorAnim_->setEaseInOutIntensity(anchorEaseInOutIntensity_);
	anchorAnim_->setPauseBetweenFrames(anchorPauseTime_);
	anchorAnim_->push_back(cameraAnchor, 0.0);
	anchorAnim_->push_back(anchor, dt * anchorTimeScale_);
	anchorAnim_->connect(Animation::ANIMATION_STOPPED, ref_ptr<LambdaEventHandler>::alloc(
		                     [this](EventObject *emitter, EventData *data) {
			                     toggleOnCameraTransform();
			                     anchorAnim_ = ref_ptr<KeyFrameController>();
		                     }));
	anchorAnim_->startAnimation();
}

void SceneDisplayWidget::nextAnchor() {
	if (anchors_.empty()) return;
	if (!mainCamera_.get()) return;
	if (anchorAnim_.get()) {
		anchorAnim_->stopAnimation();
	}
	toggleOffCameraTransform();

	anchorIndex_++;
	if (anchorIndex_ >= anchors_.size()) {
		anchorIndex_ = 0;
	}
	activateAnchor();
}

void SceneDisplayWidget::playAnchor() {
	if (anchors_.empty()) return;
	if (!mainCamera_.get()) return;
	if (anchorAnim_.get()) {
		anchorAnim_->stopAnimation();
		anchorAnim_ = ref_ptr<KeyFrameController>();
		toggleOnCameraTransform();
		return;
	}
	toggleOffCameraTransform();

	anchorAnim_ = ref_ptr<KeyFrameController>::alloc(mainCamera_);
	anchorAnim_->setRepeat(GL_TRUE);
	//anchorAnim_->setSkipFirstFrameOnLoop(GL_TRUE);
	anchorAnim_->setEaseInOutIntensity(anchorEaseInOutIntensity_);
	anchorAnim_->setPauseBetweenFrames(anchorPauseTime_);
	auto camPos = mainCamera_->position(0);
	auto camDir = mainCamera_->direction(0);
	anchorAnim_->push_back(camPos, camDir, 0.0);
	Vec3f lastPos = camPos;
	for (auto &anchor: anchors_) {
		double dt = getAnchorTime(anchor->position(), lastPos);
		anchorAnim_->push_back(anchor, dt * anchorTimeScale_);
		lastPos = anchor->position();
	}
	anchorAnim_->animate(0.0);
	anchorAnim_->startAnimation();
}

void SceneDisplayWidget::makeVideo(bool isClicked) {
	if (isClicked) {
		auto &view = *activeView_;
		auto blitState = view.node->findStateWithType<BlitToScreen>();
		if (!blitState) {
			REGEN_WARN("No FBOState found in view node");
			return;
		}
		auto blitFBO = blitState->fbo();
		auto &blitTextures = blitFBO->colorTextures();
		auto blitIndex = blitState->attachment() - GL_COLOR_ATTACHMENT0;
		if (blitTextures.size() < blitIndex) {
			REGEN_WARN("FBO has " << blitTextures.size() << " textures, but attachment is " << blitIndex);
			return;
		}
		app_->withGLContext([&] {
			//videoRecorder_ = ref_ptr<VideoRecorder>::alloc(blitTextures[blitIndex]);
			videoRecorder_ = ref_ptr<VideoRecorder>::alloc(blitFBO, blitState->attachment());
			videoRecorder_->initialize();
			videoRecorder_->startAnimation();
		});
	} else {
		videoRecorder_->connect(Animation::ANIMATION_STOPPED, ref_ptr<LambdaEventHandler>::alloc(
			                        [this](EventObject *emitter, EventData *data) {
				                        auto sourceFile = videoRecorder_->filename();
				                        videoRecorder_->finalize();
				                        videoRecorder_ = {};
				                        QMetaObject::invokeMethod(this, [this, sourceFile]() {
					                        // show a dialog asking the user where to save the video
					                        QString fileName = QFileDialog::getSaveFileName(this,
						                        tr("Save Video"),
						                        "regen.mp4",
						                        tr("Video Files (*.mp4 *.avi)"));
					                        if (!fileName.isEmpty()) {
						                        // Save the video to the selected file
						                        boost::filesystem::copy_file(sourceFile,
						                                                     fileName.toStdString(),
						                                                     boost::filesystem::copy_options::overwrite_existing);
						                        REGEN_INFO("Video saved to " << fileName.toStdString());
					                        }
				                        }, Qt::QueuedConnection);
			                        }));
		videoRecorder_->stopAnimation();
	}
}

void SceneDisplayWidget::toggleInputsDialog() {
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

void SceneDisplayWidget::toggleCameraPopup() {
	if (!mainCamera_.get()) {
		REGEN_WARN("No main camera available.");
		return;
	}

	static QDialog *cameraPopup = nullptr;
	static QLineEdit *positionLineEdit = nullptr;
	static QLineEdit *directionLineEdit = nullptr;

	if (cameraPopup && cameraPopup->isVisible()) {
		cameraPopup->hide();
		return;
	}

	if (!cameraPopup) {
		cameraPopup = new QDialog(this);
		cameraPopup->setWindowTitle("Camera Properties");
		cameraPopup->setModal(false);
		cameraPopup->setAttribute(Qt::WA_DeleteOnClose, false);
		cameraPopup->setMinimumSize(500, 200); // Set the minimum size of the dialog

		auto *layout = new QVBoxLayout(cameraPopup);
		layout->setAlignment(Qt::AlignTop); // Align items to the top

		positionLineEdit = new QLineEdit(cameraPopup);
		positionLineEdit->setTextMargins(4, 4, 4, 4);
		positionLineEdit->setStyleSheet("font-size: 14px;");
		positionLineEdit->setReadOnly(true);
		layout->addWidget(new QLabel("Position:", cameraPopup));
		layout->addWidget(positionLineEdit);

		directionLineEdit = new QLineEdit(cameraPopup);
		directionLineEdit->setTextMargins(4, 4, 4, 4);
		directionLineEdit->setStyleSheet("font-size: 14px;");
		directionLineEdit->setReadOnly(true);
		layout->addWidget(new QLabel("Direction:", cameraPopup));
		layout->addWidget(directionLineEdit);

		cameraPopup->setLayout(layout);
	}

	Vec3f position = mainCamera_->position(0);
	Vec3f direction = mainCamera_->direction(0);

	positionLineEdit->setText(QString("%1, %2, %3").arg(position.x).arg(position.y).arg(position.z));
	directionLineEdit->setText(QString("%1, %2, %3").arg(direction.x).arg(direction.y).arg(direction.z));

	cameraPopup->show();
}

void SceneDisplayWidget::toggleWireframe() {
	bool toggleState = ui_.wireframeToggle->isChecked();
	if (toggleState) {
		if (wireframeState_.get() == nullptr) {
			wireframeState_ = ref_ptr<FillModeState>::alloc(GL_LINE);
			app_->renderTree()->state()->joinStates(wireframeState_);
		}
	} else if (wireframeState_.get() != nullptr) {
		app_->renderTree()->state()->disjoinStates(wireframeState_);
		wireframeState_ = ref_ptr<State>();
	}
}

void SceneDisplayWidget::toggleVSync() {
	if (ui_.vsyncButton->isChecked()) {
		app_->setVSyncEnabled(true);
	} else {
		app_->setVSyncEnabled(false);
	}
}

void SceneDisplayWidget::toggleInfo(bool isOn) {
	if (isOn) {
		auto guiNode = app_->renderTree()->findNodeWithName("GUI-Pass");
		if (guiNode) {
			guiNode->set_isHidden(GL_FALSE);
		}
	} else {
		auto guiNode = app_->renderTree()->findNodeWithName("GUI-Pass");
		if (guiNode) {
			guiNode->set_isHidden(GL_TRUE);
		}
	}
}

void SceneDisplayWidget::updateGameTimeWidget() {
	auto &t_ptime = app_->worldTime().p_time;
	auto t_seconds = t_ptime.time_of_day().total_seconds();
	auto t_hours = t_seconds / 3600;
	auto t_minutes = (t_seconds % 3600) / 60;
	auto t_seconds_ = t_seconds % 60;
	QTime q_time(t_hours, t_minutes, t_seconds_);

	boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time();
	if ((now - lastUpdateTime_).total_seconds() >= 1) {
		ui_.worldTime->setTime(q_time);
		lastUpdateTime_ = now;
	}
}

void SceneDisplayWidget::onWorldTimeChanged() {
	// get time from the time widget
	auto q_time = ui_.worldTime->time();
	// convert to time_t
	time_t t = q_time.hour() * 3600 + q_time.minute() * 60 + q_time.second();
	app_->setWorldTime(t);
}

void SceneDisplayWidget::onWorldTimeFactorChanged(double value) {
	app_->setWorldTimeScale(value);
}

void SceneDisplayWidget::openFile() {
	QFileDialog dialog(this);
	dialog.setFileMode(QFileDialog::AnyFile);
	dialog.setNameFilters({"XML Files (*.xml)", "All files (*.*)"});
	dialog.setViewMode(QFileDialog::Detail);
	dialog.selectFile(QString(activeFile_.c_str()));

	if (!dialog.exec()) {
		if (activeFile_.empty()) {
			REGEN_WARN("no texture updater file selected, exiting.");
			exit(0);
		}
		return;
	}

	QStringList fileNames = dialog.selectedFiles();
	activeFile_ = fileNames.first().toStdString();
	writeConfig();

	loadScene(activeFile_);
}

void SceneDisplayWidget::updateSize() {
}

void SceneDisplayWidget::loadScene(const string &sceneFile) {
	if (inputDialog_ != nullptr && inputDialog_->isVisible()) {
		inputDialog_->hide();
	}
	loadAnim_ = ref_ptr<SceneLoaderAnimation>::alloc(this, sceneFile);
	loadAnim_->startAnimation();
}

/////////////////////////////
//////// Application Configuration Node
/////////////////////////////

void SceneDisplayWidget::handleCameraConfiguration(
	scene::SceneLoader &sceneParser,
	const ref_ptr<SceneInputNode> &cameraNode) {
	ref_ptr<Camera> cam = sceneParser.getResources()->getCamera(&sceneParser, cameraNode->getName());
	if (cam.get() == nullptr) {
		REGEN_WARN("Unable to find camera for '" << cameraNode->getDescription() << "'.");
		return;
	}
	ref_ptr<ModelTransformation> transform;
	if (cameraNode->hasAttribute("transform")) {
		transform = sceneParser.getResources()->getTransform(
			&sceneParser, cameraNode->getValue("transform"));
		if (transform.get() == nullptr) {
			REGEN_WARN("Unable to find transform for '" << cameraNode->getDescription() << "'.");
			return;
		}
	}
	ref_ptr<Mesh> mesh;
	auto meshes = sceneParser.getResources()->getMesh(&sceneParser, cameraNode->getValue("mesh"));
	if (meshes.get() != nullptr && !meshes->empty()) {
		auto meshIndex = cameraNode->getValue<GLuint>("mesh-index", 0u);
		if (meshIndex >= meshes->size()) {
			REGEN_WARN("Invalid mesh index for '" << cameraNode->getDescription() << "'.");
			meshIndex = 0;
		}
		mesh = (*meshes.get())[meshIndex];
	}
	mainCamera_ = cam;
	cameraController_ = ref_ptr<CameraController>();

	auto cameraMode = cameraNode->getValue<string>("mode", "first-person");
	auto cameraType = cameraNode->getValue<string>("type", "default");

	if (cameraType == "default") {
		cameraController_ = ref_ptr<CameraController>::alloc(cam);
	}
	// IMPULSE CONTROLLER
	else if (cameraType == "impulse") {
		if (mesh.get() == nullptr) {
			REGEN_WARN("Impulse controller requires a mesh.");
			return;
		}
		if (transform.get() == nullptr) {
			REGEN_WARN("Impulse controller requires a transform.");
			return;
		}
		if (mesh->physicalObjects().empty()) {
			REGEN_WARN("Impulse controller requires a physical object.");
			return;
		}
		auto &physicalObject = mesh->physicalObjects().front();
		auto impulseController = ref_ptr<ImpulseController>::alloc(cam, physicalObject);
		cameraController_ = impulseController;
	}
	// CHARACTER CONTROLLER
	else if (cameraType == "physical-character") {
		if (mesh.get() == nullptr) {
			REGEN_WARN("Physical character controller requires a mesh.");
			return;
		}
		if (transform.get() == nullptr) {
			REGEN_WARN("Physical character controller requires a transform.");
			return;
		}
		if (sceneParser.getPhysics().get() == nullptr) {
			REGEN_WARN("No BulletPhysics instance set for CharacterController.");
		}
		auto characterController = ref_ptr<CharacterController>::alloc(cam, sceneParser.getPhysics());
		characterController->setCollisionHeight(
			cameraNode->getValue<GLfloat>("collision-height", 0.8));
		characterController->setCollisionRadius(
			cameraNode->getValue<GLfloat>("collision-radius", 0.8));
		characterController->setStepHeight(
			cameraNode->getValue<GLfloat>("step-height", 0.35));
		characterController->setMaxSlope(
			cameraNode->getValue<GLfloat>("max-slope", 0.8));
		characterController->setGravityForce(
			cameraNode->getValue<GLfloat>("gravity-force", 30.0));
		characterController->setJumpVelocity(
			cameraNode->getValue<GLfloat>("jump-velocity", 16.0f));
		cameraController_ = characterController;
	}
	// KEY-FRAME CONTROLLER
	else if (cameraType == "key-frames") {
		ref_ptr<KeyFrameController> keyFramesCamera = ref_ptr<KeyFrameController>::alloc(cam);
		if (cameraNode->hasAttribute("ease-in-out-intensity")) {
			keyFramesCamera->setEaseInOutIntensity(
				cameraNode->getValue<GLdouble>("ease-in-out-intensity", 1.0));
		}
		for (const auto &x: cameraNode->getChildren("key-frame")) {
			keyFramesCamera->push_back(
				x->getValue<Vec3f>("pos", Vec3f(0.0f, 0.0f, 1.0f)),
				x->getValue<Vec3f>("dir", Vec3f(0.0f, 0.0f, -1.0f)),
				x->getValue<GLdouble>("dt", 0.0)
			);
		}
		keyFramesCamera->startAnimation();
		animations_.emplace_back(keyFramesCamera);
	} else {
		REGEN_WARN("Invalid camera type '" << cameraType << "'.");
		return;
	}

	if (cameraController_.get()) {
		cameraController_->setMeshEyeOffset(
			cameraNode->getValue<Vec3f>("eye-offset", Vec3f(0.0)));
		cameraController_->set_moveAmount(
			cameraNode->getValue<GLfloat>("speed", 0.01f));
		cameraController_->setMeshDistance(
			cameraNode->getValue<GLfloat>("mesh-distance", 10.0f));
		cameraController_->setHorizontalOrientation(
			cameraNode->getValue<GLfloat>("horizontal-orientation", 0.0));
		cameraController_->setVerticalOrientation(
			cameraNode->getValue<GLfloat>("vertical-orientation", 0.0));
		cameraController_->setMeshHorizontalOrientation(
			cameraNode->getValue<GLfloat>("mesh-horizontal-orientation", 0.0));
		if (cameraMode == "third-person") {
			cameraController_->setCameraMode(CameraController::THIRD_PERSON);
		} else {
			cameraController_->setCameraMode(CameraController::FIRST_PERSON);
		}
		// attach the camera to a transform
		if (transform.get()) {
			cameraController_->setAttachedTo(transform, mesh);
		}

		std::vector<CameraCommandMapping> keyMappings;
		for (const auto &x: cameraNode->getChildren()) {
			if (x->getCategory() == string("key-mapping")) {
				CameraCommandMapping mapping;
				mapping.key = x->getValue("key");
				mapping.command = x->getValue<CameraCommand>("command", CameraCommand::NONE);
				if (mapping.command == CameraCommand::NONE) {
					REGEN_WARN("Invalid camera command for key mapping for key '" << mapping.key << "'.");
					continue;
				}
				keyMappings.push_back(mapping);
			}
		}

		ref_ptr<QtFirstPersonEventHandler> cameraEventHandler = ref_ptr<QtFirstPersonEventHandler>::alloc(
			cameraController_, keyMappings);
		cameraEventHandler->set_sensitivity(cameraNode->getValue<GLfloat>("sensitivity", 0.005f));
		app_->connect(Scene::KEY_EVENT, cameraEventHandler);
		app_->connect(Scene::BUTTON_EVENT, cameraEventHandler);
		app_->connect(Scene::MOUSE_MOTION_EVENT, cameraEventHandler);
		eventHandler_.emplace_back(cameraEventHandler);

		// make sure camera transforms are updated in first few frames
		cameraController_->animate(0.0);
		cameraController_->glAnimate(RenderState::get(), 0.0);
		cameraController_->startAnimation();
	}

	// read anchor points
	anchorEaseInOutIntensity_ = cameraNode->getValue<GLfloat>("ease-in-out-intensity", 1.0);
	anchorPauseTime_ = cameraNode->getValue<GLfloat>("anchor-pause-time", 0.5);
	anchorTimeScale_ = cameraNode->getValue<GLfloat>("anchor-time-scale", 1.0);
	for (const auto &x: cameraNode->getChildren("anchor")) {
		if (x->hasAttribute("transform")) {
			auto transform = sceneParser.getResources()->getTransform(&sceneParser, x->getValue("transform"));
			if (transform.get() == nullptr) {
				REGEN_WARN("Unable to find transform for anchor.");
				continue;
			}
			auto anchor = ref_ptr<TransformCameraAnchor>::alloc(transform);
			auto anchorOffset = x->getValue<Vec3f>("offset", Vec3f(1.0f));
			anchor->setOffset(anchorOffset);
			anchor->setFollowing(x->getValue<bool>("follow", false));
			auto anchorMode = x->getValue("look-at");
			if (anchorMode == "back") {
				anchor->setMode(TransformCameraAnchor::LOOK_AT_BACK);
			} else {
				anchor->setMode(TransformCameraAnchor::LOOK_AT_FRONT);
			}
			anchors_.emplace_back(anchor);
		} else {
			auto pos = x->getValue<Vec3f>("pos", Vec3f(0.0));
			auto dir = x->getValue<Vec3f>("dir", Vec3f(0.0));
			pos += x->getValue<Vec3f>("offset", Vec3f(0.0));
			auto anchor = ref_ptr<FixedCameraAnchor>::alloc(pos, dir);
			anchors_.emplace_back(anchor);
		}
	}
}

float getHeight(const Vec2f &pos, const ref_ptr<HeightMap> &heightMap) {
	if (heightMap.get()) {
		return heightMap->sampleHeight(pos);
	} else {
		return 0.0f;
	}
}

static ref_ptr<WorldModel> loadWorldModel(
	scene::SceneLoader &sceneParser,
	const ref_ptr<SceneInputNode> &configNode) {
	auto worldNode = configNode->getFirstChild("world");
	if (!worldNode.get()) {
		return {};
	}
	ref_ptr<WorldModel> worldModel = ref_ptr<WorldModel>::alloc();

	ref_ptr<HeightMap> heightMap;
	if (worldNode->hasAttribute("height-map")) {
		auto heightMap2D = sceneParser.getResources()->getTexture2D(
			&sceneParser, worldNode->getValue("height-map"));
		if (!heightMap2D) {
			REGEN_WARN("Unable to find height map texture for world model in `" << worldNode->getDescription() << "`.");
		} else {
			heightMap = ref_ptr<HeightMap>::dynamicCast(heightMap2D);
			if (!heightMap) {
				REGEN_WARN("Height map texture is not a height map in `" << worldNode->getDescription() << "`.");
			}
		}
	}

	for (const auto &x: worldNode->getChildren("place")) {
		// Add place of interest to the controller.
		// Here we distinguish between "HOME", "SPIRITUAL" and "GATHERING" places.
		auto placeType = x->getValue<PlaceType>("type", PlaceType::HOME);
		auto placePos = x->getValue<Vec2f>("point", Vec2f(0.0f));
		auto placeRadius = x->getValue<float>("radius", 1.0f);
		auto placeHeight = getHeight(placePos, heightMap);
		auto place = ref_ptr<Place>::alloc(
			x->getName(),
			placeType,
			Vec3f(placePos.x, placeHeight + 0.1f, placePos.y));
		place->setRadius(placeRadius);
		worldModel->places.push_back(place);
		ref_ptr<WorldObject> placeWO = place;
		sceneParser.putResource<WorldObject>(place->name(), placeWO);

		std::map<std::string, ref_ptr<WorldObject> > objMap;
		// load objects that are part of this place
		for (const auto &s: x->getChildren("object")) {
			std::stack<ref_ptr<WorldObject>> obj_queue;

			if (s->hasAttribute("mesh")) {
				uint32_t meshIdx = s->getValue<uint32_t>("mesh-index", 0u);
				auto meshes = sceneParser.getResources()->getMesh(&sceneParser, s->getValue("mesh"));
				if (!meshes || meshes->empty()) {
					REGEN_WARN("Cannot find mesh in `" << s->getDescription() << "`.");
				} else if (meshIdx >= meshes->size()) {
					REGEN_WARN("Invalid mesh index in `" << s->getDescription() << "`.");
				} else {
					auto mesh = (*meshes.get())[meshIdx];
					auto shape = mesh->indexedShape(0);
					uint32_t numInstances = shape->transform()->numInstances();

					for (uint32_t instance = 0; instance < numInstances; instance++) {
						obj_queue.push(ref_ptr<WorldObject>::alloc(s->getName(),
							mesh->indexedShape(instance), instance));
					}
				}
			}
			if (obj_queue.empty()) {
				auto relPoint = s->getValue<Vec2f>("point", Vec2f(0.0f));
				auto absObjPos = relPoint + placePos;
				auto objHeight = getHeight(absObjPos, heightMap);
				obj_queue.push(ref_ptr<WorldObject>::alloc(s->getName(),
					Vec3f(absObjPos.x, objHeight + 0.1f, absObjPos.y)));
			}

			while (!obj_queue.empty()) {
				ref_ptr<WorldObject> obj = obj_queue.top();
				obj_queue.pop();

				obj->setRadius(s->getValue<float>("radius", 1.0f));
				objMap[s->getName()] = obj;
				// load affordances
				for (const auto &a: s->getChildren("affordance")) {
					ref_ptr<Affordance> affordance = ref_ptr<Affordance>::alloc(obj);

					affordance->type = a->getValue<ActionType>("type", ActionType::IDLE);
					affordance->minDistance = a->getValue<float>("min-distance", 0.0f);
					affordance->slotCount = a->getValue<int>("num-slots", 1);
					affordance->spacing = a->getValue<float>("spacing", 2.0f);
					affordance->layout = a->getValue<SlotLayout>("slot-layout", SlotLayout::CIRCULAR);
					affordance->radius = a->getValue<float>("radius", 1.0f);
					affordance->baseOffset = a->getValue<Vec3f>("base-offset", Vec3f(0.0f, 0.0f, 0.0f));

					affordance->initialize();
					obj->addAffordance(affordance);
				}
				place->addWorldObject(obj);
				worldModel->addWorldObject(obj);
				sceneParser.putResource<WorldObject>(obj->name(), obj);
			}
		}

		for (const auto &s: x->getChildren("path")) {
			std::vector<ref_ptr<WayPoint> > pathPoints;
			for (const auto &t: s->getChildren("point")) {
				auto relPoint = t->getValue<Vec2f>("value", Vec2f(0.0f));
				auto absPoint = relPoint + placePos;
				auto pointHeight = getHeight(absPoint, heightMap);
				// place way points slightly above the ground to avoid z-fighting
				auto point = ref_ptr<WayPoint>::alloc(
					t->getName(),
					Vec3f(absPoint.x, pointHeight + 0.1f, absPoint.y));
				point->setRadius(t->getValue<float>("radius", 1.0f));
				pathPoints.push_back(point);
				worldModel->addWorldObject(point);
			}
			auto pathType = s->getValue<PathwayType>("type", PathwayType::STROLL);
			place->addPathWay(pathType, pathPoints);
		}
		worldModel->addWorldObject(place);
	}

	for (const auto &x: worldNode->getChildren("object")) {
		auto objType = x->getValue("type");
		if (objType == "waypoint") {
			auto wpPos = x->getValue<Vec2f>("point", Vec2f(0.0f));
			auto wpHeight = getHeight(wpPos, heightMap);
			// place way points slightly above the ground to avoid z-fighting
			ref_ptr<WayPoint> wp = ref_ptr<WayPoint>::alloc(
				x->getName(),
				Vec3f(wpPos.x, wpHeight + 0.1f, wpPos.y));
			wp->setRadius(x->getValue<float>("radius", 1.0f));
			worldModel->wayPoints.push_back(wp);
			worldModel->wayPointMap[x->getName()] = wp;
			worldModel->addWorldObject(wp);
			sceneParser.putResource<WorldObject>(wp->name(), wp);
		} else {
			REGEN_WARN("Unhandled object type '" << objType << "' in NPC controller.");
		}
	}

	for (const auto &x: worldNode->getChildren("path")) {
		auto from = worldModel->wayPointMap.find(x->getValue("from"));
		auto to = worldModel->wayPointMap.find(x->getValue("to"));
		if (from != worldModel->wayPointMap.end() && to != worldModel->wayPointMap.end()) {
			worldModel->wayPointConnections.push_back(std::make_pair(from->second, to->second));
		} else {
			REGEN_WARN("Unable to find way point for path in NPC controller.");
		}
	}

	return worldModel;
}

static float randomizeTrait(float baseValue, float variation) {
	if (variation <= 0.0f) {
		return baseValue;
	}
	float r = math::random<float>(); // random number in [0,1]
	float delta = (r - 0.5f) * 2.0f * variation; // random number in [-variation, +variation]
	return std::clamp(baseValue + delta, 0.0f, 1.0f);
}

static void handleAssetController(
	const ref_ptr<WorldModel> &worldModel,
	scene::SceneLoader &sceneParser,
	const ref_ptr<SceneInputNode> &animationNode,
	std::list<ref_ptr<Animation> > &animations,
	const ref_ptr<BoneAnimationItem> &nodeAnimItem) {
	auto controllerType = animationNode->getValue("type");
	auto tf = sceneParser.getResources()->getTransform(
		&sceneParser, animationNode->getValue("tf"));
	std::vector<ref_ptr<NonPlayerCharacterController> > controller;
	ref_ptr<SpatialIndex> spatialIndex;
	if (animationNode->hasAttribute("spatial-index")) {
		spatialIndex = sceneParser.getResources()->getIndex(
			&sceneParser, animationNode->getValue("spatial-index"));
		if (!spatialIndex.get()) {
			REGEN_WARN("Unable to find spatial index for controller '" << animationNode->getDescription() << "'.");
		}
	}
	std::string indexedShapeName = animationNode->getValue("indexed-shape");

	if (!tf.get()) {
		REGEN_WARN("Unable to find transform for controller '" << animationNode->getDescription() << "'.");
		return;
	}
	if (spatialIndex.get() && indexedShapeName.empty()) {
		REGEN_WARN("Spatial index specified but no indexed shape name given for controller '"
			<< animationNode->getDescription() << "'.");
		return;
	}

	auto meshVec = sceneParser.getResources()->getMesh(
		&sceneParser, animationNode->getValue("mesh"));
	ref_ptr<Mesh> mesh;
	if (meshVec.get() != nullptr && !meshVec->empty()) {
		uint32_t meshIdx = animationNode->getValue<uint32_t>("mesh-index", 0u);
		if (meshIdx >= meshVec->size()) {
			REGEN_WARN("Invalid mesh index for '" << animationNode->getDescription() << "'.");
			meshIdx = 0;
		}
		mesh = (*meshVec.get())[meshIdx];
		if (!mesh.get()) {
			REGEN_WARN("Unable to find mesh for controller '" << animationNode->getDescription() << "'.");
			return;
		}
	} else {
		REGEN_WARN("Unable to find mesh for controller '" << animationNode->getDescription() << "'.");
		return;
	}

	if (controllerType == "animal") {
		// TODO: support multiple instances
		Indexed<ref_ptr<ModelTransformation>> indexedTF(tf, 0);
		auto animalController = ref_ptr<AnimalController>::alloc(
			mesh, indexedTF, nodeAnimItem, worldModel);
		controller.push_back(animalController);
		animalController->setWorldTime(&sceneParser.application()->worldTime());
		animalController->setMesh(mesh);
		animalController->setWalkSpeed(animationNode->getValue<float>("walk-speed", 0.05f));
		animalController->setRunSpeed(animationNode->getValue<float>("run-speed", 0.1f));
		animalController->setFloorHeight(animationNode->getValue<float>("floor-height", 0.0f));
		animalController->setLaziness(animationNode->getValue<float>("laziness", 0.5f));
		animalController->setMaxHeight(animationNode->getValue<float>("max-height", std::numeric_limits<float>::max()));
		animalController->setMinHeight(
			animationNode->getValue<float>("min-height", std::numeric_limits<float>::lowest()));
		animalController->setTerritoryBounds(
			animationNode->getValue<Vec2f>("territory-center", Vec2f(0.0)),
			animationNode->getValue<Vec2f>("territory-size", Vec2f(10.0)));

		if (animationNode->hasAttribute("height-map")) {
			auto heightMap = sceneParser.getResources()->getTexture2D(
				&sceneParser, animationNode->getValue("height-map"));
			if (heightMap.get()) {
				auto heightMapCenter = animationNode->getValue<Vec2f>("height-map-center", Vec2f(0.0));
				auto heightMapSize = animationNode->getValue<Vec2f>("height-map-size", Vec2f(10.0));
				auto heightMapFactor = animationNode->getValue<float>("height-map-factor", 8.0f);
				animalController->setHeightMap(heightMap, heightMapCenter, heightMapSize, heightMapFactor);
			} else {
				REGEN_WARN("Unable to find height map for animal controller.");
			}
		}

		for (const auto &x: animationNode->getChildren()) {
			if (x->getCategory() == string("special")) {
				animalController->addSpecial(x->getValue("name"));
			}
		}

		animalController->startAnimation();
	} else if (controllerType == "person") {
		controller.reserve(tf->numInstances());

		// load mesh for footsteps, if any
		auto footstepVec = sceneParser.getResources()->getMesh(
			&sceneParser, animationNode->getValue("footstep-mesh"));
		ref_ptr<BlanketTrail> footstepTrail;
		if (footstepVec.get() && !footstepVec->empty()) {
			auto footstepMesh = (*footstepVec.get())[0];
			footstepTrail = ref_ptr<BlanketTrail>::dynamicCast(footstepMesh);
			if (!footstepTrail) {
				REGEN_WARN("Footstep mesh is not a BlanketTrail in '" << animationNode->getDescription() << "'.");
			}
		} else if (animationNode->hasAttribute("footstep-mesh")) {
			REGEN_WARN("Unable to find footstep mesh for NPC controller in '" << animationNode->getDescription() << "'.");
		}

		for (int32_t tfIdx = 0; tfIdx < tf->numInstances(); tfIdx++) {
			Indexed<ref_ptr<ModelTransformation>> indexedTF(tf, tfIdx);
			auto personController = ref_ptr<PersonController>::alloc(
				mesh, indexedTF, nodeAnimItem, worldModel);
			personController->setMesh(mesh);
			controller.push_back(personController);

			if (spatialIndex.get()) {
				auto indexedShape = spatialIndex->getShape(indexedShapeName, tfIdx);
				if (!indexedShape) {
					REGEN_WARN("Unable to find indexed shape '" << indexedShapeName
						<< "' in spatial index for controller '" << animationNode->getDescription() << "'.");
				} else {
					auto perceptionSystem = std::make_unique<PerceptionSystem>(spatialIndex, indexedShape);
					perceptionSystem->setCollisionBit(animationNode->getValue<uint32_t>("collision-bit", 0));
					personController->setPerceptionSystem(std::move(perceptionSystem));
				}
			}
			if (footstepTrail.get()) {
				float leftFootTime = animationNode->getValue<float>("left-foot-time", 0.2f);
				float rightFootTime = animationNode->getValue<float>("right-foot-time", 0.8f);
				personController->setFootstepTrail(footstepTrail, leftFootTime, rightFootTime);
			}
			if (animationNode->hasAttribute("decision-interval")) {
				personController->setDecisionInterval(
					animationNode->getValue<float>("decision-interval", 0.25f));
			}
			if (animationNode->hasAttribute("perception-interval")) {
				personController->setPerceptionInterval(
					animationNode->getValue<float>("perception-interval", 0.1f));
			}
			personController->setWalkSpeed(animationNode->getValue<float>("walk-speed", 0.05f));
			personController->setRunSpeed(animationNode->getValue<float>("run-speed", 0.1f));
			personController->setMaxTurnDegPerSecond(
				animationNode->getValue<float>("max-turn-angle", 90.0f));
			personController->setPersonalSpace(
				animationNode->getValue<float>("personal-space", 4.5f));
			personController->setAvoidanceWeight(
				animationNode->getValue<float>("avoidance-weight", 0.5f));
			personController->setPushThroughDistance(
				animationNode->getValue<float>("push-through-distance", 0.5f));
			personController->setLookAheadThreshold(
				animationNode->getValue<float>("look-ahead-threshold", 6.0f));
			personController->setAvoidanceDecay(
				animationNode->getValue<float>("avoidance-decay", 0.5f));
			personController->setWallTangentWeight(
				animationNode->getValue<float>("wall-tangent-weight", 0.5f));
			personController->setVelOrientationWeight(
				animationNode->getValue<float>("velocity-orientation-weight", 0.5f));
			personController->setFloorHeight(animationNode->getValue<float>("floor-height", 0.0f));
			if (animationNode->hasAttribute("base-orientation")) {
				personController->setBaseOrientation(
					animationNode->getValue<GLfloat>("base-orientation", 0.0f));
			}

			auto &kb = personController->knowledgeBase();
			kb.setWorldTime(&sceneParser.application()->worldTime());
			// Set base time for actions/staying at a place
			kb.setBaseTimeActivity(
				animationNode->getValue<float>("base-time-activity", 120.0f));
			kb.setBaseTimePlace(
				animationNode->getValue<float>("base-time-place", 1200.0f));
			// Set randomized character traits.
			kb.setTraitStrength(Trait::LAZINESS, randomizeTrait(animationNode->getValue<float>(
				"laziness", 0.5f), 0.25f));
			kb.setTraitStrength(Trait::SPIRITUALITY, randomizeTrait(animationNode->getValue<float>(
				"spirituality", 0.5f), 0.25f));
			kb.setTraitStrength(Trait::ALERTNESS, randomizeTrait(animationNode->getValue<float>(
				"alertness", 0.5f), 0.25f));
			kb.setTraitStrength(Trait::BRAVERY, randomizeTrait(animationNode->getValue<float>(
				"bravery", 0.5f), 0.25f));
			kb.setTraitStrength(Trait::SOCIALABILITY, randomizeTrait(animationNode->getValue<float>(
				"sociability", 0.5f), 0.25f));

			//npcController->setHomeBounds(
			//	animationNode->getValue<Vec2f>("territory-center", Vec2f(0.0)),
			//	animationNode->getValue<Vec2f>("territory-size", Vec2f(10.0)));

			// Set the weapon mesh is any.
			auto weaponMeshVec = sceneParser.getResources()->getMesh(
				&sceneParser, animationNode->getValue("weapon-mesh"));
			if (weaponMeshVec.get() != nullptr && !weaponMeshVec->empty()) {
				uint32_t weaponMeshIdx = animationNode->getValue<uint32_t>("weapon-mesh-index", 0u);
				if (weaponMeshIdx >= weaponMeshVec->size()) {
					REGEN_WARN("Invalid weapon mesh index for '" << animationNode->getDescription() << "'.");
					weaponMeshIdx = 0;
				}
				auto weaponMesh = (*weaponMeshVec.get())[weaponMeshIdx];
				if (weaponMesh.get()) {
					personController->setWeaponMesh(weaponMesh);
				} else {
					REGEN_WARN("Unable to find weapon mesh in '" << animationNode->getDescription() << "'.");
				}
			} else if (animationNode->hasAttribute("weapon-mesh")) {
				REGEN_WARN("Unable to find weapon mesh in '" << animationNode->getDescription() << "'.");
			}

			if (animationNode->hasAttribute("height-map")) {
				auto heightMap2d = sceneParser.getResources()->getTexture2D(
					&sceneParser, animationNode->getValue("height-map"));
				if (!heightMap2d) {
					REGEN_WARN("Unable to find height map in '" << animationNode->getDescription() << "'.");
				} else {
					auto heightMap = ref_ptr<HeightMap>::dynamicCast(heightMap2d);
					if (!heightMap) {
						REGEN_WARN("Height map texture is not a height map in '" << animationNode->getDescription() << "'.");
					} else {
						personController->setHeightMap(heightMap);
					}
				}
			}

			// Load a behavior tree.
			// NOTE: XML scene may define different behavior trees for different instances.
			LoadingContext ctx(&sceneParser, {});
			for (const auto &btNodeXML: animationNode->getChildren("behavior-tree")) {
				list<IndexRange> indices = btNodeXML->getIndexSequence(tf->numInstances());
				for (auto &range : indices) {
					if (range.isWithinRange(tfIdx) && !btNodeXML->getChildren().empty()) {
						auto btRootXML = btNodeXML->getChildren().front();
						auto btRoot = BehaviorTree::load(ctx, *btRootXML.get());
						personController->setBehaviorTree(std::move(btRoot));
						break;
					}
				}
			}

			personController->initializeController();
		}
	} else {
		REGEN_WARN("Unhandled controller type in '" << animationNode->getDescription() << "'.");
	}

	for (const auto &c: controller) {
		animations.emplace_back(c);
	}
}

static void handleAssetAnimationConfiguration(
	QtApplication *app_,
	const ref_ptr<WorldModel> &worldModel,
	scene::SceneLoader &sceneParser,
	list<ref_ptr<EventHandler> > &eventHandler,
	const ref_ptr<SceneInputNode> &animationNode,
	std::list<ref_ptr<Animation> > &animations) {
	ref_ptr<AssetImporter> animAsset =
			sceneParser.getResources()->getAsset(&sceneParser, animationNode->getName());
	if (animAsset.get() == nullptr) {
		REGEN_WARN("Unable to find animation with name '" << animationNode->getName() << "'.");
		return;
	}
	auto animItem = sceneParser.getAnimationRanges(animationNode->getName());
	if (!animItem) {
		REGEN_WARN("Unable to find animation ranges for animation with name '" << animationNode->getName() << "'.");
		return;
	}
	animItem->animation = animAsset->getNodeAnimation();
	if (animItem->animation.get()) {
		animItem->animation->startAnimation();
	}

	if (animationNode->getValue("mode") == string("random")) {
		if (animItem->animation.get()) {
			ref_ptr<EventHandler> animStopped = ref_ptr<RandomAnimationRangeUpdater>::alloc(animItem);
			animItem->animation->connect(Animation::ANIMATION_STOPPED, animStopped);
			eventHandler.push_back(animStopped);

			EventData evData;
			evData.eventID = Animation::ANIMATION_STOPPED;
			animStopped->call(animItem->animation.get(), &evData);
		}
	} else if (animationNode->getValue("mode") == "fixed") {
		if (animItem->animation.get()) {
			auto &fixedRange = animItem->ranges[0];
			ref_ptr<EventHandler> animStopped = ref_ptr<FixedAnimationRangeUpdater>::alloc(animItem->animation, fixedRange);
			animItem->animation->connect(Animation::ANIMATION_STOPPED, animStopped);
			eventHandler.push_back(animStopped);

			EventData evData;
			evData.eventID = Animation::ANIMATION_STOPPED;
			animStopped->call(animItem->animation.get(), &evData);
		}
	} else if (animationNode->getCategory() == "controller") {
		handleAssetController(worldModel, sceneParser, animationNode, animations, animItem);
	} else {
		map<string, KeyAnimationMapping> keyMappings;
		string idleAnimation = animationNode->getValue("idle");

		for (const auto &x: animationNode->getChildren()) {
			KeyAnimationMapping mapping;
			if (x->getCategory() == string("key-mapping")) {
				mapping.key = x->getValue("key");
			} else if (x->getCategory() == string("mouse-mapping")) {
				mapping.key = REGEN_STRING("button" << x->getValue<int>("button", 1));
			} else if (x->getCategory() == string("controller")) {
				handleAssetController(worldModel, sceneParser, x, animations, animItem);
				continue;
			} else {
				REGEN_WARN("Unhandled animation node " << x->getDescription() << ".");
				continue;
			}
			mapping.press = x->getValue("press");
			mapping.toggle = x->getValue<GLboolean>("toggle", GL_FALSE);
			mapping.interrupt = x->getValue<GLboolean>("interrupt", GL_FALSE);
			mapping.releaseInterrupt = x->getValue<GLboolean>("release-interrupt", GL_FALSE);
			mapping.backwards = x->getValue<GLboolean>("backwards", GL_FALSE);
			mapping.idle = x->getValue("idle");
			keyMappings[mapping.key] = mapping;
		}

		if (!keyMappings.empty()) {
			if (animItem->animation.get()) {
				auto keyHandler = ref_ptr<KeyAnimationRangeUpdater>::alloc(
					animItem, keyMappings, idleAnimation);
				app_->connect(Scene::KEY_EVENT, keyHandler);
				app_->connect(Scene::BUTTON_EVENT, keyHandler);
				animItem->animation->connect(Animation::ANIMATION_STOPPED, keyHandler);
				eventHandler.emplace_back(keyHandler);
			}
		}
	}
}

static void handleMouseConfiguration(
	QtApplication *app_,
	scene::SceneLoader &sceneParser,
	const ref_ptr<SceneInputNode> &mouseNode) {
	for (auto &child: mouseNode->getChildren()) {
		if (child->getCategory() == string("click")) {
			auto nodeName = child->getValue("node");
			if (nodeName.empty()) {
				REGEN_WARN("No node name specified for mouse interaction.");
				continue;
			}

			auto interactionName = child->getValue("interaction");
			if (!interactionName.empty()) {
				auto interaction = InteractionManager::getInteraction(interactionName);
				if (!interaction.get()) {
					REGEN_WARN("Unable to find interaction with name '" << interactionName << "'.");
					continue;
				}
				app_->registerInteraction(nodeName, interaction);
			}

			auto interactionNodeName = child->getValue("interaction-node");
			if (!interactionNodeName.empty()) {
				// parse the interaction node
				auto interactionNode = ref_ptr<StateNode>::alloc();
				interactionNode->state()->joinStates(app_->renderTree()->state());
				sceneParser.processNode(interactionNode, interactionNodeName, "node");
				// create and register the interaction
				auto interaction = ref_ptr<NodeActivation>::alloc(app_, interactionNode);
				app_->registerInteraction(nodeName, interaction);
			}
		}
	}
}

/////////////////////////////
/////////////////////////////
/////////////////////////////

void SceneDisplayWidget::loadSceneGraphicsThread(const string &sceneFile) {
	REGEN_INFO("Loading XML scene at " << sceneFile << ".");

	AnimationManager::get().pause(GL_TRUE);
	AnimationManager::get().clear();
	AnimationManager::get().setRootState(app_->renderTree()->state());
	TextureBinder::reset();

	animations_.clear();
	viewNodes_.clear();
	anchors_.clear();
	if (physics_.get()) {
		physics_->clear();
		physics_ = {};
	}
	mainCamera_ = {};
	anchorAnim_ = {};
	timeWidgetAnimation_ = {};
	anchorIndex_ = 0;
	app_->clear();
	eventHandler_.clear();

	ref_ptr<RootNode> tree = app_->renderTree();

	ref_ptr<SceneInputXML> xmlInput = ref_ptr<SceneInputXML>::alloc(sceneFile);
	scene::SceneLoader sceneParser(app_, xmlInput);
	sceneParser.setNodeProcessor(ref_ptr<ViewNodeProcessor>::alloc(&viewNodes_));
	ref_ptr<SceneInputNode> root = sceneParser.getRoot();

	// Note: We need to do this before processing the root node, so that
	// any buffer objects created in the initialization node are available
	// when processing the root node.
	REGEN_INFO("Traversal of initialization node...");
	if (root->getFirstChild("node", "initialize").get() != nullptr) {
		ref_ptr<StateNode> initializeNode = ref_ptr<StateNode>::alloc();
		initializeNode->state()->joinStates(app_->renderTree()->state());
		sceneParser.processNode(initializeNode, "initialize", "node");
		// ensure the buffer objects of the initialization node are updated.
		StagingSystem::instance().updateData();
		initializeNode->traverse(RenderState::get());
		// Make sure that any FBO that is written to in the initialization node
		// has its content actually written.
		glFinish();
	}
	REGEN_INFO("Traversal of initialization node done.");

	// pre-load meshes and shapes
	REGEN_INFO("Loading shapes from scene...");
	sceneParser.loadShapes();
	REGEN_INFO("Shapes loaded.");

	// Note: configurations may refer to resources, so better to load them after the root node.
	ref_ptr<SceneInputNode> configurationNode = root->getFirstChild("node", "configuration");
	if (configurationNode.get() == nullptr) { configurationNode = root; }

	// load world model
	auto worldModel = loadWorldModel(sceneParser, configurationNode);
	app_->setWorldModel(worldModel);

	// Process the root node
	sceneParser.processNode(tree, "root", "node");
	physics_ = sceneParser.getPhysics();
	eventHandler_.insert(eventHandler_.end(),
	                     sceneParser.getEventHandler().begin(),
	                     sceneParser.getEventHandler().end());
	spatialIndices_ = sceneParser.getResources()->getIndices();

	// Process the configuration node
	for (const auto &x: configurationNode->getChildren()) {
		if (x->getCategory() == string("animation")) {
			if (x->getValue("type") == string("asset")) {
				handleAssetAnimationConfiguration(app_, worldModel, sceneParser, eventHandler_, x, animations_);
			}
		} else if (x->getCategory() == string("camera")) {
			handleCameraConfiguration(sceneParser, x);
		} else if (x->getCategory() == string("mouse")) {
			handleMouseConfiguration(app_, sceneParser, x);
		}
	}

	app_->initializeScene();

	/////////////////////////////
	//////// Configure World Time
	/////////////////////////////

	if (configurationNode->hasAttribute("date")) {
		auto dateString = configurationNode->getValue("date");
		struct tm tm;
		if (strptime(dateString.c_str(), "%d-%m-%Y %H:%M:%S", &tm)) {
			time_t raw_time = mktime(&tm);
			boost::posix_time::ptime local_time = boost::posix_time::from_time_t(raw_time);
			boost::posix_time::ptime utc_time = local_time + boost::posix_time::hours(2);
			app_->setWorldTime(boost::posix_time::to_time_t(utc_time));
		} else {
			REGEN_WARN("Invalid date string: " << dateString << ".");
		}
	}
	if (configurationNode->hasAttribute("time-scale")) {
		auto timeScale = configurationNode->getValue<double>("time-scale", 1.0);
		app_->setWorldTimeScale(timeScale);
		ui_.worldTimeFactor->setValue(timeScale);
	}
	if (configurationNode->hasAttribute("timestamp")) {
		auto time_d = configurationNode->getValue<double>("timestamp", 0.0);
		app_->setWorldTime(static_cast<time_t>(time_d));
	}

	/////////////////////////////
	/////////////////////////////
	/////////////////////////////

	// Update view...
	if (!viewNodes_.empty()) {
		activeView_ = viewNodes_.end();
		activeView_--;
		ViewNode &active = *activeView_;
		active.node->set_isHidden(GL_FALSE);
		app_->toplevelWidget()->setWindowTitle(QString(active.name.c_str()));
	}

	// Update text of FPS widget
	ref_ptr<MeshVector> fpsWidget =
			sceneParser.getResources()->getMesh(&sceneParser, "fps-widget");
	if (fpsWidget.get() != nullptr && !fpsWidget->empty()) {
		ref_ptr<TextureMappedText> text =
				ref_ptr<TextureMappedText>::dynamicCast(*fpsWidget->begin());
		if (text.get() != nullptr) {
			fbsWidgetUpdater_ = ref_ptr<UpdateFPS>::alloc(text);
			fbsWidgetUpdater_->startAnimation();
			REGEN_INFO("FPS widget found.");
		} else {
			fbsWidgetUpdater_ = ref_ptr<Animation>();
			REGEN_INFO("Unable to find FPS widget.");
		}
	} else {
		fbsWidgetUpdater_ = ref_ptr<Animation>();
		REGEN_INFO("Unable to find FPS widget.");
	}

	if (sceneParser.getRoot()->getFirstChild("node", "animations").get() != nullptr) {
		sceneParser.processNode(tree, "animations", "node");
	}

	if (anchors_.empty()) {
		ui_.playAnchor->setEnabled(false);
		ui_.nextAnchor->setEnabled(false);
	} else {
		ui_.playAnchor->setEnabled(true);
		ui_.nextAnchor->setEnabled(true);
	}

	// add mouse event handler
	auto mouseEventHandler = ref_ptr<SceneDisplayMouseHandler>::alloc(app_);
	mouseEventHandler->setAnimationName("mouse");
	mouseEventHandler->startAnimation();
	app_->connect(Scene::BUTTON_EVENT, mouseEventHandler);
	eventHandler_.emplace_back(mouseEventHandler);

	timeWidgetAnimation_ = ref_ptr<GameTimeAnimation>::alloc(this);
	timeWidgetAnimation_->startAnimation();
	animations_.emplace_back(timeWidgetAnimation_);
	loadAnim_ = ref_ptr<Animation>();
	lightStates_ = sceneParser.getResources()->getLights();
	AnimationManager::get().setSpatialIndices(spatialIndices_);
	AnimationManager::get().resetTime();

	AnimationManager::get().resume();
	REGEN_INFO("XML Scene Loaded.");
}

void SceneDisplayWidget::resizeEvent(QResizeEvent *event) {
	updateSize();
}
