#include "RegenWidgetGL.h"
#include <QTimer>

using namespace regen;

QTimer *RegenWidgetGL::timer_ = nullptr;
std::vector<RegenWidgetGL *> RegenWidgetGL::widgets_;

RegenWidgetGL::RegenWidgetGL(QWidget *parent, QWidget *container)
	: QOpenGLWidget(parent),
	  container_(container) {
    widgets_.push_back(this);
    initializeTimer();
}

RegenWidgetGL::~RegenWidgetGL() {
    auto it = std::find(widgets_.begin(), widgets_.end(), this);
    if (it != widgets_.end()) {
        widgets_.erase(it);
    }
}

void RegenWidgetGL::initializeTimer() {
    if (!timer_) {
        timer_ = new QTimer();
        QObject::connect(timer_, &QTimer::timeout, []() {
            updateAllWidgets();
        });
        timer_->start(16); // Approximately 60 FPS
    }
}

void RegenWidgetGL::updateAllWidgets() {
    for (auto widget : widgets_) {
        widget->update();
    }
}
