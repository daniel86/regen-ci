#include "RegenWidget.h"

using namespace regen;

RegenWidget::RegenWidget(const RegenWidgetData &data, QWidget *parent)
	: QWidget(parent),
	  app_(data.app),
	  node_(data.sceneNode),
	  input_(data.input) {
}
