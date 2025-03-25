#ifndef REGEN_REGEN_WIDGET_DATA_H
#define REGEN_REGEN_WIDGET_DATA_H

#include "qt-application.h"

namespace regen {
	/**
	 * \brief Data structure for passing data to RegenWidget.
	 */
	struct RegenWidgetData {
		QtApplication *app;
		ref_ptr<StateNode> sceneNode;
		ref_ptr<ShaderInput> input;
	};
} // regen

#endif //REGEN_REGEN_WIDGET_DATA_H
