#ifndef REGEN_REGEN_WIDGET_GL_H
#define REGEN_REGEN_WIDGET_GL_H

#include "RegenWidgetData.h"
#include <QOpenGLWidget>

namespace regen {
	class RegenWidgetGL : public QOpenGLWidget {
	public:
		RegenWidgetGL(QWidget *parent = nullptr, QWidget *container = nullptr);

		~RegenWidgetGL() override;

	protected:
		static QTimer *timer_;
		static std::vector<RegenWidgetGL *> widgets_;
		QWidget *container_;

		static void initializeTimer();
		static void updateAllWidgets();
	};
} // regen

#endif //REGEN_REGEN_WIDGET_GL_H
