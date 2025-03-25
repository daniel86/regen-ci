#ifndef REGEN_REGEN_WIDGET_H
#define REGEN_REGEN_WIDGET_H

#include "RegenWidgetData.h"
#include <QWidget>

namespace regen {
	class RegenWidget : public QWidget {
		Q_OBJECT

	public:
		explicit RegenWidget(const RegenWidgetData &data, QWidget *parent = nullptr);

		~RegenWidget() override = default;

		auto node() const -> const ref_ptr<StateNode> & { return node_; }

		auto input() const -> const ref_ptr<ShaderInput> & { return input_; }

		auto app() const -> QtApplication * { return app_; }

	protected:
		QtApplication *app_;
		ref_ptr<StateNode> node_;
		ref_ptr<ShaderInput> input_;
	};
} // regen

#endif //REGEN_REGEN_WIDGET_H
