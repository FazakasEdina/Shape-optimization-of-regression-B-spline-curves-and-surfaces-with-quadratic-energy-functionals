#pragma once

#include <QWidget>
#include "ui_SideWidget.h"

namespace cagd
{
    class SideWidget: public QWidget, public Ui::SideWidget
    {
        Q_OBJECT
    public:
        // special and default constructor
        SideWidget(QWidget *parent = 0);
    public slots:
        void set_max_k_after_n_change(int value);
        void set_max_k_u_after_n_u_change(int value);
        void set_max_k_v_after_n_v_change(int value);

        void set_max_k_after_type_change(int value);
        void set_max_k_u_after_type_u_change(int value);
        void set_max_k_v_after_type_v_change(int value);
    };
}
