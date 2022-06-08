#include "SideWidget.h"

namespace cagd
{
    SideWidget::SideWidget(QWidget *parent): QWidget(parent)
    {
        setupUi(this);

        QPalette p = rotate_x_slider->palette();

        p.setColor(QPalette::Highlight, QColor(255,50,10).lighter());

        rotate_x_slider->setPalette(p);

        p = rotate_y_slider->palette();

        p.setColor(QPalette::Highlight, QColor(50,255,10).lighter());

        rotate_y_slider->setPalette(p);

        set_max_k_after_n_change(this->n->value());
        set_max_k_u_after_n_u_change(this->n_u->value());
        set_max_k_v_after_n_v_change(this->n_v->value());

        set_max_k_after_type_change(this->type->currentIndex());
        set_max_k_u_after_type_u_change(this->type_u->currentIndex());
        set_max_k_u_after_type_u_change(this->type_v->currentIndex());

        connect(this->n, SIGNAL(valueChanged(int)), this, SLOT(set_max_k_after_n_change(int)));
        connect(this->n_u, SIGNAL(valueChanged(int)), this, SLOT(set_max_k_u_after_n_u_change(int)));
        connect(this->n_v, SIGNAL(valueChanged(int)), this, SLOT(set_max_k_v_after_n_v_change(int)));

        connect(this->type, SIGNAL(currentIndexChanged(int)), this, SLOT(set_max_k_after_type_change(int)));
        connect(this->type_u, SIGNAL(currentIndexChanged(int)), this, SLOT(set_max_k_u_after_type_u_change(int)));
        connect(this->type_v, SIGNAL(currentIndexChanged(int)), this, SLOT(set_max_k_v_after_type_v_change(int)));
    }

    void SideWidget::set_max_k_after_n_change(int value)
    {
        if (this->type->currentIndex() != 0) // periodic
        {
            this->k->setMaximum(value + 1);
        }
        else
        {
            if (this->k->maximum() != 100)
            {
                this->k->setMaximum(100);
            }
        }
        update();
    }

    void SideWidget::set_max_k_u_after_n_u_change(int value)
    {
        if (this->type_u->currentIndex() != 0) // periodic
        {
            this->k_u->setMaximum(value + 1);
        }
        else
        {
            if (this->k_u->maximum() != 100)
            {
                this->k_u->setMaximum(100);
            }
        }
        update();
    }
    void SideWidget::set_max_k_v_after_n_v_change(int value)
    {
        if (this->type_v->currentIndex() != 0) // periodic
        {
            this->k_v->setMaximum(value + 1);
        }
        else
        {
            if (this->k_v->maximum() != 100)
            {
                this->k_v->setMaximum(100);
            }
        }
        update();
    }

    void SideWidget::set_max_k_after_type_change(int value)
    {
        if (value == 0) // periodic
        {
            this->k->setMaximum(100);
        }
        else
        {
            this->k->setMaximum(this->n->value() + 1);
        }
        update();
    }

    void SideWidget::set_max_k_u_after_type_u_change(int value)
    {
        if (value == 0) // periodic
        {
            this->k_u->setMaximum(100);
        }
        else
        {
            this->k_u->setMaximum(this->n_u->value() + 1);
        }
        update();
    }

    void SideWidget::set_max_k_v_after_type_v_change(int value)
    {
        if (value == 0) // periodic
        {
            this->k_v->setMaximum(100);
        }
        else
        {
            this->k_v->setMaximum(this->n_v->value() + 1);
        }
        update();
    }
}
