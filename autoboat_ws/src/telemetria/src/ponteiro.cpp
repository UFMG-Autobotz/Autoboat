#include "../include/telemetria/ponteiro.hpp"

namespace telemetria
{

Ponteiro::Ponteiro(QWidget *parent)
    : QWidget(parent),
      ang(0),
      cor(0, 127, 127, 191)
{}

void Ponteiro::paintEvent(QPaintEvent *)
{
    static const QPoint triang[3] = {
        QPoint(20, 70),
        QPoint(-20, 70),
        QPoint(0, -80)
    };

    int tam = qMin(width(), height());

    QPainter ptr(this);
    ptr.setRenderHint(QPainter::Antialiasing);
    ptr.translate(width() / 2, height() / 2);
    ptr.scale(tam/200., tam/200.);

    ptr.setPen(Qt::NoPen);
    ptr.setBrush(cor);

    ptr.save();
    ptr.rotate(ang);
    ptr.drawConvexPolygon(triang, 3);
    ptr.restore();

    ptr.setPen(cor);

    for (int i = 0; i < 8; ++i)
    {
        ptr.drawLine(88, 0, 96, 0);
        ptr.rotate(45.);
    }

    lower();
}

void Ponteiro::angulo(int novo_angulo)
{
    ang = novo_angulo;
    update();
}

void Ponteiro::colorir(QColor nova_cor)
{
    cor = nova_cor;
}

}
