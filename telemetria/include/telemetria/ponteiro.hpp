#ifndef telemetria_PONTEIRO_HPP_
#define telemetria_PONTEIRO_HPP_

#include <QWidget>
#include <QPainter>

namespace telemetria
{

class Ponteiro : public QWidget
{
    Q_OBJECT

private:
    int ang;
    QColor cor;

public:
    Ponteiro(QWidget *parent = 0);
    void angulo(int novo_angulo);

protected:
    void paintEvent(QPaintEvent *event);
    void colorir(QColor nova_cor);
};

} // namespace telemetria

#endif // telemetria_PONTEIRO_HPP_
