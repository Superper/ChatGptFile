#ifndef MYCHECKBOX_H
#define MYCHECKBOX_H
#include <QCheckBox>

class MyCheckBox : public QCheckBox
{
    Q_OBJECT
  public:
    explicit MyCheckBox(QWidget *parent = nullptr);
    ~MyCheckBox() = default;

  protected:
    void mousePressEvent(QMouseEvent *event);
};

#endif // MYCHECKBOX_H