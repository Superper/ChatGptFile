#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "ui_mwd.h"
#include <QMainWindow>
#include <thread>
#include <mutex>
#include <QUdpSocket>

class MainWindow : public QMainWindow
{
    Q_OBJECT

  public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void UIControlDisplay(const std::string &msg);

    std::string ip_se;
    uint16_t port_se;

    std::string ip_msif;
    uint16_t port_msif;

    std::string ip_cd;
    uint16_t port_cd;

    std::string ip_ds;
    uint16_t port_ds;

  private:
  std::mutex mx_;
  Ui::MainWindow *ui;
    QUdpSocket udp;

  std::unique_ptr<std::thread> thread_;
  signals:
    void SetPlainTextEditSignal(QString msg);
private slots:
    void on_pushButton_clicked();
    void SetPlainTextEdit(QString msg);
};

#endif // MAINWINDOW_H
