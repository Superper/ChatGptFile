#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "ui_mwd.h"
#include <QMainWindow>
#include <thread>
#include "../Logmanager/LogManager.h"
class MainWindow : public QMainWindow
{
    Q_OBJECT

  public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

  private:
    Ui::MainWindow *ui;

    std::unique_ptr< std::thread> thread_;
  private slots:
    void StartSoftWare();
    void StopSoftWare();
};

#endif // MAINWINDOW_H
