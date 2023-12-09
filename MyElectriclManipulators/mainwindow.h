#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMessageBox>

#include "globalvars.h"
#include "globalfunction.h"
#include "electrickinematics.h"
#include "mythread.h"
#include "SaveDataThread.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MyThread m_myThread;
    SaveDataThread m_SaveDataThread;

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_Enable_clicked();
    void on_enable_3_clicked();

    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_4_clicked();

    void on_pushButton_3_clicked();

private:
    Ui::MainWindow *ui;
};
#endif // MAINWINDOW_H
