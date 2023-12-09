#include "mainwindow.h"
#include "mythread.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    MyThread my_thread;
    my_thread.start();

    MainWindow w;
    w.show();

    return a.exec();
}
