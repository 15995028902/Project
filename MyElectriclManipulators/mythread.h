#ifndef MYTHREAD_H
#define MYTHREAD_H

#include <QThread>
#include <QSqlDatabase> //提供数据库访问、数据库连接操作
#include <QSqlQuery> //提供与数据库的交互操作
#include <QSqlError> //提供针对特定数据库在连接、查询时候发生错误时,提供关键的信息
#include <QDateTime>
#include <QDebug>
#include <Eigen/Eigen>

#include<QTimer>
#include<QTime>

#include "globalfunction.h"


class MyThread:public QThread
{
   Q_OBJECT

public:
    MyThread();

    QSqlDatabase  m_db;  //创建一个SQLite数据库
    QString m_dbname;
    bool  m_StopFlag;

    QTimer  *timer1, *timer2;
    QTime m_lastTime;

    void StartMotion();
    void StopMotion();
    bool WaitForStop();

    bool CreateSqlConnection();
    void CloseSql();
    bool WriteSql();

    void CalculateDesiredPosition();
    void DriveMotors();

private slots:
    void timer1update();
    void timer2update();

signals:   // 声明信号的关键字 signals，前面没有public/private/protected等限定符
      void SensorsSignal(int ID, int timeInterval,float value1, float value2,float value3,float value4,float value5,float value6 ); // 发送给SaveDataThread线程

};

#endif // MYTHREAD_H
