#ifndef SAVEDATATHREAD_H
#define SAVEDATATHREAD_H

#include <QObject>
#include <QThread>

#include <QSqlDatabase>
#include <QSqlQuery>
#include <QSqlError>
#include <QFile>
#include <QDir>
#include <QDebug>

#include <QDateTime>
#include <QPointF>
#include<QString>

#include "xlsxdocument.h"
#include "xlsxformat.h"
#include "xlsxcellrange.h"
#include "xlsxworksheet.h"

#include <vector>
using std::vector;

class SaveDataThread : public QThread
{
    Q_OBJECT
public:
    SaveDataThread();
public:
    QSqlDatabase  m_db;
    QString  m_dbname;
    QXlsx::Document *m_xlsx;//用来保存PID参数到excel文件中
    QString m_xlsxname;//文件路径

public:
    void CreateXlsx();

    bool CreateSqlConnection();
    void CloseSQL();
    bool WriteSql();

public slots: //自定义槽函数，接收信号，需要在cpp中实现
      void WriteXlsxSlot(int ID, int timeInterval,float value1, float value2,float value3,float value4,float value5,float value6);
};

#endif // SAVEDATATHREAD_H
