#include "SaveDataThread.h"

SaveDataThread::SaveDataThread()
{

}

void SaveDataThread::CreateXlsx()
{
    QDateTime time = QDateTime::currentDateTime();//获取系统现在的时间
    QString str = time.toString("yyyy年-MM月-dd日 hh时-mm分-ss秒 ddd"); //设置显示格式

    m_xlsxname =  str + ".xlsx"; // 可以含有路径，作为文件的存放路径
    m_xlsx= new QXlsx::Document(m_xlsxname);//将变量m_xlsx指向建立的excel文件

    m_xlsx->addSheet("期望位移");//增加一页，命名为 data1,此页为当前页，操作都是针对此页进行的
    m_xlsx->selectSheet("期望位移");//设置为当前页
    //excel表格下标是1开始的，即第一个单元格位置是(1,1)
    m_xlsx->write(1, 1, "时间");
    m_xlsx->write(1, 2, "底座驱动器期望角度");
    m_xlsx->write(1, 3, "大臂驱动器期望角度");
    m_xlsx->write(1, 4, "二臂驱动器期望角度");
    m_xlsx->write(1, 5, "摇杆驱动器期望角度");
    m_xlsx->write(1, 6, "侧摆驱动器期望角度");
    m_xlsx->write(1, 7, "末端回转驱动器期望角度");

    m_xlsx->addSheet("底座实际信息");//增加一页，此页为当前页，操作都是针对此页进行的
    m_xlsx->selectSheet("底座实际信息");//设置为当前页
    m_xlsx->write(1, 1, "时间");
    m_xlsx->write(1, 2, "底座驱动器实际角度");
    m_xlsx->write(1, 3, "底座驱动器实际速度(rpm)");
    m_xlsx->write(1, 4, "底座驱动器A相电流(A)");

    m_xlsx->addSheet("大臂实际信息");//增加一页，此页为当前页，操作都是针对此页进行的
    m_xlsx->selectSheet("大臂实际信息");//设置为当前页
    m_xlsx->write(1, 1, "时间");
    m_xlsx->write(1, 2, "大臂驱动器实际位移");
    m_xlsx->write(1, 3, "大臂驱动器实际速度(rpm)");
    m_xlsx->write(1, 4, "大臂驱动器A相电流(A)");

    m_xlsx->addSheet("二臂实际信息");//增加一页，此页为当前页，操作都是针对此页进行的
    m_xlsx->selectSheet("二臂实际信息");//设置为当前页
    m_xlsx->write(1, 1, "时间");
    m_xlsx->write(1, 2, "二臂驱动器实际位移");
    m_xlsx->write(1, 3, "二臂驱动器实际速度(rpm)");
    m_xlsx->write(1, 4, "二臂驱动器A相电流(A)");

    m_xlsx->addSheet("摇杆实际信息");//增加一页，此页为当前页，操作都是针对此页进行的
    m_xlsx->selectSheet("摇杆实际信息");//设置为当前页
    m_xlsx->write(1, 1, "时间");
    m_xlsx->write(1, 2, "摇杆驱动器实际位移");
    m_xlsx->write(1, 3, "摇杆驱动器实际速度(rpm)");
    m_xlsx->write(1, 4, "摇杆驱动器A相电流(A)");

    m_xlsx->addSheet("侧摆实际信息");//增加一页，此页为当前页，操作都是针对此页进行的
    m_xlsx->selectSheet("侧摆实际信息");//设置为当前页
    m_xlsx->write(1, 1, "时间");
    m_xlsx->write(1, 2, "侧摆驱动器实际位移");
    m_xlsx->write(1, 3, "侧摆驱动器实际速度(rpm)");
    m_xlsx->write(1, 4, "侧摆驱动器A相电流(A)");

    m_xlsx->addSheet("末端回转实际信息");//增加一页，此页为当前页，操作都是针对此页进行的
    m_xlsx->selectSheet("末端回转实际信息");//设置为当前页
    m_xlsx->write(1, 1, "时间");
    m_xlsx->write(1, 2, "末端回转驱动器实际位移");
    m_xlsx->write(1, 3, "末端回转驱动器实际速度(rpm)");
    m_xlsx->write(1, 4, "末端回转驱动器A相电流(A)");

    m_xlsx->saveAs(m_xlsxname);
    delete m_xlsx;
}

/**
 * @brief SaveDataThread::WriteXlsxSlot
 * @param ID 表示excel的哪一页。0-期望位移，1-底座实际信息，2-大臂实际信息，3-二臂实际信息，4-摇杆实际信息，5-侧摆实际信息，6-末端回转实际信息
 * @param timeInterval
 * @param value1
 * @param value2
 * @param value3
 * @param value4
 * @param value5
 * @param value6
 */
void SaveDataThread::WriteXlsxSlot(int ID, int timeInterval,float value1, float value2,float value3,float value4,float value5,float value6)
{
    static int i=0;
    if(i==0)//利用静态变量，保证CreateXlsx()函数只执行一次
    {
        CreateXlsx();
        i=1;
    }

    static vector<float> tempVector0[7], tempVector1[4], tempVector2[4], tempVector3[4], tempVector4[4], tempVector5[4], tempVector6[4];

    switch (ID) {
    case 0://期望值
        tempVector0[0].push_back(timeInterval);
        tempVector0[1].push_back(value1);
        tempVector0[2].push_back(value2);
        tempVector0[3].push_back(value3);
        tempVector0[4].push_back(value4);
        tempVector0[5].push_back(value5);
        tempVector0[6].push_back(value6);
        break;
    case 1://底座实际信息
        tempVector1[0].push_back(timeInterval);
        tempVector1[1].push_back(value1);
        tempVector1[2].push_back(value2);
        tempVector1[3].push_back(value3);
        break;
    case 2://大臂实际信息
        tempVector2[0].push_back(timeInterval);
        tempVector2[1].push_back(value1);
        tempVector2[2].push_back(value2);
        tempVector2[3].push_back(value3);
        break;
    case 3://二臂实际信息
        tempVector3[0].push_back(timeInterval);
        tempVector3[1].push_back(value1);
        tempVector3[2].push_back(value2);
        tempVector3[3].push_back(value3);
        break;
    case 4://摇杆实际信息
        tempVector4[0].push_back(timeInterval);
        tempVector4[1].push_back(value1);
        tempVector4[2].push_back(value2);
        tempVector4[3].push_back(value3);
        break;
    case 5://侧摆实际信息
        tempVector5[0].push_back(timeInterval);
        tempVector5[1].push_back(value1);
        tempVector5[2].push_back(value2);
        tempVector5[3].push_back(value3);
        break;
    case 6://末端回转实际信息
        tempVector6[0].push_back(timeInterval);
        tempVector6[1].push_back(value1);
        tempVector6[2].push_back(value2);
        tempVector6[3].push_back(value3);
        break;
    }

    const int COUNT =100;

    if( tempVector0[0].size()>=COUNT )
    {
        if(!QFile::exists(m_xlsxname)) return;
        //如果excel文件存在，则开始文件数据写入
        m_xlsx = new QXlsx::Document(m_xlsxname);
        m_xlsx->selectSheet("期望位移");//设置为当前页
        int rowcount;
        rowcount= m_xlsx->dimension().rowCount();//获取当前页的行数
        for(int i=0;i<=tempVector0[0].size()-1; i++)
        {
            //excel表格下标是1开始的，即第一个单元格位置是(1,1)
            m_xlsx->write(rowcount+1+i,1,tempVector0[0].at(i));//时间
            m_xlsx->write(rowcount+1+i,2,tempVector0[1].at(i));//底座驱动器期望角度
            m_xlsx->write(rowcount+1+i,3,tempVector0[2].at(i));//大臂驱动器期望角度
            m_xlsx->write(rowcount+1+i,4,tempVector0[3].at(i));//二臂驱动器期望角度
            m_xlsx->write(rowcount+1+i,5,tempVector0[4].at(i));//摇杆驱动器期望角度
            m_xlsx->write(rowcount+1+i,6,tempVector0[5].at(i));//侧摆驱动器期望角度
            m_xlsx->write(rowcount+1+i,7,tempVector0[6].at(i));//末端回转驱动器期望角度
        }

        for(int i=0;i<=6;i++) tempVector0[i].clear();

        m_xlsx->save();
        delete m_xlsx;
    }

    if(tempVector1[0].size()>=COUNT)
    {
        if(!QFile::exists(m_xlsxname)) return;
        //如果excel文件存在，则开始文件数据写入
        m_xlsx = new QXlsx::Document(m_xlsxname);
        int rowcount;
        m_xlsx->selectSheet("底座实际信息");//设置为当前页
        rowcount= m_xlsx->dimension().rowCount();//获取当前页的行数
        for(int i=0;i<=tempVector1[0].size()-1; i++)
        {
            //excel表格下标是1开始的，即第一个单元格位置是(1,1)
            m_xlsx->write(rowcount+1+i,1,tempVector1[0].at(i));//时间
            m_xlsx->write(rowcount+1+i,2,tempVector1[1].at(i));//底座驱动器实际角度
            m_xlsx->write(rowcount+1+i,3,tempVector1[2].at(i));//底座驱动器实际速度
            m_xlsx->write(rowcount+1+i,4,tempVector1[3].at(i));//底座驱动器A相电流
        }

        for(int i=0;i<=3;i++) tempVector1[i].clear();

        m_xlsx->save();
        delete m_xlsx;
    }

    if(tempVector2[0].size()>=COUNT)
    {
        if(!QFile::exists(m_xlsxname)) return;
        //如果excel文件存在，则开始文件数据写入
        m_xlsx = new QXlsx::Document(m_xlsxname);
        int rowcount;
        m_xlsx->selectSheet("大臂实际信息");//设置为当前页
        rowcount= m_xlsx->dimension().rowCount();//获取当前页的行数
        for(int i=0;i<=tempVector2[0].size()-1; i++)
        {
            //excel表格下标是1开始的，即第一个单元格位置是(1,1)
            m_xlsx->write(rowcount+1+i,1,tempVector2[0].at(i));//时间
            m_xlsx->write(rowcount+1+i,2,tempVector2[1].at(i));//大臂驱动器实际角度
            m_xlsx->write(rowcount+1+i,3,tempVector2[2].at(i));//大臂驱动器实际速度
            m_xlsx->write(rowcount+1+i,4,tempVector2[3].at(i));//大臂驱动器A相电流
        }

        for(int i=0;i<=3;i++) tempVector2[i].clear();

        m_xlsx->save();
        delete m_xlsx;
    }

    if(tempVector3[0].size()>=COUNT)
    {
        if(!QFile::exists(m_xlsxname)) return;
        //如果excel文件存在，则开始文件数据写入
        m_xlsx = new QXlsx::Document(m_xlsxname);
        int rowcount;
        m_xlsx->selectSheet("二臂实际信息");//设置为当前页
        rowcount= m_xlsx->dimension().rowCount();//获取当前页的行数
        for(int i=0;i<=tempVector3[0].size()-1; i++)
        {
            //excel表格下标是1开始的，即第一个单元格位置是(1,1)
            m_xlsx->write(rowcount+1+i,1,tempVector3[0].at(i));//时间
            m_xlsx->write(rowcount+1+i,2,tempVector3[1].at(i));//二臂驱动器实际角度
            m_xlsx->write(rowcount+1+i,3,tempVector3[2].at(i));//二臂驱动器实际速度
            m_xlsx->write(rowcount+1+i,4,tempVector3[3].at(i));//二臂驱动器A相电流
        }

        for(int i=0;i<=3;i++) tempVector3[i].clear();

        m_xlsx->save();
        delete m_xlsx;
    }

    if(tempVector4[0].size()>=COUNT)
    {
        if(!QFile::exists(m_xlsxname)) return;
        //如果excel文件存在，则开始文件数据写入
        m_xlsx = new QXlsx::Document(m_xlsxname);
        int rowcount;

        m_xlsx->selectSheet("摇杆实际信息");//设置为当前页
        rowcount= m_xlsx->dimension().rowCount();//获取当前页的行数
        for(int i=0;i<=tempVector4[0].size()-1; i++)
        {
            //excel表格下标是1开始的，即第一个单元格位置是(1,1)
            m_xlsx->write(rowcount+1+i,1,tempVector4[0].at(i));//时间
            m_xlsx->write(rowcount+1+i,2,tempVector4[1].at(i));//摇杆驱动器实际角度
            m_xlsx->write(rowcount+1+i,3,tempVector4[2].at(i));//摇杆驱动器实际速度
            m_xlsx->write(rowcount+1+i,4,tempVector4[3].at(i));//摇杆驱动器A相电流
        }

        for(int i=0;i<=3;i++) tempVector4[i].clear();

        m_xlsx->save();
        delete m_xlsx;
    }

    if(tempVector5[0].size()>=COUNT)
    {
        if(!QFile::exists(m_xlsxname)) return;
        //如果excel文件存在，则开始文件数据写入
        m_xlsx = new QXlsx::Document(m_xlsxname);
        int rowcount;

        m_xlsx->selectSheet("侧摆实际信息");//设置为当前页
        rowcount= m_xlsx->dimension().rowCount();//获取当前页的行数
        for(int i=0;i<=tempVector5[0].size()-1; i++)
        {
            //excel表格下标是1开始的，即第一个单元格位置是(1,1)
            m_xlsx->write(rowcount+1+i,1,tempVector5[0].at(i));//时间
            m_xlsx->write(rowcount+1+i,2,tempVector5[1].at(i));//侧摆驱动器实际角度
            m_xlsx->write(rowcount+1+i,3,tempVector5[2].at(i));//侧摆驱动器实际速度
            m_xlsx->write(rowcount+1+i,4,tempVector5[3].at(i));//侧摆驱动器A相电流
        }

        for(int i=0;i<=3;i++) tempVector5[i].clear();

        m_xlsx->save();
        delete m_xlsx;
    }

    if(tempVector6[0].size()>=COUNT)
    {
        if(!QFile::exists(m_xlsxname)) return;
        //如果excel文件存在，则开始文件数据写入
        m_xlsx = new QXlsx::Document(m_xlsxname);
        int rowcount;

        m_xlsx->selectSheet("末端回转实际信息");//设置为当前页
        rowcount= m_xlsx->dimension().rowCount();//获取当前页的行数
        for(int i=0;i<=tempVector6[0].size()-1; i++)
        {
            //excel表格下标是1开始的，即第一个单元格位置是(1,1)
            m_xlsx->write(rowcount+1+i,1,tempVector6[0].at(i));//时间
            m_xlsx->write(rowcount+1+i,2,tempVector6[1].at(i));//末端回转驱动器实际角度
            m_xlsx->write(rowcount+1+i,3,tempVector6[2].at(i));//末端回转驱动器实际速度
            m_xlsx->write(rowcount+1+i,4,tempVector6[3].at(i));//末端回转驱动器A相电流
        }

        for(int i=0;i<=3;i++) tempVector6[i].clear();

        m_xlsx->save();
        delete m_xlsx;
    }

}




#if 0
bool SaveDataThread::CreateSqlConnection()
{
    QDateTime time = QDateTime::currentDateTime();//获取系统现在的时间
    m_dbname = time.toString("yyyy年-MM月-dd日 hh时-mm分-ss秒 ddd"); //设置显示格式

    m_db = QSqlDatabase::addDatabase("QSQLITE"); //加载数据库驱动
    m_db.setHostName("localhost"); //设置数据库主机名

    m_db.setDatabaseName(m_dbname); //表示创建一个名为gl_dbname的SQLITE数据库，如果这个数据库不存在，则创建；数据库存在，后续的数据库操作在已有的数据库上进行
 //   gl_db.setUserName("root");//设置数据库用户名
   // gl_db.setPassword("");//设置数据库密码

    if (!m_db.open()) {
        qDebug()<<"无法创建数据库连接！";//printf( "无法创建数据库连接！");
        return false;
    }

    //下面来创建表
    // 如果 数据库中已经存在同名的表， 那么下面的代码不会执行
     QSqlQuery sql_query(m_db); //这句话非常重要！！！！！！！！！！！！！！！

    // gl_query=sql_query;//利用局部变量，赋值给全局变量gl_query
   // 使数据库支持中文
     sql_query.exec("SET NAMES 'Latin1'");

     bool p=sql_query.exec("delete table Sampled_Desired_Data'");//删除已经存在的数据表CaptureMotion
     Q_UNUSED(p);

    //创建gl_db数据库文件的表Sampled_Desired_Data
   p= sql_query.exec("create table Sampled_Desired_Data ( Base_time double primary key, Base_SampledAngle double, Base_DesiredAngle double)");

   sql_query.finish();
    m_db.close();

    return true;
}


void SaveDataThread::CloseSQL()
{
    m_db.close();
}


//全局函数，采用事务操作
bool SaveDataThread::WriteSql()
{
    if (!m_db.open()) {

     //printf( "无法创建数据库连接！");
        qDebug()<<"open db fail！";
     return false;
    }

    QSqlQuery sql_query(m_db); //这句话非常重要！！！！！！！！！！！！！！！
    //gl_query=sql_query;//利用局部变量，赋值给全局变量gl_query

    sql_query.exec( "select count(*) from Sampled_Desired_Data");

    long numberOfRows = 0;
    if(sql_query.last())//定位到数据表最后一行
    {
        numberOfRows =  sql_query.at() + 1; //获取现在有多少列
    }

    QString strTime, strAngle, strToruqe;

   if(!m_db.transaction())//开始事务处理，插入大批量数据
     qDebug()<<"gl_db.transaction() fail";

//  named binding方式：
    if(!  sql_query.prepare("INSERT INTO Sampled_Desired_Data(Base_time, Base_SampledAngle, Base_DesiredAngle)"
           "VALUES (:Base_time, :Base_SampledAngle, :Base_DesiredAngle)") )
        qDebug()<<"gl_db.prepare() fail";

    int DATACOUNT =5000;
    for(long i=0; i< DATACOUNT; i++)//DATACOUNT
    {

        numberOfRows=numberOfRows+i;

//重要提示： size()>=2的原因是为了防止线程1的写操作和线程2的读/擦除操作同时施加于变量的同一个内存块。！！！这是不用互斥锁Mutex，实现多线程的关键

        if( gl_Base_AngleStruct.m_DesiredData_Save.size()>=2){

            sql_query.bindValue(":Base_DesiredAngle", gl_Base_AngleStruct.m_DesiredData_Save.at(0));
            gl_Base_AngleStruct.m_DesiredData_Save.erase(gl_Base_AngleStruct.m_DesiredData_Save.begin());
        }
        if( gl_Base_AngleStruct.m_SampledData_Save.size()>=2){
            sql_query.bindValue(":Base_SampledAngle", gl_Base_AngleStruct.m_SampledData_Save.at(0));
            gl_Base_AngleStruct.m_SampledData_Save.erase(gl_Base_AngleStruct.m_SampledData_Save.begin());
        }
        if( gl_Base_AngleStruct.m_Time_Save.size()>=2){
            sql_query.bindValue(":Base_time", gl_Base_AngleStruct.m_Time_Save.at(0));
            gl_Base_AngleStruct.m_Time_Save.erase(gl_Base_AngleStruct.m_Time_Save.begin());
        }


        if(!sql_query.exec()){
            QSqlError lastError = sql_query.lastError();
            //qDebug()<<lastError.driverText()<<QString(QObject::tr("gl_query.exec() fail"));
        }
    }

    bool i=m_db.commit();//事务提交，数据库写入结束

    sql_query.finish();
    m_db.close();
    return true;
}
#endif
