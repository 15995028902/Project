#ifndef MYSQL_H
#define MYSQL_H


class MySQL
{
public:
    MySQL();

    bool CreateSqlConnection();
    void CloseSql();
    bool WriteSql();
};

#endif // MYSQL_H
