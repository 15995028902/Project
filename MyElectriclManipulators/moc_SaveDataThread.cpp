/****************************************************************************
** Meta object code from reading C++ file 'SaveDataThread.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.14.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "SaveDataThread.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'SaveDataThread.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.14.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_SaveDataThread_t {
    QByteArrayData data[11];
    char stringdata0[88];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_SaveDataThread_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_SaveDataThread_t qt_meta_stringdata_SaveDataThread = {
    {
QT_MOC_LITERAL(0, 0, 14), // "SaveDataThread"
QT_MOC_LITERAL(1, 15, 13), // "WriteXlsxSlot"
QT_MOC_LITERAL(2, 29, 0), // ""
QT_MOC_LITERAL(3, 30, 2), // "ID"
QT_MOC_LITERAL(4, 33, 12), // "timeInterval"
QT_MOC_LITERAL(5, 46, 6), // "value1"
QT_MOC_LITERAL(6, 53, 6), // "value2"
QT_MOC_LITERAL(7, 60, 6), // "value3"
QT_MOC_LITERAL(8, 67, 6), // "value4"
QT_MOC_LITERAL(9, 74, 6), // "value5"
QT_MOC_LITERAL(10, 81, 6) // "value6"

    },
    "SaveDataThread\0WriteXlsxSlot\0\0ID\0"
    "timeInterval\0value1\0value2\0value3\0"
    "value4\0value5\0value6"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_SaveDataThread[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       1,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    8,   19,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void, QMetaType::Int, QMetaType::Int, QMetaType::Float, QMetaType::Float, QMetaType::Float, QMetaType::Float, QMetaType::Float, QMetaType::Float,    3,    4,    5,    6,    7,    8,    9,   10,

       0        // eod
};

void SaveDataThread::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<SaveDataThread *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->WriteXlsxSlot((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< float(*)>(_a[3])),(*reinterpret_cast< float(*)>(_a[4])),(*reinterpret_cast< float(*)>(_a[5])),(*reinterpret_cast< float(*)>(_a[6])),(*reinterpret_cast< float(*)>(_a[7])),(*reinterpret_cast< float(*)>(_a[8]))); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject SaveDataThread::staticMetaObject = { {
    QMetaObject::SuperData::link<QThread::staticMetaObject>(),
    qt_meta_stringdata_SaveDataThread.data,
    qt_meta_data_SaveDataThread,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *SaveDataThread::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *SaveDataThread::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_SaveDataThread.stringdata0))
        return static_cast<void*>(this);
    return QThread::qt_metacast(_clname);
}

int SaveDataThread::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 1)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 1;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 1)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 1;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
