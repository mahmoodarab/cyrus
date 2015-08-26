/****************************************************************************
** Meta object code from reading C++ file 'builtindebug.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.3.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../cyrus2014/server/debug-tools/builtindebug.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'builtindebug.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.3.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_BuiltInDebug_t {
    QByteArrayData data[18];
    char stringdata[184];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_BuiltInDebug_t, stringdata) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_BuiltInDebug_t qt_meta_stringdata_BuiltInDebug = {
    {
QT_MOC_LITERAL(0, 0, 12),
QT_MOC_LITERAL(1, 13, 10),
QT_MOC_LITERAL(2, 24, 0),
QT_MOC_LITERAL(3, 25, 5),
QT_MOC_LITERAL(4, 31, 4),
QT_MOC_LITERAL(5, 36, 8),
QT_MOC_LITERAL(6, 45, 16),
QT_MOC_LITERAL(7, 62, 14),
QT_MOC_LITERAL(8, 77, 1),
QT_MOC_LITERAL(9, 79, 22),
QT_MOC_LITERAL(10, 102, 10),
QT_MOC_LITERAL(11, 113, 2),
QT_MOC_LITERAL(12, 116, 16),
QT_MOC_LITERAL(13, 133, 11),
QT_MOC_LITERAL(14, 145, 3),
QT_MOC_LITERAL(15, 149, 21),
QT_MOC_LITERAL(16, 171, 9),
QT_MOC_LITERAL(17, 181, 2)
    },
    "BuiltInDebug\0plotSignal\0\0value\0name\0"
    "category\0plotPacketSignal\0Plotter_Packet\0"
    "p\0updateRobotStateSignal\0RobotState\0"
    "rs\0newMessageSignal\0const char*\0msg\0"
    "updateBallStateSignal\0BallState\0bs"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_BuiltInDebug[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       5,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    3,   39,    2, 0x06 /* Public */,
       6,    1,   46,    2, 0x06 /* Public */,
       9,    1,   49,    2, 0x06 /* Public */,
      12,    2,   52,    2, 0x06 /* Public */,
      15,    1,   57,    2, 0x06 /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::Double, QMetaType::QString, QMetaType::QString,    3,    4,    5,
    QMetaType::Void, 0x80000000 | 7,    8,
    QMetaType::Void, 0x80000000 | 10,   11,
    QMetaType::Void, 0x80000000 | 13, QMetaType::QString,   14,    5,
    QMetaType::Void, 0x80000000 | 16,   17,

       0        // eod
};

void BuiltInDebug::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        BuiltInDebug *_t = static_cast<BuiltInDebug *>(_o);
        switch (_id) {
        case 0: _t->plotSignal((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2])),(*reinterpret_cast< QString(*)>(_a[3]))); break;
        case 1: _t->plotPacketSignal((*reinterpret_cast< Plotter_Packet(*)>(_a[1]))); break;
        case 2: _t->updateRobotStateSignal((*reinterpret_cast< RobotState(*)>(_a[1]))); break;
        case 3: _t->newMessageSignal((*reinterpret_cast< const char*(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2]))); break;
        case 4: _t->updateBallStateSignal((*reinterpret_cast< BallState(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (BuiltInDebug::*_t)(double , QString , QString );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&BuiltInDebug::plotSignal)) {
                *result = 0;
            }
        }
        {
            typedef void (BuiltInDebug::*_t)(Plotter_Packet );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&BuiltInDebug::plotPacketSignal)) {
                *result = 1;
            }
        }
        {
            typedef void (BuiltInDebug::*_t)(RobotState );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&BuiltInDebug::updateRobotStateSignal)) {
                *result = 2;
            }
        }
        {
            typedef void (BuiltInDebug::*_t)(const char * , QString );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&BuiltInDebug::newMessageSignal)) {
                *result = 3;
            }
        }
        {
            typedef void (BuiltInDebug::*_t)(BallState );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&BuiltInDebug::updateBallStateSignal)) {
                *result = 4;
            }
        }
    }
}

const QMetaObject BuiltInDebug::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_BuiltInDebug.data,
      qt_meta_data_BuiltInDebug,  qt_static_metacall, 0, 0}
};


const QMetaObject *BuiltInDebug::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *BuiltInDebug::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_BuiltInDebug.stringdata))
        return static_cast<void*>(const_cast< BuiltInDebug*>(this));
    if (!strcmp(_clname, "Debugger"))
        return static_cast< Debugger*>(const_cast< BuiltInDebug*>(this));
    return QObject::qt_metacast(_clname);
}

int BuiltInDebug::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 5)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 5;
    }
    return _id;
}

// SIGNAL 0
void BuiltInDebug::plotSignal(double _t1, QString _t2, QString _t3)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void BuiltInDebug::plotPacketSignal(Plotter_Packet _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void BuiltInDebug::updateRobotStateSignal(RobotState _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void BuiltInDebug::newMessageSignal(const char * _t1, QString _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void BuiltInDebug::updateBallStateSignal(BallState _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}
QT_END_MOC_NAMESPACE
