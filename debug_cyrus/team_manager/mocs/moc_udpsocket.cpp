/****************************************************************************
** Meta object code from reading C++ file 'udpsocket.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.3.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../Team_manager/TeamManager/udpsocket.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'udpsocket.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.3.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_UdpSocket_t {
    QByteArrayData data[9];
    char stringdata[97];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_UdpSocket_t, stringdata) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_UdpSocket_t qt_meta_stringdata_UdpSocket = {
    {
QT_MOC_LITERAL(0, 0, 9),
QT_MOC_LITERAL(1, 10, 18),
QT_MOC_LITERAL(2, 29, 0),
QT_MOC_LITERAL(3, 30, 11),
QT_MOC_LITERAL(4, 42, 3),
QT_MOC_LITERAL(5, 46, 5),
QT_MOC_LITERAL(6, 52, 12),
QT_MOC_LITERAL(7, 65, 18),
QT_MOC_LITERAL(8, 84, 12)
    },
    "UdpSocket\0dataPacketReceived\0\0joinNetwork\0"
    "IP_\0port_\0leaveNetwork\0processPendingData\0"
    "timerTimeout"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_UdpSocket[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   49,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       3,    2,   52,    2, 0x0a /* Public */,
       3,    1,   57,    2, 0x2a /* Public | MethodCloned */,
       3,    0,   60,    2, 0x2a /* Public | MethodCloned */,
       6,    0,   61,    2, 0x0a /* Public */,
       7,    0,   62,    2, 0x08 /* Private */,
       8,    0,   63,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, QMetaType::QByteArray,    2,

 // slots: parameters
    QMetaType::Bool, QMetaType::QString, QMetaType::Int,    4,    5,
    QMetaType::Bool, QMetaType::QString,    4,
    QMetaType::Bool,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void UdpSocket::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        UdpSocket *_t = static_cast<UdpSocket *>(_o);
        switch (_id) {
        case 0: _t->dataPacketReceived((*reinterpret_cast< QByteArray(*)>(_a[1]))); break;
        case 1: { bool _r = _t->joinNetwork((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])));
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = _r; }  break;
        case 2: { bool _r = _t->joinNetwork((*reinterpret_cast< QString(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = _r; }  break;
        case 3: { bool _r = _t->joinNetwork();
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = _r; }  break;
        case 4: _t->leaveNetwork(); break;
        case 5: _t->processPendingData(); break;
        case 6: _t->timerTimeout(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (UdpSocket::*_t)(QByteArray );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&UdpSocket::dataPacketReceived)) {
                *result = 0;
            }
        }
    }
}

const QMetaObject UdpSocket::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_UdpSocket.data,
      qt_meta_data_UdpSocket,  qt_static_metacall, 0, 0}
};


const QMetaObject *UdpSocket::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *UdpSocket::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_UdpSocket.stringdata))
        return static_cast<void*>(const_cast< UdpSocket*>(this));
    return QObject::qt_metacast(_clname);
}

int UdpSocket::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 7)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 7;
    }
    return _id;
}

// SIGNAL 0
void UdpSocket::dataPacketReceived(QByteArray _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE
