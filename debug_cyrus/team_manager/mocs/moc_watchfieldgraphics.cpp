/****************************************************************************
** Meta object code from reading C++ file 'watchfieldgraphics.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.3.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../cyrus/TeamManager/watchfieldgraphics.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'watchfieldgraphics.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.3.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_WatchFieldGraphics_t {
    QByteArrayData data[18];
    char stringdata[206];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_WatchFieldGraphics_t, stringdata) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_WatchFieldGraphics_t qt_meta_stringdata_WatchFieldGraphics = {
    {
QT_MOC_LITERAL(0, 0, 18),
QT_MOC_LITERAL(1, 19, 18),
QT_MOC_LITERAL(2, 38, 0),
QT_MOC_LITERAL(3, 39, 11),
QT_MOC_LITERAL(4, 51, 8),
QT_MOC_LITERAL(5, 60, 9),
QT_MOC_LITERAL(6, 70, 10),
QT_MOC_LITERAL(7, 81, 10),
QT_MOC_LITERAL(8, 92, 8),
QT_MOC_LITERAL(9, 101, 1),
QT_MOC_LITERAL(10, 103, 7),
QT_MOC_LITERAL(11, 111, 19),
QT_MOC_LITERAL(12, 131, 17),
QT_MOC_LITERAL(13, 149, 16),
QT_MOC_LITERAL(14, 166, 10),
QT_MOC_LITERAL(15, 177, 2),
QT_MOC_LITERAL(16, 180, 15),
QT_MOC_LITERAL(17, 196, 9)
    },
    "WatchFieldGraphics\0initializeQVectors\0"
    "\0const char*\0filename\0updateGUI\0"
    "drawBounds\0setCounter\0long int\0c\0"
    "getSize\0changeCyrusToYellow\0"
    "changeCyrusToBlue\0updateRobotState\0"
    "RobotState\0st\0updateBallState\0BallState"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_WatchFieldGraphics[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       9,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   59,    2, 0x0a /* Public */,
       5,    0,   62,    2, 0x0a /* Public */,
       6,    0,   63,    2, 0x0a /* Public */,
       7,    1,   64,    2, 0x0a /* Public */,
      10,    0,   67,    2, 0x0a /* Public */,
      11,    0,   68,    2, 0x0a /* Public */,
      12,    0,   69,    2, 0x0a /* Public */,
      13,    1,   70,    2, 0x0a /* Public */,
      16,    1,   73,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 8,    9,
    0x80000000 | 8,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 14,   15,
    QMetaType::Void, 0x80000000 | 17,   15,

       0        // eod
};

void WatchFieldGraphics::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        WatchFieldGraphics *_t = static_cast<WatchFieldGraphics *>(_o);
        switch (_id) {
        case 0: _t->initializeQVectors((*reinterpret_cast< const char*(*)>(_a[1]))); break;
        case 1: _t->updateGUI(); break;
        case 2: _t->drawBounds(); break;
        case 3: _t->setCounter((*reinterpret_cast< long int(*)>(_a[1]))); break;
        case 4: { long int _r = _t->getSize();
            if (_a[0]) *reinterpret_cast< long int*>(_a[0]) = _r; }  break;
        case 5: _t->changeCyrusToYellow(); break;
        case 6: _t->changeCyrusToBlue(); break;
        case 7: _t->updateRobotState((*reinterpret_cast< const RobotState(*)>(_a[1]))); break;
        case 8: _t->updateBallState((*reinterpret_cast< const BallState(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject WatchFieldGraphics::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_WatchFieldGraphics.data,
      qt_meta_data_WatchFieldGraphics,  qt_static_metacall, 0, 0}
};


const QMetaObject *WatchFieldGraphics::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *WatchFieldGraphics::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_WatchFieldGraphics.stringdata))
        return static_cast<void*>(const_cast< WatchFieldGraphics*>(this));
    return QWidget::qt_metacast(_clname);
}

int WatchFieldGraphics::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 9)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 9;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 9)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 9;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
