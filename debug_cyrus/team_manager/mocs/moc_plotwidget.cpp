/****************************************************************************
** Meta object code from reading C++ file 'plotwidget.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.3.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../Team_manager/TeamManager/plot-manager/plotwidget.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'plotwidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.3.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_PlotWidget_t {
    QByteArrayData data[14];
    char stringdata[199];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_PlotWidget_t, stringdata) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_PlotWidget_t qt_meta_stringdata_PlotWidget = {
    {
QT_MOC_LITERAL(0, 0, 10),
QT_MOC_LITERAL(1, 11, 7),
QT_MOC_LITERAL(2, 19, 0),
QT_MOC_LITERAL(3, 20, 14),
QT_MOC_LITERAL(4, 35, 16),
QT_MOC_LITERAL(5, 52, 10),
QT_MOC_LITERAL(6, 63, 10),
QT_MOC_LITERAL(7, 74, 12),
QT_MOC_LITERAL(8, 87, 21),
QT_MOC_LITERAL(9, 109, 9),
QT_MOC_LITERAL(10, 119, 22),
QT_MOC_LITERAL(11, 142, 25),
QT_MOC_LITERAL(12, 168, 7),
QT_MOC_LITERAL(13, 176, 22)
    },
    "PlotWidget\0closeMe\0\0setMeMinimized\0"
    "selectionChanged\0mousePress\0mouseWheel\0"
    "graphClicked\0QCPAbstractPlottable*\0"
    "plottable\0on_closeButton_clicked\0"
    "on_minimizeButton_clicked\0checked\0"
    "on_pauseButton_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_PlotWidget[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       9,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   59,    2, 0x06 /* Public */,
       3,    2,   62,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       4,    0,   67,    2, 0x08 /* Private */,
       5,    0,   68,    2, 0x08 /* Private */,
       6,    0,   69,    2, 0x08 /* Private */,
       7,    1,   70,    2, 0x08 /* Private */,
      10,    0,   73,    2, 0x08 /* Private */,
      11,    1,   74,    2, 0x08 /* Private */,
      13,    1,   77,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, QMetaType::QString,    2,
    QMetaType::Void, QMetaType::QString, QMetaType::Bool,    2,    2,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 8,    9,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,   12,
    QMetaType::Void, QMetaType::Bool,   12,

       0        // eod
};

void PlotWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        PlotWidget *_t = static_cast<PlotWidget *>(_o);
        switch (_id) {
        case 0: _t->closeMe((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 1: _t->setMeMinimized((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2]))); break;
        case 2: _t->selectionChanged(); break;
        case 3: _t->mousePress(); break;
        case 4: _t->mouseWheel(); break;
        case 5: _t->graphClicked((*reinterpret_cast< QCPAbstractPlottable*(*)>(_a[1]))); break;
        case 6: _t->on_closeButton_clicked(); break;
        case 7: _t->on_minimizeButton_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 8: _t->on_pauseButton_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (PlotWidget::*_t)(QString );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&PlotWidget::closeMe)) {
                *result = 0;
            }
        }
        {
            typedef void (PlotWidget::*_t)(QString , bool );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&PlotWidget::setMeMinimized)) {
                *result = 1;
            }
        }
    }
}

const QMetaObject PlotWidget::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_PlotWidget.data,
      qt_meta_data_PlotWidget,  qt_static_metacall, 0, 0}
};


const QMetaObject *PlotWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *PlotWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_PlotWidget.stringdata))
        return static_cast<void*>(const_cast< PlotWidget*>(this));
    return QWidget::qt_metacast(_clname);
}

int PlotWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
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

// SIGNAL 0
void PlotWidget::closeMe(QString _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void PlotWidget::setMeMinimized(QString _t1, bool _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_END_MOC_NAMESPACE
