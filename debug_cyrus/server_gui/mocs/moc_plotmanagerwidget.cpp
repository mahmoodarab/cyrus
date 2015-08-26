/****************************************************************************
** Meta object code from reading C++ file 'plotmanagerwidget.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.3.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../cyrus2014/TeamManager/plot-manager/plotmanagerwidget.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'plotmanagerwidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.3.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_PlotManagerWidget_t {
    QByteArrayData data[15];
    char stringdata[167];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_PlotManagerWidget_t, stringdata) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_PlotManagerWidget_t qt_meta_stringdata_PlotManagerWidget = {
    {
QT_MOC_LITERAL(0, 0, 17),
QT_MOC_LITERAL(1, 18, 4),
QT_MOC_LITERAL(2, 23, 0),
QT_MOC_LITERAL(3, 24, 5),
QT_MOC_LITERAL(4, 30, 9),
QT_MOC_LITERAL(5, 40, 8),
QT_MOC_LITERAL(6, 49, 14),
QT_MOC_LITERAL(7, 64, 14),
QT_MOC_LITERAL(8, 79, 6),
QT_MOC_LITERAL(9, 86, 20),
QT_MOC_LITERAL(10, 107, 18),
QT_MOC_LITERAL(11, 126, 8),
QT_MOC_LITERAL(12, 135, 4),
QT_MOC_LITERAL(13, 140, 16),
QT_MOC_LITERAL(14, 157, 9)
    },
    "PlotManagerWidget\0plot\0\0value\0plot_name\0"
    "category\0newPlotMessage\0Plotter_Packet\0"
    "packet\0joinMulticastNetwork\0"
    "processPendingData\0quitPlot\0name\0"
    "setPlotMinimized\0minimized"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_PlotManagerWidget[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    3,   44,    2, 0x0a /* Public */,
       6,    1,   51,    2, 0x0a /* Public */,
       9,    0,   54,    2, 0x0a /* Public */,
      10,    0,   55,    2, 0x08 /* Private */,
      11,    1,   56,    2, 0x08 /* Private */,
      13,    2,   59,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void, QMetaType::Double, QMetaType::QString, QMetaType::QString,    3,    4,    5,
    QMetaType::Void, 0x80000000 | 7,    8,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,   12,
    QMetaType::Void, QMetaType::QString, QMetaType::Bool,   12,   14,

       0        // eod
};

void PlotManagerWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        PlotManagerWidget *_t = static_cast<PlotManagerWidget *>(_o);
        switch (_id) {
        case 0: _t->plot((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2])),(*reinterpret_cast< QString(*)>(_a[3]))); break;
        case 1: _t->newPlotMessage((*reinterpret_cast< Plotter_Packet(*)>(_a[1]))); break;
        case 2: _t->joinMulticastNetwork(); break;
        case 3: _t->processPendingData(); break;
        case 4: _t->quitPlot((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 5: _t->setPlotMinimized((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2]))); break;
        default: ;
        }
    }
}

const QMetaObject PlotManagerWidget::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_PlotManagerWidget.data,
      qt_meta_data_PlotManagerWidget,  qt_static_metacall, 0, 0}
};


const QMetaObject *PlotManagerWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *PlotManagerWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_PlotManagerWidget.stringdata))
        return static_cast<void*>(const_cast< PlotManagerWidget*>(this));
    return QWidget::qt_metacast(_clname);
}

int PlotManagerWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 6)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 6;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
