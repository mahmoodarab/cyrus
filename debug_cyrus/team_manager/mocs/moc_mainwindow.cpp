/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.3.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../cyrus/TeamManager/mainwindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.3.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_MainWindow_t {
    QByteArrayData data[23];
    char stringdata[457];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MainWindow_t, stringdata) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MainWindow_t qt_meta_stringdata_MainWindow = {
    {
QT_MOC_LITERAL(0, 0, 10),
QT_MOC_LITERAL(1, 11, 26),
QT_MOC_LITERAL(2, 38, 0),
QT_MOC_LITERAL(3, 39, 25),
QT_MOC_LITERAL(4, 65, 8),
QT_MOC_LITERAL(5, 74, 25),
QT_MOC_LITERAL(6, 100, 21),
QT_MOC_LITERAL(7, 122, 15),
QT_MOC_LITERAL(8, 138, 7),
QT_MOC_LITERAL(9, 146, 4),
QT_MOC_LITERAL(10, 151, 23),
QT_MOC_LITERAL(11, 175, 11),
QT_MOC_LITERAL(12, 187, 28),
QT_MOC_LITERAL(13, 216, 4),
QT_MOC_LITERAL(14, 221, 28),
QT_MOC_LITERAL(15, 250, 20),
QT_MOC_LITERAL(16, 271, 26),
QT_MOC_LITERAL(17, 298, 33),
QT_MOC_LITERAL(18, 332, 5),
QT_MOC_LITERAL(19, 338, 21),
QT_MOC_LITERAL(20, 360, 28),
QT_MOC_LITERAL(21, 389, 32),
QT_MOC_LITERAL(22, 422, 34)
    },
    "MainWindow\0handleChoosingVisionSource\0"
    "\0setEnabledLogControlPanel\0enabled_\0"
    "setEnabledShowItemButtons\0"
    "handleChangingMainTab\0setCurrentTabTo\0"
    "MainTab\0tab_\0handlePinMainToolBarBtn\0"
    "quitProgram\0on_actionIdle_Server_toggled\0"
    "arg1\0on_actionIdle_Vision_toggled\0"
    "on_logPlayTB_clicked\0on_logFilebrowseTB_clicked\0"
    "on_logSeekPlayingHSl_valueChanged\0"
    "value\0on_logPauseTB_clicked\0"
    "on_logFileNameLE_textChanged\0"
    "on_chooseTeamColorBlueRB_clicked\0"
    "on_chooseTeamColorYellowRB_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MainWindow[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      16,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   94,    2, 0x08 /* Private */,
       3,    1,   95,    2, 0x08 /* Private */,
       5,    1,   98,    2, 0x08 /* Private */,
       6,    0,  101,    2, 0x08 /* Private */,
       7,    1,  102,    2, 0x08 /* Private */,
      10,    0,  105,    2, 0x08 /* Private */,
      11,    0,  106,    2, 0x08 /* Private */,
      12,    1,  107,    2, 0x08 /* Private */,
      14,    1,  110,    2, 0x08 /* Private */,
      15,    0,  113,    2, 0x08 /* Private */,
      16,    0,  114,    2, 0x08 /* Private */,
      17,    1,  115,    2, 0x08 /* Private */,
      19,    0,  118,    2, 0x08 /* Private */,
      20,    1,  119,    2, 0x08 /* Private */,
      21,    0,  122,    2, 0x08 /* Private */,
      22,    0,  123,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,    4,
    QMetaType::Void, QMetaType::Bool,    4,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 8,    9,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,   13,
    QMetaType::Void, QMetaType::Bool,   13,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,   18,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,   13,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        MainWindow *_t = static_cast<MainWindow *>(_o);
        switch (_id) {
        case 0: _t->handleChoosingVisionSource(); break;
        case 1: _t->setEnabledLogControlPanel((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 2: _t->setEnabledShowItemButtons((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 3: _t->handleChangingMainTab(); break;
        case 4: _t->setCurrentTabTo((*reinterpret_cast< MainTab(*)>(_a[1]))); break;
        case 5: _t->handlePinMainToolBarBtn(); break;
        case 6: _t->quitProgram(); break;
        case 7: _t->on_actionIdle_Server_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 8: _t->on_actionIdle_Vision_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 9: _t->on_logPlayTB_clicked(); break;
        case 10: _t->on_logFilebrowseTB_clicked(); break;
        case 11: _t->on_logSeekPlayingHSl_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 12: _t->on_logPauseTB_clicked(); break;
        case 13: _t->on_logFileNameLE_textChanged((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 14: _t->on_chooseTeamColorBlueRB_clicked(); break;
        case 15: _t->on_chooseTeamColorYellowRB_clicked(); break;
        default: ;
        }
    }
}

const QMetaObject MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_MainWindow.data,
      qt_meta_data_MainWindow,  qt_static_metacall, 0, 0}
};


const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow.stringdata))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 16)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 16;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 16)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 16;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
