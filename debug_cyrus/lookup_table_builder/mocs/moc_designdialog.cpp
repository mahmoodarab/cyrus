/****************************************************************************
** Meta object code from reading C++ file 'designdialog.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.3.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../cyrus2014/LookupTableBuilder/Dialogs/designdialog.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'designdialog.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.3.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_DesignDialog_t {
    QByteArrayData data[11];
    char stringdata[171];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_DesignDialog_t, stringdata) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_DesignDialog_t qt_meta_stringdata_DesignDialog = {
    {
QT_MOC_LITERAL(0, 0, 12),
QT_MOC_LITERAL(1, 13, 4),
QT_MOC_LITERAL(2, 18, 0),
QT_MOC_LITERAL(3, 19, 6),
QT_MOC_LITERAL(4, 26, 21),
QT_MOC_LITERAL(5, 48, 21),
QT_MOC_LITERAL(6, 70, 18),
QT_MOC_LITERAL(7, 89, 21),
QT_MOC_LITERAL(8, 111, 20),
QT_MOC_LITERAL(9, 132, 18),
QT_MOC_LITERAL(10, 151, 19)
    },
    "DesignDialog\0save\0\0reject\0"
    "on_txtH_returnPressed\0on_txtW_returnPressed\0"
    "on_openBtn_clicked\0on_saveButton_clicked\0"
    "on_saveAsBtn_clicked\0on_quitBtn_clicked\0"
    "on_resetBtn_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_DesignDialog[] = {

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
       1,    0,   59,    2, 0x08 /* Private */,
       3,    0,   60,    2, 0x08 /* Private */,
       4,    0,   61,    2, 0x08 /* Private */,
       5,    0,   62,    2, 0x08 /* Private */,
       6,    0,   63,    2, 0x08 /* Private */,
       7,    0,   64,    2, 0x08 /* Private */,
       8,    0,   65,    2, 0x08 /* Private */,
       9,    0,   66,    2, 0x08 /* Private */,
      10,    0,   67,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void DesignDialog::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        DesignDialog *_t = static_cast<DesignDialog *>(_o);
        switch (_id) {
        case 0: _t->save(); break;
        case 1: _t->reject(); break;
        case 2: _t->on_txtH_returnPressed(); break;
        case 3: _t->on_txtW_returnPressed(); break;
        case 4: _t->on_openBtn_clicked(); break;
        case 5: _t->on_saveButton_clicked(); break;
        case 6: _t->on_saveAsBtn_clicked(); break;
        case 7: _t->on_quitBtn_clicked(); break;
        case 8: _t->on_resetBtn_clicked(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObject DesignDialog::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_DesignDialog.data,
      qt_meta_data_DesignDialog,  qt_static_metacall, 0, 0}
};


const QMetaObject *DesignDialog::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *DesignDialog::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_DesignDialog.stringdata))
        return static_cast<void*>(const_cast< DesignDialog*>(this));
    return QDialog::qt_metacast(_clname);
}

int DesignDialog::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
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
