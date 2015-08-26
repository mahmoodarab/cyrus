/********************************************************************************
** Form generated from reading UI file 'designdialog.ui'
**
** Created by: Qt User Interface Compiler version 5.3.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_DESIGNDIALOG_H
#define UI_DESIGNDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QVBoxLayout>

QT_BEGIN_NAMESPACE

class Ui_DesignDialog
{
public:
    QHBoxLayout *horizontalLayout;
    QVBoxLayout *verticalLayout;
    QVBoxLayout *verticalLayout_3;
    QLabel *label;
    QLineEdit *txtW;
    QLabel *label_2;
    QLineEdit *txtH;
    QSpacerItem *verticalSpacer;
    QVBoxLayout *verticalLayout_4;
    QPushButton *openBtn;
    QLabel *label_3;
    QSpinBox *intervalSpinBox;
    QPushButton *saveButton;
    QPushButton *saveAsBtn;
    QVBoxLayout *verticalLayout_2;
    QPushButton *resetBtn;
    QPushButton *quitBtn;
    QVBoxLayout *graphLayout;

    void setupUi(QDialog *DesignDialog)
    {
        if (DesignDialog->objectName().isEmpty())
            DesignDialog->setObjectName(QStringLiteral("DesignDialog"));
        DesignDialog->setWindowModality(Qt::NonModal);
        DesignDialog->resize(121, 600);
        DesignDialog->setModal(false);
        horizontalLayout = new QHBoxLayout(DesignDialog);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        label = new QLabel(DesignDialog);
        label->setObjectName(QStringLiteral("label"));

        verticalLayout_3->addWidget(label);

        txtW = new QLineEdit(DesignDialog);
        txtW->setObjectName(QStringLiteral("txtW"));
        txtW->setInputMethodHints(Qt::ImhDigitsOnly);

        verticalLayout_3->addWidget(txtW);

        label_2 = new QLabel(DesignDialog);
        label_2->setObjectName(QStringLiteral("label_2"));

        verticalLayout_3->addWidget(label_2);

        txtH = new QLineEdit(DesignDialog);
        txtH->setObjectName(QStringLiteral("txtH"));
        txtH->setInputMethodHints(Qt::ImhDigitsOnly);

        verticalLayout_3->addWidget(txtH);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_3->addItem(verticalSpacer);


        verticalLayout->addLayout(verticalLayout_3);

        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setObjectName(QStringLiteral("verticalLayout_4"));
        openBtn = new QPushButton(DesignDialog);
        openBtn->setObjectName(QStringLiteral("openBtn"));

        verticalLayout_4->addWidget(openBtn);

        label_3 = new QLabel(DesignDialog);
        label_3->setObjectName(QStringLiteral("label_3"));

        verticalLayout_4->addWidget(label_3);

        intervalSpinBox = new QSpinBox(DesignDialog);
        intervalSpinBox->setObjectName(QStringLiteral("intervalSpinBox"));
        intervalSpinBox->setMinimum(1);
        intervalSpinBox->setMaximum(10000);

        verticalLayout_4->addWidget(intervalSpinBox);

        saveButton = new QPushButton(DesignDialog);
        saveButton->setObjectName(QStringLiteral("saveButton"));

        verticalLayout_4->addWidget(saveButton);

        saveAsBtn = new QPushButton(DesignDialog);
        saveAsBtn->setObjectName(QStringLiteral("saveAsBtn"));

        verticalLayout_4->addWidget(saveAsBtn);


        verticalLayout->addLayout(verticalLayout_4);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        resetBtn = new QPushButton(DesignDialog);
        resetBtn->setObjectName(QStringLiteral("resetBtn"));

        verticalLayout_2->addWidget(resetBtn);

        quitBtn = new QPushButton(DesignDialog);
        quitBtn->setObjectName(QStringLiteral("quitBtn"));

        verticalLayout_2->addWidget(quitBtn);


        verticalLayout->addLayout(verticalLayout_2);


        horizontalLayout->addLayout(verticalLayout);

        graphLayout = new QVBoxLayout();
        graphLayout->setObjectName(QStringLiteral("graphLayout"));
        graphLayout->setContentsMargins(-1, -1, 6, -1);

        horizontalLayout->addLayout(graphLayout);


        retranslateUi(DesignDialog);

        QMetaObject::connectSlotsByName(DesignDialog);
    } // setupUi

    void retranslateUi(QDialog *DesignDialog)
    {
        DesignDialog->setWindowTitle(QApplication::translate("DesignDialog", "Dialog", 0));
        label->setText(QApplication::translate("DesignDialog", "X", 0));
        txtW->setText(QApplication::translate("DesignDialog", "600", 0));
        label_2->setText(QApplication::translate("DesignDialog", "Y", 0));
        txtH->setText(QApplication::translate("DesignDialog", "255", 0));
        openBtn->setText(QApplication::translate("DesignDialog", "Open", 0));
        label_3->setText(QApplication::translate("DesignDialog", "Save interval:", 0));
        saveButton->setText(QApplication::translate("DesignDialog", "Save", 0));
        saveAsBtn->setText(QApplication::translate("DesignDialog", "Save As ...", 0));
        resetBtn->setText(QApplication::translate("DesignDialog", "Reset", 0));
        quitBtn->setText(QApplication::translate("DesignDialog", "Quit", 0));
    } // retranslateUi

};

namespace Ui {
    class DesignDialog: public Ui_DesignDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_DESIGNDIALOG_H
