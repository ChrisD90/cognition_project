/********************************************************************************
** Form generated from reading UI file 'main_window.ui'
**
** Created: Mon Sep 8 10:54:02 2014
**      by: Qt User Interface Compiler version 4.8.4
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAIN_WINDOW_H
#define UI_MAIN_WINDOW_H

#include <QtCore/QLocale>
#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QComboBox>
#include <QtGui/QFrame>
#include <QtGui/QGridLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QListView>
#include <QtGui/QMainWindow>
#include <QtGui/QPushButton>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindowDesign
{
public:
    QAction *action_Quit;
    QAction *action_Preferences;
    QAction *actionAbout;
    QAction *actionAbout_Qt;
    QWidget *centralwidget;
    QHBoxLayout *hboxLayout;
    QGroupBox *groupBox;
    QGridLayout *gridLayout_2;
    QListView *list_view_info;
    QGridLayout *grid_layout_action;
    QPushButton *button_receive_stop;
    QPushButton *button_receive_start;
    QPushButton *pushButton_3;
    QPushButton *pushButton_4;
    QPushButton *pushButton;
    QPushButton *pushButton_2;
    QGridLayout *grid_layout_frames;
    QComboBox *comboBox_11;
    QComboBox *comboBox_12;
    QLineEdit *lineEdit_5;
    QLabel *label;
    QLineEdit *lineEdit_6;
    QLineEdit *lineEdit_4;
    QLabel *label_2;
    QLineEdit *lineEdit_2;
    QLineEdit *lineEdit;
    QLabel *label_3;
    QLabel *label_6;
    QLabel *label_5;
    QLabel *label_4;
    QLabel *label_7;
    QComboBox *comboBox_13;
    QLabel *label_9;
    QLabel *label_8;
    QLabel *label_10;
    QLabel *label_12;
    QLabel *label_11;
    QComboBox *comboBox;
    QLabel *label_13;
    QComboBox *comboBox_2;
    QComboBox *comboBox_3;
    QComboBox *comboBox_4;
    QComboBox *comboBox_5;
    QComboBox *comboBox_6;
    QComboBox *comboBox_7;
    QComboBox *comboBox_8;
    QComboBox *comboBox_9;
    QComboBox *comboBox_10;
    QLineEdit *lineEdit_3;
    QLineEdit *lineEdit_7;
    QLineEdit *lineEdit_8;
    QLineEdit *lineEdit_9;
    QLineEdit *lineEdit_10;
    QLineEdit *lineEdit_11;
    QLineEdit *lineEdit_12;
    QLineEdit *lineEdit_13;
    QFrame *line;
    QGridLayout *grid_layout_setup;
    QPushButton *button_setup_edit;
    QPushButton *button_setup_done;
    QLabel *label_setup_0;
    QGridLayout *grid_layout_host_master;
    QLineEdit *line_edit_master_uri;
    QLineEdit *line_edit_host_ip;
    QLabel *label_master_uri;
    QLabel *label_host_ip;
    QPushButton *button_host_info;
    QPushButton *button_master_info;
    QLabel *label_setup_1;
    QFrame *line_2;

    void setupUi(QMainWindow *MainWindowDesign)
    {
        if (MainWindowDesign->objectName().isEmpty())
            MainWindowDesign->setObjectName(QString::fromUtf8("MainWindowDesign"));
        MainWindowDesign->resize(450, 700);
        MainWindowDesign->setMinimumSize(QSize(450, 700));
        MainWindowDesign->setMaximumSize(QSize(450, 700));
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/images/icon.png"), QSize(), QIcon::Normal, QIcon::Off);
        MainWindowDesign->setWindowIcon(icon);
        MainWindowDesign->setLocale(QLocale(QLocale::German, QLocale::Germany));
        action_Quit = new QAction(MainWindowDesign);
        action_Quit->setObjectName(QString::fromUtf8("action_Quit"));
        action_Quit->setShortcutContext(Qt::ApplicationShortcut);
        action_Preferences = new QAction(MainWindowDesign);
        action_Preferences->setObjectName(QString::fromUtf8("action_Preferences"));
        action_Preferences->setEnabled(true);
        actionAbout = new QAction(MainWindowDesign);
        actionAbout->setObjectName(QString::fromUtf8("actionAbout"));
        actionAbout_Qt = new QAction(MainWindowDesign);
        actionAbout_Qt->setObjectName(QString::fromUtf8("actionAbout_Qt"));
        centralwidget = new QWidget(MainWindowDesign);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        hboxLayout = new QHBoxLayout(centralwidget);
        hboxLayout->setSpacing(0);
        hboxLayout->setObjectName(QString::fromUtf8("hboxLayout"));
        hboxLayout->setContentsMargins(0, 0, 9, 6);
        groupBox = new QGroupBox(centralwidget);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(groupBox->sizePolicy().hasHeightForWidth());
        groupBox->setSizePolicy(sizePolicy);
        groupBox->setAlignment(Qt::AlignCenter);
        gridLayout_2 = new QGridLayout(groupBox);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        gridLayout_2->setHorizontalSpacing(0);
        gridLayout_2->setVerticalSpacing(3);
        gridLayout_2->setContentsMargins(0, 0, 0, 0);
        list_view_info = new QListView(groupBox);
        list_view_info->setObjectName(QString::fromUtf8("list_view_info"));
        list_view_info->setMinimumSize(QSize(0, 150));

        gridLayout_2->addWidget(list_view_info, 12, 0, 1, 1);

        grid_layout_action = new QGridLayout();
        grid_layout_action->setSpacing(3);
        grid_layout_action->setObjectName(QString::fromUtf8("grid_layout_action"));
        grid_layout_action->setContentsMargins(3, 3, 3, 3);
        button_receive_stop = new QPushButton(groupBox);
        button_receive_stop->setObjectName(QString::fromUtf8("button_receive_stop"));

        grid_layout_action->addWidget(button_receive_stop, 2, 0, 1, 1);

        button_receive_start = new QPushButton(groupBox);
        button_receive_start->setObjectName(QString::fromUtf8("button_receive_start"));

        grid_layout_action->addWidget(button_receive_start, 1, 0, 1, 1);

        pushButton_3 = new QPushButton(groupBox);
        pushButton_3->setObjectName(QString::fromUtf8("pushButton_3"));

        grid_layout_action->addWidget(pushButton_3, 1, 2, 1, 1);

        pushButton_4 = new QPushButton(groupBox);
        pushButton_4->setObjectName(QString::fromUtf8("pushButton_4"));

        grid_layout_action->addWidget(pushButton_4, 2, 2, 1, 1);

        pushButton = new QPushButton(groupBox);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));

        grid_layout_action->addWidget(pushButton, 2, 1, 1, 1);

        pushButton_2 = new QPushButton(groupBox);
        pushButton_2->setObjectName(QString::fromUtf8("pushButton_2"));

        grid_layout_action->addWidget(pushButton_2, 1, 1, 1, 1);


        gridLayout_2->addLayout(grid_layout_action, 8, 0, 1, 1);

        grid_layout_frames = new QGridLayout();
        grid_layout_frames->setSpacing(3);
        grid_layout_frames->setObjectName(QString::fromUtf8("grid_layout_frames"));
        grid_layout_frames->setContentsMargins(3, 3, 3, 3);
        comboBox_11 = new QComboBox(groupBox);
        comboBox_11->setObjectName(QString::fromUtf8("comboBox_11"));

        grid_layout_frames->addWidget(comboBox_11, 12, 1, 1, 1);

        comboBox_12 = new QComboBox(groupBox);
        comboBox_12->setObjectName(QString::fromUtf8("comboBox_12"));

        grid_layout_frames->addWidget(comboBox_12, 1, 1, 1, 1);

        lineEdit_5 = new QLineEdit(groupBox);
        lineEdit_5->setObjectName(QString::fromUtf8("lineEdit_5"));

        grid_layout_frames->addWidget(lineEdit_5, 4, 2, 1, 1);

        label = new QLabel(groupBox);
        label->setObjectName(QString::fromUtf8("label"));

        grid_layout_frames->addWidget(label, 1, 0, 1, 1);

        lineEdit_6 = new QLineEdit(groupBox);
        lineEdit_6->setObjectName(QString::fromUtf8("lineEdit_6"));

        grid_layout_frames->addWidget(lineEdit_6, 5, 2, 1, 1);

        lineEdit_4 = new QLineEdit(groupBox);
        lineEdit_4->setObjectName(QString::fromUtf8("lineEdit_4"));

        grid_layout_frames->addWidget(lineEdit_4, 3, 2, 1, 1);

        label_2 = new QLabel(groupBox);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        grid_layout_frames->addWidget(label_2, 0, 0, 1, 1);

        lineEdit_2 = new QLineEdit(groupBox);
        lineEdit_2->setObjectName(QString::fromUtf8("lineEdit_2"));

        grid_layout_frames->addWidget(lineEdit_2, 1, 2, 1, 1);

        lineEdit = new QLineEdit(groupBox);
        lineEdit->setObjectName(QString::fromUtf8("lineEdit"));

        grid_layout_frames->addWidget(lineEdit, 0, 2, 1, 1);

        label_3 = new QLabel(groupBox);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        grid_layout_frames->addWidget(label_3, 2, 0, 1, 1);

        label_6 = new QLabel(groupBox);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        grid_layout_frames->addWidget(label_6, 5, 0, 1, 1);

        label_5 = new QLabel(groupBox);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        grid_layout_frames->addWidget(label_5, 4, 0, 1, 1);

        label_4 = new QLabel(groupBox);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        grid_layout_frames->addWidget(label_4, 3, 0, 1, 1);

        label_7 = new QLabel(groupBox);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        grid_layout_frames->addWidget(label_7, 6, 0, 1, 1);

        comboBox_13 = new QComboBox(groupBox);
        comboBox_13->setObjectName(QString::fromUtf8("comboBox_13"));

        grid_layout_frames->addWidget(comboBox_13, 0, 1, 1, 1);

        label_9 = new QLabel(groupBox);
        label_9->setObjectName(QString::fromUtf8("label_9"));

        grid_layout_frames->addWidget(label_9, 8, 0, 1, 1);

        label_8 = new QLabel(groupBox);
        label_8->setObjectName(QString::fromUtf8("label_8"));

        grid_layout_frames->addWidget(label_8, 7, 0, 1, 1);

        label_10 = new QLabel(groupBox);
        label_10->setObjectName(QString::fromUtf8("label_10"));

        grid_layout_frames->addWidget(label_10, 9, 0, 1, 1);

        label_12 = new QLabel(groupBox);
        label_12->setObjectName(QString::fromUtf8("label_12"));

        grid_layout_frames->addWidget(label_12, 11, 0, 1, 1);

        label_11 = new QLabel(groupBox);
        label_11->setObjectName(QString::fromUtf8("label_11"));

        grid_layout_frames->addWidget(label_11, 10, 0, 1, 1);

        comboBox = new QComboBox(groupBox);
        comboBox->setObjectName(QString::fromUtf8("comboBox"));

        grid_layout_frames->addWidget(comboBox, 2, 1, 1, 1);

        label_13 = new QLabel(groupBox);
        label_13->setObjectName(QString::fromUtf8("label_13"));

        grid_layout_frames->addWidget(label_13, 12, 0, 1, 1);

        comboBox_2 = new QComboBox(groupBox);
        comboBox_2->setObjectName(QString::fromUtf8("comboBox_2"));

        grid_layout_frames->addWidget(comboBox_2, 3, 1, 1, 1);

        comboBox_3 = new QComboBox(groupBox);
        comboBox_3->setObjectName(QString::fromUtf8("comboBox_3"));

        grid_layout_frames->addWidget(comboBox_3, 4, 1, 1, 1);

        comboBox_4 = new QComboBox(groupBox);
        comboBox_4->setObjectName(QString::fromUtf8("comboBox_4"));

        grid_layout_frames->addWidget(comboBox_4, 5, 1, 1, 1);

        comboBox_5 = new QComboBox(groupBox);
        comboBox_5->setObjectName(QString::fromUtf8("comboBox_5"));

        grid_layout_frames->addWidget(comboBox_5, 6, 1, 1, 1);

        comboBox_6 = new QComboBox(groupBox);
        comboBox_6->setObjectName(QString::fromUtf8("comboBox_6"));

        grid_layout_frames->addWidget(comboBox_6, 7, 1, 1, 1);

        comboBox_7 = new QComboBox(groupBox);
        comboBox_7->setObjectName(QString::fromUtf8("comboBox_7"));

        grid_layout_frames->addWidget(comboBox_7, 8, 1, 1, 1);

        comboBox_8 = new QComboBox(groupBox);
        comboBox_8->setObjectName(QString::fromUtf8("comboBox_8"));

        grid_layout_frames->addWidget(comboBox_8, 9, 1, 1, 1);

        comboBox_9 = new QComboBox(groupBox);
        comboBox_9->setObjectName(QString::fromUtf8("comboBox_9"));

        grid_layout_frames->addWidget(comboBox_9, 10, 1, 1, 1);

        comboBox_10 = new QComboBox(groupBox);
        comboBox_10->setObjectName(QString::fromUtf8("comboBox_10"));

        grid_layout_frames->addWidget(comboBox_10, 11, 1, 1, 1);

        lineEdit_3 = new QLineEdit(groupBox);
        lineEdit_3->setObjectName(QString::fromUtf8("lineEdit_3"));

        grid_layout_frames->addWidget(lineEdit_3, 2, 2, 1, 1);

        lineEdit_7 = new QLineEdit(groupBox);
        lineEdit_7->setObjectName(QString::fromUtf8("lineEdit_7"));

        grid_layout_frames->addWidget(lineEdit_7, 6, 2, 1, 1);

        lineEdit_8 = new QLineEdit(groupBox);
        lineEdit_8->setObjectName(QString::fromUtf8("lineEdit_8"));

        grid_layout_frames->addWidget(lineEdit_8, 7, 2, 1, 1);

        lineEdit_9 = new QLineEdit(groupBox);
        lineEdit_9->setObjectName(QString::fromUtf8("lineEdit_9"));

        grid_layout_frames->addWidget(lineEdit_9, 8, 2, 1, 1);

        lineEdit_10 = new QLineEdit(groupBox);
        lineEdit_10->setObjectName(QString::fromUtf8("lineEdit_10"));

        grid_layout_frames->addWidget(lineEdit_10, 9, 2, 1, 1);

        lineEdit_11 = new QLineEdit(groupBox);
        lineEdit_11->setObjectName(QString::fromUtf8("lineEdit_11"));

        grid_layout_frames->addWidget(lineEdit_11, 10, 2, 1, 1);

        lineEdit_12 = new QLineEdit(groupBox);
        lineEdit_12->setObjectName(QString::fromUtf8("lineEdit_12"));

        grid_layout_frames->addWidget(lineEdit_12, 11, 2, 1, 1);

        lineEdit_13 = new QLineEdit(groupBox);
        lineEdit_13->setObjectName(QString::fromUtf8("lineEdit_13"));

        grid_layout_frames->addWidget(lineEdit_13, 12, 2, 1, 1);


        gridLayout_2->addLayout(grid_layout_frames, 13, 0, 1, 1);

        line = new QFrame(groupBox);
        line->setObjectName(QString::fromUtf8("line"));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);

        gridLayout_2->addWidget(line, 3, 0, 1, 1);

        grid_layout_setup = new QGridLayout();
        grid_layout_setup->setObjectName(QString::fromUtf8("grid_layout_setup"));
        grid_layout_setup->setContentsMargins(-1, 0, -1, -1);
        button_setup_edit = new QPushButton(groupBox);
        button_setup_edit->setObjectName(QString::fromUtf8("button_setup_edit"));

        grid_layout_setup->addWidget(button_setup_edit, 0, 1, 1, 1);

        button_setup_done = new QPushButton(groupBox);
        button_setup_done->setObjectName(QString::fromUtf8("button_setup_done"));

        grid_layout_setup->addWidget(button_setup_done, 0, 0, 1, 1);


        gridLayout_2->addLayout(grid_layout_setup, 4, 0, 1, 1);

        label_setup_0 = new QLabel(groupBox);
        label_setup_0->setObjectName(QString::fromUtf8("label_setup_0"));

        gridLayout_2->addWidget(label_setup_0, 0, 0, 1, 1);

        grid_layout_host_master = new QGridLayout();
        grid_layout_host_master->setSpacing(3);
        grid_layout_host_master->setObjectName(QString::fromUtf8("grid_layout_host_master"));
        grid_layout_host_master->setContentsMargins(3, 3, 3, 3);
        line_edit_master_uri = new QLineEdit(groupBox);
        line_edit_master_uri->setObjectName(QString::fromUtf8("line_edit_master_uri"));

        grid_layout_host_master->addWidget(line_edit_master_uri, 1, 2, 1, 1);

        line_edit_host_ip = new QLineEdit(groupBox);
        line_edit_host_ip->setObjectName(QString::fromUtf8("line_edit_host_ip"));

        grid_layout_host_master->addWidget(line_edit_host_ip, 0, 2, 1, 1);

        label_master_uri = new QLabel(groupBox);
        label_master_uri->setObjectName(QString::fromUtf8("label_master_uri"));

        grid_layout_host_master->addWidget(label_master_uri, 1, 0, 1, 1);

        label_host_ip = new QLabel(groupBox);
        label_host_ip->setObjectName(QString::fromUtf8("label_host_ip"));

        grid_layout_host_master->addWidget(label_host_ip, 0, 0, 1, 1);

        button_host_info = new QPushButton(groupBox);
        button_host_info->setObjectName(QString::fromUtf8("button_host_info"));
        button_host_info->setMinimumSize(QSize(30, 0));
        button_host_info->setMaximumSize(QSize(30, 16777215));

        grid_layout_host_master->addWidget(button_host_info, 0, 1, 1, 1);

        button_master_info = new QPushButton(groupBox);
        button_master_info->setObjectName(QString::fromUtf8("button_master_info"));
        button_master_info->setMinimumSize(QSize(30, 0));
        button_master_info->setMaximumSize(QSize(30, 16777215));

        grid_layout_host_master->addWidget(button_master_info, 1, 1, 1, 1);


        gridLayout_2->addLayout(grid_layout_host_master, 2, 0, 1, 1);

        label_setup_1 = new QLabel(groupBox);
        label_setup_1->setObjectName(QString::fromUtf8("label_setup_1"));

        gridLayout_2->addWidget(label_setup_1, 1, 0, 1, 1);

        line_2 = new QFrame(groupBox);
        line_2->setObjectName(QString::fromUtf8("line_2"));
        line_2->setFrameShape(QFrame::HLine);
        line_2->setFrameShadow(QFrame::Sunken);

        gridLayout_2->addWidget(line_2, 5, 0, 1, 1);


        hboxLayout->addWidget(groupBox);

        MainWindowDesign->setCentralWidget(centralwidget);

        retranslateUi(MainWindowDesign);
        QObject::connect(action_Quit, SIGNAL(triggered()), MainWindowDesign, SLOT(close()));

        QMetaObject::connectSlotsByName(MainWindowDesign);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindowDesign)
    {
        MainWindowDesign->setWindowTitle(QApplication::translate("MainWindowDesign", "cognition_project", 0, QApplication::UnicodeUTF8));
        action_Quit->setText(QApplication::translate("MainWindowDesign", "&Quit", 0, QApplication::UnicodeUTF8));
        action_Quit->setShortcut(QApplication::translate("MainWindowDesign", "Ctrl+Q", 0, QApplication::UnicodeUTF8));
        action_Preferences->setText(QApplication::translate("MainWindowDesign", "&Preferences", 0, QApplication::UnicodeUTF8));
        actionAbout->setText(QApplication::translate("MainWindowDesign", "&About", 0, QApplication::UnicodeUTF8));
        actionAbout_Qt->setText(QApplication::translate("MainWindowDesign", "About &Qt", 0, QApplication::UnicodeUTF8));
        button_receive_stop->setText(QApplication::translate("MainWindowDesign", "Stop receiving", 0, QApplication::UnicodeUTF8));
        button_receive_start->setText(QApplication::translate("MainWindowDesign", "Start receiving", 0, QApplication::UnicodeUTF8));
        pushButton_3->setText(QApplication::translate("MainWindowDesign", "PushButton", 0, QApplication::UnicodeUTF8));
        pushButton_4->setText(QApplication::translate("MainWindowDesign", "PushButton", 0, QApplication::UnicodeUTF8));
        pushButton->setText(QApplication::translate("MainWindowDesign", "PushButton", 0, QApplication::UnicodeUTF8));
        pushButton_2->setText(QApplication::translate("MainWindowDesign", "PushButton", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("MainWindowDesign", "TextLabel", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("MainWindowDesign", "TextLabel", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("MainWindowDesign", "TextLabel", 0, QApplication::UnicodeUTF8));
        label_6->setText(QApplication::translate("MainWindowDesign", "TextLabel", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("MainWindowDesign", "TextLabel", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("MainWindowDesign", "TextLabel", 0, QApplication::UnicodeUTF8));
        label_7->setText(QApplication::translate("MainWindowDesign", "TextLabel", 0, QApplication::UnicodeUTF8));
        label_9->setText(QApplication::translate("MainWindowDesign", "TextLabel", 0, QApplication::UnicodeUTF8));
        label_8->setText(QApplication::translate("MainWindowDesign", "TextLabel", 0, QApplication::UnicodeUTF8));
        label_10->setText(QApplication::translate("MainWindowDesign", "TextLabel", 0, QApplication::UnicodeUTF8));
        label_12->setText(QApplication::translate("MainWindowDesign", "TextLabel", 0, QApplication::UnicodeUTF8));
        label_11->setText(QApplication::translate("MainWindowDesign", "TextLabel", 0, QApplication::UnicodeUTF8));
        label_13->setText(QApplication::translate("MainWindowDesign", "TextLabel", 0, QApplication::UnicodeUTF8));
        button_setup_edit->setText(QApplication::translate("MainWindowDesign", "Edit setup", 0, QApplication::UnicodeUTF8));
        button_setup_done->setText(QApplication::translate("MainWindowDesign", "Setup done", 0, QApplication::UnicodeUTF8));
        label_setup_0->setText(QApplication::translate("MainWindowDesign", "1. Connect to WLAN 'mocap'", 0, QApplication::UnicodeUTF8));
        label_master_uri->setText(QApplication::translate("MainWindowDesign", "Master-URI", 0, QApplication::UnicodeUTF8));
        label_host_ip->setText(QApplication::translate("MainWindowDesign", "Host-IP", 0, QApplication::UnicodeUTF8));
        button_host_info->setText(QApplication::translate("MainWindowDesign", "?", 0, QApplication::UnicodeUTF8));
        button_master_info->setText(QApplication::translate("MainWindowDesign", "?", 0, QApplication::UnicodeUTF8));
        label_setup_1->setText(QApplication::translate("MainWindowDesign", "2. Start ROS Master with 'roscore' console command", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindowDesign: public Ui_MainWindowDesign {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAIN_WINDOW_H
