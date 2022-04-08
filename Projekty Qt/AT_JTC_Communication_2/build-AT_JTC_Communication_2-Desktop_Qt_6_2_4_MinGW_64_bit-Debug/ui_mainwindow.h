/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 6.2.4
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTableView>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QTabWidget *tabWidget;
    QWidget *tab;
    QWidget *gridLayoutWidget;
    QGridLayout *gridLayout_8;
    QTextEdit *Com_ReadTextEdit;
    QPushButton *Com_ReadTextEdit_2;
    QTextEdit *Com_DebugTextEdit;
    QPushButton *Com_ReadDebugTextedit;
    QWidget *layoutWidget;
    QGridLayout *gridLayout;
    QGridLayout *gridLayout_3;
    QPushButton *Com_ReadFrictionTableFloat;
    QPushButton *Com_SendCommanUsDefaultFrictionTableToJtc;
    QGridLayout *gridLayout_9;
    QPushButton *Com_SendFrictionPolynomialCoeffsToJtc;
    QPushButton *Com_SendFrictionTableToJtc;
    QPushButton *Com_SendCommanUsDefaultFrictionPolynomialCoeffsToJtc;
    QPushButton *Com_ReadFrictionPolynomialCoeffsFloat;
    QPushButton *Com_OpenButt;
    QGridLayout *gridLayout_2;
    QPushButton *Com_SendTrajectoryToJtc;
    QPushButton *Com_ReadTrajectoryInt16;
    QPushButton *Com_CloseButt;
    QComboBox *Com_NameComboBox;
    QGridLayout *gridLayout_6;
    QPushButton *Com_ReadArmModel;
    QPushButton *Com_SendArmModelToJtc;
    QPushButton *Com_SendCommandUseDefaultArmModelToJtc;
    QGridLayout *gridLayout_7;
    QPushButton *Com_SendCommandUseDefaultPidParamlToJtc;
    QPushButton *Com_ReadPidParam;
    QPushButton *Com_SendPidParamlToJtc;
    QPushButton *Com_OnOffSynchroTransmision;
    QVBoxLayout *verticalLayout;
    QRadioButton *pathWindowsRadioButton;
    QRadioButton *pathLinuxRadioButton;
    QWidget *tab_3;
    QTableView *Com_JointsParamTableView;
    QTableView *Com_JtcParamTableView;
    QWidget *gridLayoutWidget_2;
    QGridLayout *gridLayout_5;
    QPushButton *JTC_ClearErrors;
    QPushButton *Traj_TrajectoryExecute;
    QPushButton *JTC_ClearOccuredErrors;
    QPushButton *Traj_TrajectoryPause;
    QPushButton *Traj_TrajectoryStop;
    QPushButton *JTC_TeachingModeEnable;
    QPushButton *JTC_TeachingModeDisable;
    QWidget *Com;
    QTextEdit *Com_TrajTextEdit;
    QTableView *Com_TrajTableView;
    QWidget *Can;
    QTextEdit *Com_FrictionTableTextEdit;
    QTableView *Com_FrictionTableTableView;
    QWidget *tab_4;
    QTextEdit *Com_PidParametersTextEdit;
    QTableView *Com_PidParametersTableView;
    QWidget *tab_2;
    QTableView *Com_ArmModelLinksTableView;
    QTextEdit *Com_ArmModelTextEdit;
    QTableView *Com_ArmModelJointsTableView;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1600, 971);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        tabWidget = new QTabWidget(centralwidget);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tabWidget->setGeometry(QRect(0, 10, 16500, 911));
        QFont font;
        font.setPointSize(10);
        font.setBold(false);
        tabWidget->setFont(font);
        tabWidget->setIconSize(QSize(16, 16));
        tab = new QWidget();
        tab->setObjectName(QString::fromUtf8("tab"));
        gridLayoutWidget = new QWidget(tab);
        gridLayoutWidget->setObjectName(QString::fromUtf8("gridLayoutWidget"));
        gridLayoutWidget->setGeometry(QRect(10, 320, 1391, 501));
        gridLayout_8 = new QGridLayout(gridLayoutWidget);
        gridLayout_8->setObjectName(QString::fromUtf8("gridLayout_8"));
        gridLayout_8->setContentsMargins(0, 0, 0, 0);
        Com_ReadTextEdit = new QTextEdit(gridLayoutWidget);
        Com_ReadTextEdit->setObjectName(QString::fromUtf8("Com_ReadTextEdit"));

        gridLayout_8->addWidget(Com_ReadTextEdit, 1, 0, 1, 1);

        Com_ReadTextEdit_2 = new QPushButton(gridLayoutWidget);
        Com_ReadTextEdit_2->setObjectName(QString::fromUtf8("Com_ReadTextEdit_2"));
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(Com_ReadTextEdit_2->sizePolicy().hasHeightForWidth());
        Com_ReadTextEdit_2->setSizePolicy(sizePolicy);
        Com_ReadTextEdit_2->setFont(font);

        gridLayout_8->addWidget(Com_ReadTextEdit_2, 0, 0, 1, 1);

        Com_DebugTextEdit = new QTextEdit(gridLayoutWidget);
        Com_DebugTextEdit->setObjectName(QString::fromUtf8("Com_DebugTextEdit"));

        gridLayout_8->addWidget(Com_DebugTextEdit, 1, 1, 1, 1);

        Com_ReadDebugTextedit = new QPushButton(gridLayoutWidget);
        Com_ReadDebugTextedit->setObjectName(QString::fromUtf8("Com_ReadDebugTextedit"));
        sizePolicy.setHeightForWidth(Com_ReadDebugTextedit->sizePolicy().hasHeightForWidth());
        Com_ReadDebugTextedit->setSizePolicy(sizePolicy);
        Com_ReadDebugTextedit->setFont(font);

        gridLayout_8->addWidget(Com_ReadDebugTextedit, 0, 1, 1, 1);

        layoutWidget = new QWidget(tab);
        layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));
        layoutWidget->setGeometry(QRect(10, 10, 1389, 301));
        gridLayout = new QGridLayout(layoutWidget);
        gridLayout->setSpacing(10);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        gridLayout->setContentsMargins(6, 6, 6, 6);
        gridLayout_3 = new QGridLayout();
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        Com_ReadFrictionTableFloat = new QPushButton(layoutWidget);
        Com_ReadFrictionTableFloat->setObjectName(QString::fromUtf8("Com_ReadFrictionTableFloat"));
        QSizePolicy sizePolicy1(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(Com_ReadFrictionTableFloat->sizePolicy().hasHeightForWidth());
        Com_ReadFrictionTableFloat->setSizePolicy(sizePolicy1);
        Com_ReadFrictionTableFloat->setFont(font);

        gridLayout_3->addWidget(Com_ReadFrictionTableFloat, 0, 0, 1, 1);

        Com_SendCommanUsDefaultFrictionTableToJtc = new QPushButton(layoutWidget);
        Com_SendCommanUsDefaultFrictionTableToJtc->setObjectName(QString::fromUtf8("Com_SendCommanUsDefaultFrictionTableToJtc"));
        sizePolicy1.setHeightForWidth(Com_SendCommanUsDefaultFrictionTableToJtc->sizePolicy().hasHeightForWidth());
        Com_SendCommanUsDefaultFrictionTableToJtc->setSizePolicy(sizePolicy1);
        Com_SendCommanUsDefaultFrictionTableToJtc->setFont(font);

        gridLayout_3->addWidget(Com_SendCommanUsDefaultFrictionTableToJtc, 0, 2, 1, 1);

        gridLayout_9 = new QGridLayout();
        gridLayout_9->setObjectName(QString::fromUtf8("gridLayout_9"));
        Com_SendFrictionPolynomialCoeffsToJtc = new QPushButton(layoutWidget);
        Com_SendFrictionPolynomialCoeffsToJtc->setObjectName(QString::fromUtf8("Com_SendFrictionPolynomialCoeffsToJtc"));
        sizePolicy1.setHeightForWidth(Com_SendFrictionPolynomialCoeffsToJtc->sizePolicy().hasHeightForWidth());
        Com_SendFrictionPolynomialCoeffsToJtc->setSizePolicy(sizePolicy1);
        Com_SendFrictionPolynomialCoeffsToJtc->setFont(font);

        gridLayout_9->addWidget(Com_SendFrictionPolynomialCoeffsToJtc, 0, 1, 1, 1);


        gridLayout_3->addLayout(gridLayout_9, 1, 1, 1, 1);

        Com_SendFrictionTableToJtc = new QPushButton(layoutWidget);
        Com_SendFrictionTableToJtc->setObjectName(QString::fromUtf8("Com_SendFrictionTableToJtc"));
        sizePolicy1.setHeightForWidth(Com_SendFrictionTableToJtc->sizePolicy().hasHeightForWidth());
        Com_SendFrictionTableToJtc->setSizePolicy(sizePolicy1);
        Com_SendFrictionTableToJtc->setFont(font);

        gridLayout_3->addWidget(Com_SendFrictionTableToJtc, 0, 1, 1, 1);

        Com_SendCommanUsDefaultFrictionPolynomialCoeffsToJtc = new QPushButton(layoutWidget);
        Com_SendCommanUsDefaultFrictionPolynomialCoeffsToJtc->setObjectName(QString::fromUtf8("Com_SendCommanUsDefaultFrictionPolynomialCoeffsToJtc"));
        sizePolicy1.setHeightForWidth(Com_SendCommanUsDefaultFrictionPolynomialCoeffsToJtc->sizePolicy().hasHeightForWidth());
        Com_SendCommanUsDefaultFrictionPolynomialCoeffsToJtc->setSizePolicy(sizePolicy1);
        Com_SendCommanUsDefaultFrictionPolynomialCoeffsToJtc->setFont(font);

        gridLayout_3->addWidget(Com_SendCommanUsDefaultFrictionPolynomialCoeffsToJtc, 1, 2, 1, 1);

        Com_ReadFrictionPolynomialCoeffsFloat = new QPushButton(layoutWidget);
        Com_ReadFrictionPolynomialCoeffsFloat->setObjectName(QString::fromUtf8("Com_ReadFrictionPolynomialCoeffsFloat"));
        sizePolicy1.setHeightForWidth(Com_ReadFrictionPolynomialCoeffsFloat->sizePolicy().hasHeightForWidth());
        Com_ReadFrictionPolynomialCoeffsFloat->setSizePolicy(sizePolicy1);
        Com_ReadFrictionPolynomialCoeffsFloat->setFont(font);

        gridLayout_3->addWidget(Com_ReadFrictionPolynomialCoeffsFloat, 1, 0, 1, 1);


        gridLayout->addLayout(gridLayout_3, 2, 0, 1, 5);

        Com_OpenButt = new QPushButton(layoutWidget);
        Com_OpenButt->setObjectName(QString::fromUtf8("Com_OpenButt"));
        sizePolicy1.setHeightForWidth(Com_OpenButt->sizePolicy().hasHeightForWidth());
        Com_OpenButt->setSizePolicy(sizePolicy1);
        Com_OpenButt->setFont(font);

        gridLayout->addWidget(Com_OpenButt, 0, 1, 1, 1);

        gridLayout_2 = new QGridLayout();
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        Com_SendTrajectoryToJtc = new QPushButton(layoutWidget);
        Com_SendTrajectoryToJtc->setObjectName(QString::fromUtf8("Com_SendTrajectoryToJtc"));
        sizePolicy1.setHeightForWidth(Com_SendTrajectoryToJtc->sizePolicy().hasHeightForWidth());
        Com_SendTrajectoryToJtc->setSizePolicy(sizePolicy1);
        Com_SendTrajectoryToJtc->setFont(font);

        gridLayout_2->addWidget(Com_SendTrajectoryToJtc, 0, 1, 1, 1);

        Com_ReadTrajectoryInt16 = new QPushButton(layoutWidget);
        Com_ReadTrajectoryInt16->setObjectName(QString::fromUtf8("Com_ReadTrajectoryInt16"));
        sizePolicy1.setHeightForWidth(Com_ReadTrajectoryInt16->sizePolicy().hasHeightForWidth());
        Com_ReadTrajectoryInt16->setSizePolicy(sizePolicy1);
        Com_ReadTrajectoryInt16->setFont(font);

        gridLayout_2->addWidget(Com_ReadTrajectoryInt16, 0, 0, 1, 1);


        gridLayout->addLayout(gridLayout_2, 5, 0, 1, 5);

        Com_CloseButt = new QPushButton(layoutWidget);
        Com_CloseButt->setObjectName(QString::fromUtf8("Com_CloseButt"));
        sizePolicy1.setHeightForWidth(Com_CloseButt->sizePolicy().hasHeightForWidth());
        Com_CloseButt->setSizePolicy(sizePolicy1);
        Com_CloseButt->setFont(font);

        gridLayout->addWidget(Com_CloseButt, 0, 2, 1, 1);

        Com_NameComboBox = new QComboBox(layoutWidget);
        Com_NameComboBox->setObjectName(QString::fromUtf8("Com_NameComboBox"));
        sizePolicy1.setHeightForWidth(Com_NameComboBox->sizePolicy().hasHeightForWidth());
        Com_NameComboBox->setSizePolicy(sizePolicy1);

        gridLayout->addWidget(Com_NameComboBox, 0, 0, 1, 1);

        gridLayout_6 = new QGridLayout();
        gridLayout_6->setObjectName(QString::fromUtf8("gridLayout_6"));
        Com_ReadArmModel = new QPushButton(layoutWidget);
        Com_ReadArmModel->setObjectName(QString::fromUtf8("Com_ReadArmModel"));
        sizePolicy1.setHeightForWidth(Com_ReadArmModel->sizePolicy().hasHeightForWidth());
        Com_ReadArmModel->setSizePolicy(sizePolicy1);
        Com_ReadArmModel->setFont(font);

        gridLayout_6->addWidget(Com_ReadArmModel, 0, 0, 1, 1);

        Com_SendArmModelToJtc = new QPushButton(layoutWidget);
        Com_SendArmModelToJtc->setObjectName(QString::fromUtf8("Com_SendArmModelToJtc"));
        sizePolicy1.setHeightForWidth(Com_SendArmModelToJtc->sizePolicy().hasHeightForWidth());
        Com_SendArmModelToJtc->setSizePolicy(sizePolicy1);
        Com_SendArmModelToJtc->setFont(font);

        gridLayout_6->addWidget(Com_SendArmModelToJtc, 0, 1, 1, 1);

        Com_SendCommandUseDefaultArmModelToJtc = new QPushButton(layoutWidget);
        Com_SendCommandUseDefaultArmModelToJtc->setObjectName(QString::fromUtf8("Com_SendCommandUseDefaultArmModelToJtc"));
        sizePolicy1.setHeightForWidth(Com_SendCommandUseDefaultArmModelToJtc->sizePolicy().hasHeightForWidth());
        Com_SendCommandUseDefaultArmModelToJtc->setSizePolicy(sizePolicy1);
        Com_SendCommandUseDefaultArmModelToJtc->setFont(font);

        gridLayout_6->addWidget(Com_SendCommandUseDefaultArmModelToJtc, 0, 2, 1, 1);


        gridLayout->addLayout(gridLayout_6, 4, 0, 1, 5);

        gridLayout_7 = new QGridLayout();
        gridLayout_7->setObjectName(QString::fromUtf8("gridLayout_7"));
        Com_SendCommandUseDefaultPidParamlToJtc = new QPushButton(layoutWidget);
        Com_SendCommandUseDefaultPidParamlToJtc->setObjectName(QString::fromUtf8("Com_SendCommandUseDefaultPidParamlToJtc"));
        sizePolicy1.setHeightForWidth(Com_SendCommandUseDefaultPidParamlToJtc->sizePolicy().hasHeightForWidth());
        Com_SendCommandUseDefaultPidParamlToJtc->setSizePolicy(sizePolicy1);
        Com_SendCommandUseDefaultPidParamlToJtc->setFont(font);

        gridLayout_7->addWidget(Com_SendCommandUseDefaultPidParamlToJtc, 0, 2, 1, 1);

        Com_ReadPidParam = new QPushButton(layoutWidget);
        Com_ReadPidParam->setObjectName(QString::fromUtf8("Com_ReadPidParam"));
        sizePolicy1.setHeightForWidth(Com_ReadPidParam->sizePolicy().hasHeightForWidth());
        Com_ReadPidParam->setSizePolicy(sizePolicy1);
        Com_ReadPidParam->setFont(font);

        gridLayout_7->addWidget(Com_ReadPidParam, 0, 0, 1, 1);

        Com_SendPidParamlToJtc = new QPushButton(layoutWidget);
        Com_SendPidParamlToJtc->setObjectName(QString::fromUtf8("Com_SendPidParamlToJtc"));
        sizePolicy1.setHeightForWidth(Com_SendPidParamlToJtc->sizePolicy().hasHeightForWidth());
        Com_SendPidParamlToJtc->setSizePolicy(sizePolicy1);
        Com_SendPidParamlToJtc->setFont(font);

        gridLayout_7->addWidget(Com_SendPidParamlToJtc, 0, 1, 1, 1);


        gridLayout->addLayout(gridLayout_7, 3, 0, 1, 5);

        Com_OnOffSynchroTransmision = new QPushButton(layoutWidget);
        Com_OnOffSynchroTransmision->setObjectName(QString::fromUtf8("Com_OnOffSynchroTransmision"));
        sizePolicy1.setHeightForWidth(Com_OnOffSynchroTransmision->sizePolicy().hasHeightForWidth());
        Com_OnOffSynchroTransmision->setSizePolicy(sizePolicy1);
        Com_OnOffSynchroTransmision->setFont(font);

        gridLayout->addWidget(Com_OnOffSynchroTransmision, 0, 4, 1, 1);

        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        pathWindowsRadioButton = new QRadioButton(layoutWidget);
        pathWindowsRadioButton->setObjectName(QString::fromUtf8("pathWindowsRadioButton"));
        QSizePolicy sizePolicy2(QSizePolicy::Minimum, QSizePolicy::Expanding);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(pathWindowsRadioButton->sizePolicy().hasHeightForWidth());
        pathWindowsRadioButton->setSizePolicy(sizePolicy2);
        pathWindowsRadioButton->setChecked(false);

        verticalLayout->addWidget(pathWindowsRadioButton);

        pathLinuxRadioButton = new QRadioButton(layoutWidget);
        pathLinuxRadioButton->setObjectName(QString::fromUtf8("pathLinuxRadioButton"));
        sizePolicy2.setHeightForWidth(pathLinuxRadioButton->sizePolicy().hasHeightForWidth());
        pathLinuxRadioButton->setSizePolicy(sizePolicy2);
        pathLinuxRadioButton->setChecked(true);

        verticalLayout->addWidget(pathLinuxRadioButton);


        gridLayout->addLayout(verticalLayout, 0, 3, 1, 1);

        tabWidget->addTab(tab, QString());
        tab_3 = new QWidget();
        tab_3->setObjectName(QString::fromUtf8("tab_3"));
        Com_JointsParamTableView = new QTableView(tab_3);
        Com_JointsParamTableView->setObjectName(QString::fromUtf8("Com_JointsParamTableView"));
        Com_JointsParamTableView->setGeometry(QRect(0, 330, 1291, 291));
        Com_JtcParamTableView = new QTableView(tab_3);
        Com_JtcParamTableView->setObjectName(QString::fromUtf8("Com_JtcParamTableView"));
        Com_JtcParamTableView->setGeometry(QRect(0, 10, 1291, 81));
        gridLayoutWidget_2 = new QWidget(tab_3);
        gridLayoutWidget_2->setObjectName(QString::fromUtf8("gridLayoutWidget_2"));
        gridLayoutWidget_2->setGeometry(QRect(0, 99, 1291, 221));
        gridLayout_5 = new QGridLayout(gridLayoutWidget_2);
        gridLayout_5->setObjectName(QString::fromUtf8("gridLayout_5"));
        gridLayout_5->setContentsMargins(0, 0, 0, 0);
        JTC_ClearErrors = new QPushButton(gridLayoutWidget_2);
        JTC_ClearErrors->setObjectName(QString::fromUtf8("JTC_ClearErrors"));
        sizePolicy1.setHeightForWidth(JTC_ClearErrors->sizePolicy().hasHeightForWidth());
        JTC_ClearErrors->setSizePolicy(sizePolicy1);
        JTC_ClearErrors->setFont(font);

        gridLayout_5->addWidget(JTC_ClearErrors, 1, 0, 1, 1);

        Traj_TrajectoryExecute = new QPushButton(gridLayoutWidget_2);
        Traj_TrajectoryExecute->setObjectName(QString::fromUtf8("Traj_TrajectoryExecute"));
        sizePolicy1.setHeightForWidth(Traj_TrajectoryExecute->sizePolicy().hasHeightForWidth());
        Traj_TrajectoryExecute->setSizePolicy(sizePolicy1);
        Traj_TrajectoryExecute->setFont(font);

        gridLayout_5->addWidget(Traj_TrajectoryExecute, 0, 2, 1, 1);

        JTC_ClearOccuredErrors = new QPushButton(gridLayoutWidget_2);
        JTC_ClearOccuredErrors->setObjectName(QString::fromUtf8("JTC_ClearOccuredErrors"));
        sizePolicy1.setHeightForWidth(JTC_ClearOccuredErrors->sizePolicy().hasHeightForWidth());
        JTC_ClearOccuredErrors->setSizePolicy(sizePolicy1);
        JTC_ClearOccuredErrors->setFont(font);

        gridLayout_5->addWidget(JTC_ClearOccuredErrors, 1, 2, 1, 1);

        Traj_TrajectoryPause = new QPushButton(gridLayoutWidget_2);
        Traj_TrajectoryPause->setObjectName(QString::fromUtf8("Traj_TrajectoryPause"));
        sizePolicy1.setHeightForWidth(Traj_TrajectoryPause->sizePolicy().hasHeightForWidth());
        Traj_TrajectoryPause->setSizePolicy(sizePolicy1);
        Traj_TrajectoryPause->setFont(font);

        gridLayout_5->addWidget(Traj_TrajectoryPause, 0, 1, 1, 1);

        Traj_TrajectoryStop = new QPushButton(gridLayoutWidget_2);
        Traj_TrajectoryStop->setObjectName(QString::fromUtf8("Traj_TrajectoryStop"));
        sizePolicy1.setHeightForWidth(Traj_TrajectoryStop->sizePolicy().hasHeightForWidth());
        Traj_TrajectoryStop->setSizePolicy(sizePolicy1);
        Traj_TrajectoryStop->setFont(font);

        gridLayout_5->addWidget(Traj_TrajectoryStop, 0, 0, 1, 1);

        JTC_TeachingModeEnable = new QPushButton(gridLayoutWidget_2);
        JTC_TeachingModeEnable->setObjectName(QString::fromUtf8("JTC_TeachingModeEnable"));
        sizePolicy1.setHeightForWidth(JTC_TeachingModeEnable->sizePolicy().hasHeightForWidth());
        JTC_TeachingModeEnable->setSizePolicy(sizePolicy1);
        JTC_TeachingModeEnable->setFont(font);

        gridLayout_5->addWidget(JTC_TeachingModeEnable, 2, 0, 1, 1);

        JTC_TeachingModeDisable = new QPushButton(gridLayoutWidget_2);
        JTC_TeachingModeDisable->setObjectName(QString::fromUtf8("JTC_TeachingModeDisable"));
        sizePolicy1.setHeightForWidth(JTC_TeachingModeDisable->sizePolicy().hasHeightForWidth());
        JTC_TeachingModeDisable->setSizePolicy(sizePolicy1);
        JTC_TeachingModeDisable->setFont(font);

        gridLayout_5->addWidget(JTC_TeachingModeDisable, 2, 2, 1, 1);

        tabWidget->addTab(tab_3, QString());
        Com = new QWidget();
        Com->setObjectName(QString::fromUtf8("Com"));
        Com_TrajTextEdit = new QTextEdit(Com);
        Com_TrajTextEdit->setObjectName(QString::fromUtf8("Com_TrajTextEdit"));
        Com_TrajTextEdit->setGeometry(QRect(0, 10, 1400, 180));
        Com_TrajTableView = new QTableView(Com);
        Com_TrajTableView->setObjectName(QString::fromUtf8("Com_TrajTableView"));
        Com_TrajTableView->setGeometry(QRect(0, 200, 1400, 650));
        tabWidget->addTab(Com, QString());
        Can = new QWidget();
        Can->setObjectName(QString::fromUtf8("Can"));
        Com_FrictionTableTextEdit = new QTextEdit(Can);
        Com_FrictionTableTextEdit->setObjectName(QString::fromUtf8("Com_FrictionTableTextEdit"));
        Com_FrictionTableTextEdit->setGeometry(QRect(0, 10, 1400, 180));
        Com_FrictionTableTableView = new QTableView(Can);
        Com_FrictionTableTableView->setObjectName(QString::fromUtf8("Com_FrictionTableTableView"));
        Com_FrictionTableTableView->setGeometry(QRect(0, 200, 1400, 650));
        tabWidget->addTab(Can, QString());
        tab_4 = new QWidget();
        tab_4->setObjectName(QString::fromUtf8("tab_4"));
        Com_PidParametersTextEdit = new QTextEdit(tab_4);
        Com_PidParametersTextEdit->setObjectName(QString::fromUtf8("Com_PidParametersTextEdit"));
        Com_PidParametersTextEdit->setGeometry(QRect(0, 10, 1400, 180));
        Com_PidParametersTableView = new QTableView(tab_4);
        Com_PidParametersTableView->setObjectName(QString::fromUtf8("Com_PidParametersTableView"));
        Com_PidParametersTableView->setGeometry(QRect(0, 200, 1400, 650));
        tabWidget->addTab(tab_4, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QString::fromUtf8("tab_2"));
        Com_ArmModelLinksTableView = new QTableView(tab_2);
        Com_ArmModelLinksTableView->setObjectName(QString::fromUtf8("Com_ArmModelLinksTableView"));
        Com_ArmModelLinksTableView->setGeometry(QRect(10, 260, 1400, 291));
        Com_ArmModelTextEdit = new QTextEdit(tab_2);
        Com_ArmModelTextEdit->setObjectName(QString::fromUtf8("Com_ArmModelTextEdit"));
        Com_ArmModelTextEdit->setGeometry(QRect(10, 10, 1400, 241));
        Com_ArmModelJointsTableView = new QTableView(tab_2);
        Com_ArmModelJointsTableView->setObjectName(QString::fromUtf8("Com_ArmModelJointsTableView"));
        Com_ArmModelJointsTableView->setGeometry(QRect(10, 560, 1400, 291));
        tabWidget->addTab(tab_2, QString());
        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 1600, 21));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);

        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "MainWindow", nullptr));
        Com_ReadTextEdit_2->setText(QCoreApplication::translate("MainWindow", "Clear", nullptr));
        Com_ReadDebugTextedit->setText(QCoreApplication::translate("MainWindow", "Clear", nullptr));
        Com_ReadFrictionTableFloat->setText(QCoreApplication::translate("MainWindow", "Read Friction Table Float", nullptr));
        Com_SendCommanUsDefaultFrictionTableToJtc->setText(QCoreApplication::translate("MainWindow", "Send Command Use Default Friction Talbe", nullptr));
        Com_SendFrictionPolynomialCoeffsToJtc->setText(QCoreApplication::translate("MainWindow", "Send Friction Polynomial Coeffs To JTC", nullptr));
        Com_SendFrictionTableToJtc->setText(QCoreApplication::translate("MainWindow", "Send Friction Table To JTC", nullptr));
        Com_SendCommanUsDefaultFrictionPolynomialCoeffsToJtc->setText(QCoreApplication::translate("MainWindow", "Send Command Use Default Friction Polynomial Coeffs", nullptr));
        Com_ReadFrictionPolynomialCoeffsFloat->setText(QCoreApplication::translate("MainWindow", "Read Friction Polynomial Coeffs Float", nullptr));
        Com_OpenButt->setText(QCoreApplication::translate("MainWindow", "Open Com Port", nullptr));
        Com_SendTrajectoryToJtc->setText(QCoreApplication::translate("MainWindow", "Send Trajectory To JTC", nullptr));
        Com_ReadTrajectoryInt16->setText(QCoreApplication::translate("MainWindow", "Read Trajectory Int16_t", nullptr));
        Com_CloseButt->setText(QCoreApplication::translate("MainWindow", "Close Com Port", nullptr));
        Com_ReadArmModel->setText(QCoreApplication::translate("MainWindow", "Read Arm Model", nullptr));
        Com_SendArmModelToJtc->setText(QCoreApplication::translate("MainWindow", "Send Arm Model to JTC", nullptr));
        Com_SendCommandUseDefaultArmModelToJtc->setText(QCoreApplication::translate("MainWindow", "Send Command Use Default Arm Model", nullptr));
        Com_SendCommandUseDefaultPidParamlToJtc->setText(QCoreApplication::translate("MainWindow", "Send Command Use Default Pid Parameters", nullptr));
        Com_ReadPidParam->setText(QCoreApplication::translate("MainWindow", "Read Pid Parameters", nullptr));
        Com_SendPidParamlToJtc->setText(QCoreApplication::translate("MainWindow", "Send Pid Parameters to JTC", nullptr));
        Com_OnOffSynchroTransmision->setText(QCoreApplication::translate("MainWindow", "On/Off Synchronic transmision", nullptr));
        pathWindowsRadioButton->setText(QCoreApplication::translate("MainWindow", "Windows path", nullptr));
        pathLinuxRadioButton->setText(QCoreApplication::translate("MainWindow", "Linux path", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab), QCoreApplication::translate("MainWindow", "Com Port", nullptr));
        JTC_ClearErrors->setText(QCoreApplication::translate("MainWindow", "Clear Current Errors", nullptr));
        Traj_TrajectoryExecute->setText(QCoreApplication::translate("MainWindow", "Trajectory Execute", nullptr));
        JTC_ClearOccuredErrors->setText(QCoreApplication::translate("MainWindow", "Clear Occured Errors", nullptr));
        Traj_TrajectoryPause->setText(QCoreApplication::translate("MainWindow", "Trajectory Pause", nullptr));
        Traj_TrajectoryStop->setText(QCoreApplication::translate("MainWindow", "Trajectory Stop", nullptr));
        JTC_TeachingModeEnable->setText(QCoreApplication::translate("MainWindow", "Teaching Mode Enable", nullptr));
        JTC_TeachingModeDisable->setText(QCoreApplication::translate("MainWindow", "Teaching Mode Disable", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_3), QCoreApplication::translate("MainWindow", "JTC Status", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(Com), QCoreApplication::translate("MainWindow", "Trajectory", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(Can), QCoreApplication::translate("MainWindow", "Friction table", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_4), QCoreApplication::translate("MainWindow", "Pid Parameters", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QCoreApplication::translate("MainWindow", "Arm model", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
