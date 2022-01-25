#include "mainwindow.h"
#include "mainwindow.h"
#include "ui_mainwindow.h"

union conv32
{
    uint32_t u32; // here_write_bits
    float    f32; // here_read_float
};
MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    foreach (const QSerialPortInfo &serialPortInfo, QSerialPortInfo::availablePorts())
        ui->Com_NameComboBox->addItem(serialPortInfo.portName());
    trajStandardItemModel = new QStandardItemModel(this);
    frictionTableStandardItemModel = new QStandardItemModel(this);
    pidParametersStandardItemModel = new QStandardItemModel(this);
    jtcParametersStandardItemModel = new QStandardItemModel(this);
    jointsParametersStandardItemModel = new QStandardItemModel(this);
    joints[0].fricTableFilePath = "..\\..\\Dane JTC\\FricTableJoint0.csv";
    joints[1].fricTableFilePath = "..\\..\\Dane JTC\\FricTableJoint1.csv";
    joints[2].fricTableFilePath = "..\\..\\Dane JTC\\FricTableJoint2.csv";
    joints[3].fricTableFilePath = "..\\..\\Dane JTC\\FricTableJoint3.csv";
    joints[4].fricTableFilePath = "..\\..\\Dane JTC\\FricTableJoint4.csv";
    joints[5].fricTableFilePath = "..\\..\\Dane JTC\\FricTableJoint5.csv";
    joints[0].pidParamFilePath = "..\\..\\Dane JTC\\PidParamJoint0.csv";
    joints[1].pidParamFilePath = "..\\..\\Dane JTC\\PidParamJoint1.csv";
    joints[2].pidParamFilePath = "..\\..\\Dane JTC\\PidParamJoint2.csv";
    joints[3].pidParamFilePath = "..\\..\\Dane JTC\\PidParamJoint3.csv";
    joints[4].pidParamFilePath = "..\\..\\Dane JTC\\PidParamJoint4.csv";
    joints[5].pidParamFilePath = "..\\..\\Dane JTC\\PidParamJoint5.csv";
    armModelFilePath = "..\\..\\Dane JTC\\Avena.urdf";

    SetDefualtArmModel();

    comAsynchronicSend = false;
    comSynchroTransmisionEnable = true;
    numFrameToAsynchroSend = Host_FTAS_Null;
    comReadString.clear();

    timerSerialSend = new QTimer(this);
    timerSerialRead = new QTimer(this);
    timerSerialTimeout = new QTimer(this);
    timerLabels = new QTimer(this);
    serial = new QSerialPort(this);
    connect(timerSerialSend, SIGNAL(timeout()), this, SLOT(Com_Send()));
    connect(timerSerialRead, SIGNAL(timeout()), this, SLOT(Com_Read()));
    connect(timerSerialTimeout, SIGNAL(timeout()), this, SLOT(Com_Timeout()));
    connect(timerLabels, SIGNAL(timeout()), this, SLOT(RefreshLabels()));
//    connect(serial, SIGNAL(readyRead()), this, SLOT(Com_Read()));
    timerLabels->start(100);
}
MainWindow::~MainWindow()
{
    delete ui;
}
uint16_t MainWindow::Com_Crc16(uint8_t *packet, uint32_t nBytes)
{
    uint16_t crc = 0;
    for(uint32_t byte = 0; byte < nBytes; byte++)
    {
        crc = crc ^ ((uint16_t)packet[byte] << 8);
        for (uint8_t bit = 0; bit < 8; bit++)
            if(crc & 0x8000) 	crc = (crc << 1) ^ 0x1021;
            else							crc = crc << 1;
    }
    return crc;
}
uint16_t MainWindow::Com_Crc16v2(QByteArray packet, uint32_t nBytes)
{
    uint16_t crc = 0;
    for(uint32_t byte = 0; byte < nBytes; byte++)
    {
        crc = crc ^ ((uint16_t)packet[byte] << 8);
        for (uint8_t bit = 0; bit < 8; bit++)
            if(crc & 0x8000) 	crc = (crc << 1) ^ 0x1021;
            else							crc = crc << 1;
    }
    return crc;
}
static void Control_SetTable(double* t, double a0, double a1, double a2, double a3, double a4, double a5)
{
    t[0] = a0; t[1] = a1; t[2] = a2; t[3] = a3; t[4] = a4; t[5] = a5;
}
void MainWindow::SetDefualtArmModel()
{
    Arm.Joints.resize(ARMMODEL_DOF+1);
    Arm.Links.resize(ARMMODEL_DOF+1);

    Control_SetTable(Arm.Joints[0].origin, 0, 0, 0.063, 0, 0, -0.1963);						//joint 0 coordinate system
    Control_SetTable(Arm.Joints[1].origin, 0.064, 0, 0.0495, 0, 1.570796, 0);				//joint 1 coordinate system
    Control_SetTable(Arm.Joints[2].origin, -0.452, 0, 0, 0, 3.141592, 0);					//joint 2 coordinate system
    Control_SetTable(Arm.Joints[3].origin, 0.452, 0, 0.0226, 3.141592, 0, 0);				//joint 3 coordinate system
    Control_SetTable(Arm.Joints[4].origin, 0.0495, 0, 0.064, 0, -1.570796, 3.141592);		//joint 4 coordinate system
    Control_SetTable(Arm.Joints[5].origin, 0.0495, 0, 0.064, 0, 1.570796, 0);				//joint 5 coordinate system
    Control_SetTable(Arm.Joints[6].origin, 0, 0, 0.0181, 0, 0, 0);							//joint 6 coordinate system

    Control_SetTable(Arm.Links[0].origin, 0, -0.0129, 0.00034, 0, 0, 0);					//center of mass link 0 coordinate system
    Control_SetTable(Arm.Links[1].origin, 0.019, 0, 0.0445, 0, 0, 1.570796);				//center of mass link 1 coordinate system
    Control_SetTable(Arm.Links[2].origin, -0.226, 0, 0.0334, 0, 0, 1.570796);				//center of mass link 2 coordinate system
    Control_SetTable(Arm.Links[3].origin, 0.3046, 0, 0.0588, 0, 0, 1.570796);				//center of mass link 3 coordinate system
    Control_SetTable(Arm.Links[4].origin, 0.019, 0, 0.059, 0, 0, 1.570796);					//center of mass link 4 coordinate system
    Control_SetTable(Arm.Links[5].origin, 0.019, 0, 0.059, 0, 0, 1.570796);					//center of mass link 5 coordinate system
    Control_SetTable(Arm.Links[6].origin, 0, 0, 0, 0, 0, 0);														//center of mass link 6 coordinate system

    Control_SetTable(Arm.Links[0].innertia, 1, 0, 0, 1, 0, 1);								//link 0 innertial values
    Control_SetTable(Arm.Links[1].innertia, 0.002233, 0, 0, 0.002047, 0, 0.002384);			//link 1 innertial values
    Control_SetTable(Arm.Links[2].innertia, 0.1941, 0, 0, 0.005522, 0, 0.1941);				//link 2 innertial values
    Control_SetTable(Arm.Links[3].innertia, 0.1149, 0, 0, 0.004518, 0, 0.1148);				//link 3 innertial values
    Control_SetTable(Arm.Links[4].innertia, 0.002233, 0, 0, 0.002047, 0, 0.002384);			//link 4 innertial values
    Control_SetTable(Arm.Links[5].innertia, 0.002233, 0, 0, 0.002047, 0, 0.002384);			//link 5 innertial values
    Control_SetTable(Arm.Links[6].innertia, 0.000041, 0, 0, 0.000042, 0, 0.00008);			//link 6 innertial values

    Arm.Links[0].mass = 0.750;		//mass of link 0
    Arm.Links[1].mass = 1.960;		//mass of link 1
    Arm.Links[2].mass = 4.660;		//mass of link 2
    Arm.Links[3].mass = 3.375;		//mass of link 3
    Arm.Links[4].mass = 1.960;		//mass of link 4
    Arm.Links[5].mass = 1.960;		//mass of link 5
    Arm.Links[6].mass = 0.082;		//mass of link 6
}
void MainWindow::ShowTrajectory()
{
    trajStandardItemModel->clear();
    ui->Com_TrajTableView->setModel(trajStandardItemModel);
    int rows;
    QList<QList<QStandardItem*>> itemTable;
    QStringList columnHeader;
    columnHeader.append("Time (ms)");
    for(uint8_t i=0;i<JOINTS_MAX;i++)
    {
        QString str = "J " + QString::number(i) + " Pos";
        columnHeader.append(str);
    }
    for(uint8_t i=0;i<JOINTS_MAX;i++)
    {
        QString str = "J " + QString::number(i) + " Vel";
        columnHeader.append(str);
    }
    for(uint8_t i=0;i<JOINTS_MAX;i++)
    {
        QString str = "J " + QString::number(i) + " Acc";
        columnHeader.append(str);
    }
    trajStandardItemModel->insertColumns(0, columnHeader.length());
    for(int i=0;i<columnHeader.length();i++)
    {
        trajStandardItemModel->setHeaderData(i, Qt::Horizontal, columnHeader[i]);
    }
    for(int i=0;i<traj.value.length();i++)
    {
        rows = trajStandardItemModel->rowCount();
        trajStandardItemModel->insertRow(rows);

        QList<QStandardItem*> itemList;
        itemTable.append(itemList);
        for(int j=0;j<traj.value[i].length();j++)
        {
            QStandardItem* item = new QStandardItem;
            if(j == 0)
                item->setText(QString::number(traj.value[i][j]));
            else
                item->setText(QString::number(traj.value[i][j],'f',6));
            item->setTextAlignment(Qt::AlignRight);
            itemList.append(item);
            trajStandardItemModel->setItem(i, j, itemList[j]);
        }
    }
    ui->Com_TrajTableView->show();
    ui->Com_TrajTableView->resizeColumnsToContents();
}
void MainWindow::ShowJtcValues()
{
    jtcParametersStandardItemModel->clear();
    ui->Com_JtcParamTableView->setModel(jtcParametersStandardItemModel);
    int rows;
    QList<QList<QStandardItem*>> itemTable;

    QStringList columnHeader = {"Jtc Fsm", "Jtc Errors","Jtc Occured Errors", "Jtc Init Status", "Joints Init Status", "Traj. Status", "Traj. num of point", "Can Tx timeout", "Can Rx timeout"};

    jtcParametersStandardItemModel->insertColumns(0, columnHeader.length());
    for(int i=0;i<columnHeader.length();i++)
    {
        jtcParametersStandardItemModel->setHeaderData(i, Qt::Horizontal, columnHeader[i]);
    }

    rows = jtcParametersStandardItemModel->rowCount();
    jtcParametersStandardItemModel->insertRow(rows);
    QString rowHeaderStr = "Jtc ";
    jtcParametersStandardItemModel->setHeaderData(0, Qt::Vertical, rowHeaderStr);
    QList<QStandardItem*> itemList;
    itemTable.append(itemList);

    QString val;
    if(jtcFsm == JTC_FSM_Start)
        val = "JTC_FSM_Start";
    else if(jtcFsm == JTC_FSM_Init)
        val = "JTC_FSM_Init";
    else if(jtcFsm == JTC_FSM_Operate)
        val = "JTC_FSM_Operate";
    else if(jtcFsm == JTC_FSM_Error)
        val = "JTC_FSM_Error";
    else
        val = "Jtc_FSM_Unknown";

    QStandardItem* itemFsm = new QStandardItem;
    itemFsm->setText(val);
    itemFsm->setTextAlignment(Qt::AlignRight);
    itemList.append(itemFsm);
    jtcParametersStandardItemModel->setItem(0, 0, itemList[0]);

    QStandardItem* itemJtcErrors = new QStandardItem;
    itemJtcErrors->setText(QString::number(jtcErrors, 2));
    itemJtcErrors->setTextAlignment(Qt::AlignRight);
    itemList.append(itemJtcErrors);
    jtcParametersStandardItemModel->setItem(0, 1, itemList[1]);

    QStandardItem* itemJtcOccuredErrors = new QStandardItem;
    itemJtcOccuredErrors->setText(QString::number(jtcOccuredErrors, 2));
    itemJtcOccuredErrors->setTextAlignment(Qt::AlignRight);
    itemList.append(itemJtcOccuredErrors);
    jtcParametersStandardItemModel->setItem(0, 2, itemList[2]);

    QStandardItem* itemJtcInitStatus = new QStandardItem;
    itemJtcInitStatus->setText(QString::number(jtcInitStatus, 2));
    itemJtcInitStatus->setTextAlignment(Qt::AlignRight);
    itemList.append(itemJtcInitStatus);
    jtcParametersStandardItemModel->setItem(0, 3, itemList[3]);

    QStandardItem* itemJointsInitStatus = new QStandardItem;
    itemJointsInitStatus->setText(QString::number(jointsInitStatus, 2));
    itemJointsInitStatus->setTextAlignment(Qt::AlignRight);
    itemList.append(itemJointsInitStatus);
    jtcParametersStandardItemModel->setItem(0, 4, itemList[4]);

    if(traj.currentStatus == TES_Null)
        val = "TES_Null";
    else if(traj.currentStatus == TES_Stop)
        val = "TES_Stop";
    else if(traj.currentStatus == TES_Pause)
        val = "TES_Pause";
    else if(traj.currentStatus == TES_Execute)
        val = "TES_Execute";
    else if(traj.currentStatus == TES_Finish)
        val = "TES_Finish";
    else
        val = "TRAJ_FSM_Unknown";
    QStandardItem* itemTrajStatus = new QStandardItem;
    itemTrajStatus->setText(val);
    itemTrajStatus->setTextAlignment(Qt::AlignRight);
    itemList.append(itemTrajStatus);
    jtcParametersStandardItemModel->setItem(0, 5, itemList[5]);

    QStandardItem* itemTrajNumPoint = new QStandardItem;
    itemTrajNumPoint->setText(QString::number(traj.numCurrentPoint));
    itemTrajNumPoint->setTextAlignment(Qt::AlignRight);
    itemList.append(itemTrajNumPoint);
    jtcParametersStandardItemModel->setItem(0, 6, itemList[6]);

    uint8_t canTxTimeout = canStatusFlags;
    QStandardItem* itemCanTxTimeout = new QStandardItem;
    itemCanTxTimeout->setText(QString::number(canTxTimeout, 2));
    itemCanTxTimeout->setTextAlignment(Qt::AlignRight);
    itemList.append(itemCanTxTimeout);
    jtcParametersStandardItemModel->setItem(0, 7, itemList[7]);

    uint8_t canRxTimeout = canStatusFlags >> 8;
    QStandardItem* itemCanRxTimeout = new QStandardItem;
    itemCanRxTimeout->setText(QString::number(canRxTimeout, 2));
    itemCanRxTimeout->setTextAlignment(Qt::AlignRight);
    itemList.append(itemCanRxTimeout);
    jtcParametersStandardItemModel->setItem(0, 8, itemList[8]);

    ui->Com_JtcParamTableView->show();
    ui->Com_JtcParamTableView->resizeColumnsToContents();
}
void MainWindow::ShowJointsValues()
{
    jointsParametersStandardItemModel->clear();
    ui->Com_JointsParamTableView->setModel(jointsParametersStandardItemModel);
    int rows;
    QList<QList<QStandardItem*>> itemTable;

    QStringList columnHeader = {"CurrentFsm", "Pos","Vel","Torque","Temp.", "MC Current Errors","MC Occured Errors","Current Errors","Current Warning","Internall Errors", "Internall Occured Errors"};

    jointsParametersStandardItemModel->insertColumns(0, columnHeader.length());
    for(int i=0;i<columnHeader.length();i++)
    {
        jointsParametersStandardItemModel->setHeaderData(i, Qt::Horizontal, columnHeader[i]);
    }

    for(int i=0;i<JOINTS_MAX;i++)
    {
        rows = jointsParametersStandardItemModel->rowCount();
        jointsParametersStandardItemModel->insertRow(rows);
        QString rowHeaderStr = "Joint " + QString::number(i);
        jointsParametersStandardItemModel->setHeaderData(i, Qt::Vertical, rowHeaderStr);
        QList<QStandardItem*> itemList;
        itemTable.append(itemList);

        QString val;
        if(joints[i].currentFsm == Joint_FSM_Start)
            val = "Joint_FSM_Start";
        else if(joints[i].currentFsm == Joint_FSM_Init)
            val = "Joint_FSM_Init";
        else if(joints[i].currentFsm == Joint_FSM_ReadyToOperate)
            val = "Joint_FSM_ReadyToOperate";
        else if(joints[i].currentFsm == Joint_FSM_OperatioEnable)
            val = "Joint_FSM_OperatioEnable";
        else if(joints[i].currentFsm == Joint_FSM_TransStartToInit)
            val = "Joint_FSM_TransStartToInit";
        else if(joints[i].currentFsm == Joint_FSM_TransInitToReadyToOperate)
            val = "Joint_FSM_TransInitToReadyToOperate";
        else if(joints[i].currentFsm == Joint_FSM_TransReadyToOperateToOperationEnable)
            val = "Joint_FSM_TransReadyToOperateToOperationEnable";
        else if(joints[i].currentFsm == Joint_FSM_TransOperationEnableToReadyToOperate)
            val = "Joint_FSM_TransOperationEnableToReadyToOperate";
        else if(joints[i].currentFsm == Joint_FSM_TransFaultReactionActiveToFault)
            val = "Joint_FSM_TransFaultReactionActiveToFault";
        else if(joints[i].currentFsm == Joint_FSM_TransFaultToReadyToOperate)
            val = "Joint_FSM_TransFaultToReadyToOperate";
        else if(joints[i].currentFsm == Joint_FSM_ReactionActive)
            val = "Joint_FSM_ReactionActive";
        else if(joints[i].currentFsm == Joint_FSM_Fault)
            val = "Joint_FSM_Fault";
        else
            val = "Joint_FSM_Unknown";

        QStandardItem* itemFsm = new QStandardItem;
        itemFsm->setText(val);
        itemFsm->setTextAlignment(Qt::AlignRight);
        itemList.append(itemFsm);
        jointsParametersStandardItemModel->setItem(i, 0, itemList[0]);

        QStandardItem* itemPos = new QStandardItem;
        itemPos->setText(QString::number(joints[i].pos, 'f', 3));
        itemPos->setTextAlignment(Qt::AlignRight);
        itemList.append(itemPos);
        jointsParametersStandardItemModel->setItem(i, 1, itemList[1]);

        QStandardItem* itemVel = new QStandardItem;
        itemVel->setText(QString::number(joints[i].vel, 'f', 3));
        itemVel->setTextAlignment(Qt::AlignRight);
        itemList.append(itemVel);
        jointsParametersStandardItemModel->setItem(i, 2, itemList[2]);

        QStandardItem* itemTorque = new QStandardItem;
        itemTorque->setText(QString::number(joints[i].torque, 'f', 3));
        itemTorque->setTextAlignment(Qt::AlignRight);
        itemList.append(itemTorque);
        jointsParametersStandardItemModel->setItem(i, 3, itemList[3]);

        QStandardItem* itemTemperature = new QStandardItem;
        itemTemperature->setText(QString::number(joints[i].temperature, 'f', 1));
        itemTemperature->setTextAlignment(Qt::AlignRight);
        itemList.append(itemTemperature);
        jointsParametersStandardItemModel->setItem(i, 4, itemList[4]);

        QStandardItem* itemMcCurrentError = new QStandardItem;
        itemMcCurrentError->setText(QString::number(joints[i].mcCurrentError, 2));
        itemMcCurrentError->setTextAlignment(Qt::AlignRight);
        itemList.append(itemMcCurrentError);
        jointsParametersStandardItemModel->setItem(i, 5, itemList[5]);

        QStandardItem* itemMcOccuredError = new QStandardItem;
        itemMcOccuredError->setText(QString::number(joints[i].mcOccuredError, 2));
        itemMcOccuredError->setTextAlignment(Qt::AlignRight);
        itemList.append(itemMcOccuredError);
        jointsParametersStandardItemModel->setItem(i, 6, itemList[6]);

        QStandardItem* itemCurrentError = new QStandardItem;
        itemCurrentError->setText(QString::number(joints[i].currentError, 2));
        itemCurrentError->setTextAlignment(Qt::AlignRight);
        itemList.append(itemCurrentError);
        jointsParametersStandardItemModel->setItem(i, 7, itemList[7]);

        QStandardItem* itemCurrentWarning = new QStandardItem;
        itemCurrentWarning->setText(QString::number(joints[i].currentWarning, 2));
        itemCurrentWarning->setTextAlignment(Qt::AlignRight);
        itemList.append(itemCurrentWarning);
        jointsParametersStandardItemModel->setItem(i, 8, itemList[8]);

        QStandardItem* itemInternallErrors = new QStandardItem;
        itemInternallErrors->setText(QString::number(joints[i].internallErrors, 2));
        itemInternallErrors->setTextAlignment(Qt::AlignRight);
        itemList.append(itemInternallErrors);
        jointsParametersStandardItemModel->setItem(i, 9, itemList[9]);

        QStandardItem* itemInternallOccuredErrors = new QStandardItem;
        itemInternallOccuredErrors->setText(QString::number(joints[i].internallOccuredErrors, 2));
        itemInternallOccuredErrors->setTextAlignment(Qt::AlignRight);
        itemList.append(itemInternallOccuredErrors);
        jointsParametersStandardItemModel->setItem(i, 10, itemList[10]);
    }
    ui->Com_JointsParamTableView->show();
    ui->Com_JointsParamTableView->resizeColumnsToContents();
}
void MainWindow::RedrawButtons(void)
{
    ui->Com_OnOffSynchroTransmision->setStyleSheet("background-color: lightgray");
    ui->Traj_TrajectoryExecute->setStyleSheet("background-color: lightgray");
    ui->Traj_TrajectoryPause->setStyleSheet("background-color: lightgray");
    ui->Traj_TrajectoryStop->setStyleSheet("background-color: lightgray");

    if(comSynchroTransmisionEnable == true)
        ui->Com_OnOffSynchroTransmision->setStyleSheet("background-color: lightgreen");

    if(traj.currentStatus == TES_Execute)
        ui->Traj_TrajectoryExecute->setStyleSheet("background-color: lightgreen");
    if(traj.currentStatus == TES_Pause)
        ui->Traj_TrajectoryPause->setStyleSheet("background-color: lightgreen");
    if(traj.currentStatus == TES_Stop)
        ui->Traj_TrajectoryStop->setStyleSheet("background-color: lightgreen");
}
void MainWindow::RefreshLabels()
{
    ShowJtcValues();
    ShowJointsValues();
    RedrawButtons();
}
void MainWindow::ShowFrictionTable(uint8_t num)
{
    frictionTableStandardItemModel->clear();
    ui->Com_FrictionTableTableView->setModel(frictionTableStandardItemModel);
    int rows;
    QList<QList<QStandardItem*>> itemTable;
    QStringList columnHeader;
    for(uint32_t i=0;i<joints[num].fricTableValue[0].length();i++)
    {
        QString str = "V= " + QString::number(joints[num].fricTableValue[0][i], 'f', 2) + " rad/s";
        columnHeader.append(str);
    }
    frictionTableStandardItemModel->insertColumns(0, columnHeader.length());
    for(int i=0;i<columnHeader.length();i++)
    {
        frictionTableStandardItemModel->setHeaderData(i, Qt::Horizontal, columnHeader[i]);
    }

    for(int i=0;i<joints[num].fricTableValue.length()-2;i++)
    {
        rows = frictionTableStandardItemModel->rowCount();
        frictionTableStandardItemModel->insertRow(rows);

        QString rowHeaderStr = "T= " + QString::number(joints[num].fricTableValue[1][i], 'f', 2) + " C";
        frictionTableStandardItemModel->setHeaderData(i, Qt::Vertical, rowHeaderStr);

        QList<QStandardItem*> itemList;
        itemTable.append(itemList);
        for(int j=0;j<joints[num].fricTableValue[i+2].length();j++)
        {
            QStandardItem* item = new QStandardItem;
            item->setText(QString::number(joints[num].fricTableValue[i+2][j],'f',4));
            item->setTextAlignment(Qt::AlignRight);
            itemList.append(item);
            frictionTableStandardItemModel->setItem(i, j, itemList[j]);
        }
    }

    ui->Com_FrictionTableTableView->show();
    ui->Com_FrictionTableTableView->resizeColumnsToContents();
}
void MainWindow::Com_ReadFrameJtcStatus(uint8_t *buf)
{
    uint32_t idx = 4+5;

    //JTC status
    jtcFsm = (eJTC_FSM)buf[idx++];
    jtcErrors = (uint16_t)buf[idx++] << 8;
    jtcErrors += (uint16_t)buf[idx++] << 0;
    jtcOccuredErrors = (uint16_t)buf[idx++] << 8;
    jtcOccuredErrors += (uint16_t)buf[idx++] << 0;
    jtcInitStatus = buf[idx++];
    jointsInitStatus = buf[idx++];
    traj.currentStatus = (eTrajExecStatus)buf[idx++];
    traj.numCurrentPoint = (uint16_t)buf[idx++] << 8;
    traj.numCurrentPoint += (uint16_t)buf[idx++] << 0;

    //CAN Status
    canStatus = buf[idx++];
    canStatusFlags = (uint16_t)buf[idx++] << 24;
    canStatusFlags += (uint16_t)buf[idx++] << 16;
    canStatusFlags += (uint16_t)buf[idx++] << 8;
    canStatusFlags += (uint16_t)buf[idx++] << 0;
    canOccuredFlags = (uint16_t)buf[idx++] << 24;
    canOccuredFlags += (uint16_t)buf[idx++] << 16;
    canOccuredFlags += (uint16_t)buf[idx++] << 8;
    canOccuredFlags += (uint16_t)buf[idx++] << 0;

    //Joints Status and Values
    union conv32 x;
    for(uint32_t num=0;num<JOINTS_MAX;num++)
    {
        joints[num].currentFsm = buf[idx++];
        joints[num].mcCurrentError = buf[idx++];
        joints[num].mcOccuredError = buf[idx++];
        joints[num].currentError = buf[idx++];
        joints[num].currentWarning = buf[idx++];
        joints[num].internallErrors = (uint16_t)buf[idx++] << 8;
        joints[num].internallErrors += (uint16_t)buf[idx++] << 0;
        joints[num].internallOccuredErrors = (uint16_t)buf[idx++] << 8;
        joints[num].internallOccuredErrors += (uint16_t)buf[idx++] << 0;

        x.u32 = ((uint32_t)buf[idx++] << 24);
        x.u32 += ((uint32_t)buf[idx++] << 16);
        x.u32 += ((uint32_t)buf[idx++] << 8);
        x.u32 += ((uint32_t)buf[idx++] << 0);
        joints[num].pos = x.f32;

        x.u32 = ((uint32_t)buf[idx++] << 24);
        x.u32 += ((uint32_t)buf[idx++] << 16);
        x.u32 += ((uint32_t)buf[idx++] << 8);
        x.u32 += ((uint32_t)buf[idx++] << 0);
        joints[num].vel = x.f32;

        x.u32 = ((uint32_t)buf[idx++] << 24);
        x.u32 += ((uint32_t)buf[idx++] << 16);
        x.u32 += ((uint32_t)buf[idx++] << 8);
        x.u32 += ((uint32_t)buf[idx++] << 0);
        joints[num].torque = x.f32;

        joints[num].temperature = buf[idx++];
    }
}
void MainWindow::Com_ReadFrameReceivedFrameResponse(uint8_t *buf)
{
    if(buf[4] == Host_FT_null)
    {
//        ui->Com_DebugTextEdit->append("Response: Host_FT_null");
        if(buf[5] == Host_RxFS_ErrorIncorrectHeader)
        {
            ui->Com_DebugTextEdit->append("Response: Incorrect header.");
            traj.numOfSegToSend = 0;
        }
        if(buf[5] == Host_RxFS_ErrorIncorrectFrameType)
        {
            ui->Com_DebugTextEdit->append("Response: Incorrect frametype.");
            traj.numOfSegToSend = 0;
        }
        if(buf[5] == Host_RxFS_ErrorIncorrectCrc)
        {
            ui->Com_DebugTextEdit->append("Response: Incorrect CRC.");
            traj.numOfSegToSend = 0;
        }
    }
    if(buf[4] == Host_FT_Trajectory) // odbiór potwierdzenia trajektorii
    {
        if(buf[5] == Host_RxFS_NoError)
        {
            if(buf[6] == Host_RxDS_NoError)
            {
                ui->Com_DebugTextEdit->append("Response trajectory: Data No Error");
                traj.numOfSegToSend++;
                numFrameToAsynchroSend = Host_FTAS_Trajectory;
                if(traj.numOfSegToSend >= traj.lenSegsInTraj)
                {
                    comAsynchronicSend = false;
                    numFrameToAsynchroSend = Host_FTAS_Null;
                }
            }
            if(buf[6] == Host_RxDS_TrajIncorrectSegOrder)
            {
                ui->Com_DebugTextEdit->append("Response trajectory: Incorrect segments order.");
                traj.numOfSegToSend = 0;
            }
        }
    }
    if(buf[4] == Host_FT_ClearCurrentErrors) // odbiór potwierdzenia Clear Current errors
    {
        if(buf[5] == Host_RxFS_NoError)
        {
            if(buf[6] == Host_RxDS_NoError)
            {
                ui->Com_DebugTextEdit->append("Response Clear errors: Data No Error");
                comAsynchronicSend = false;
            }
            else
            {
                ui->Com_DebugTextEdit->append("Response Clear errors: Incorrect data.");
            }
        }
    }
    if(buf[4] == Host_FT_ClearOccuredErrors) // odbiór potwierdzenia Clear occured errors
    {
        if(buf[5] == Host_RxFS_NoError)
        {
            if(buf[6] == Host_RxDS_NoError)
            {
                ui->Com_DebugTextEdit->append("Response Clear errors: Data No Error");
                comAsynchronicSend = false;
            }
            else
            {
                ui->Com_DebugTextEdit->append("Response Clear errors: Incorrect data.");
            }
        }
    }
    if(buf[4] == Host_FT_FrictionTable) // odbiór potwierdzenia Friction table
    {
        if(buf[5] == Host_RxFS_NoError)
        {
            if(buf[6] == Host_RxDS_NoError)
            {
                ui->Com_DebugTextEdit->append("Response Friction table: Data No Error");
                comAsynchronicSend = false;
            }
            else
            {
                ui->Com_DebugTextEdit->append("Response Friction table: Incorrect data.");
            }
        }
    }
    if(buf[4] == Host_FT_FrictionTableUseDefault) // odbiór potwierdzenia Friction table Use Default
    {
        if(buf[5] == Host_RxFS_NoError)
        {
            if(buf[6] == Host_RxDS_NoError)
            {
                ui->Com_DebugTextEdit->append("Response Friction table Use Default: Data No Error");
                comAsynchronicSend = false;
            }
            else
            {
                ui->Com_DebugTextEdit->append("Response Friction table Use Default: Incorrect data.");
            }
        }
    }
    if(buf[4] == Host_FT_PidParamUseDefault) // odbiór potwierdzenia Pid param default
    {
        if(buf[5] == Host_RxFS_NoError)
        {
            if(buf[6] == Host_RxDS_NoError)
            {
                ui->Com_DebugTextEdit->append("Response PID Parameters Use Defaults: Data No Error");
                comAsynchronicSend = false;
            }
            else
            {
                ui->Com_DebugTextEdit->append("Response PID Parameters: Incorrect data  Use Defaults.");
            }
        }
    }
    if(buf[4] == Host_FT_PidParam) // odbiór potwierdzenia PID Parameters
    {
        if(buf[5] == Host_RxFS_NoError)
        {
            if(buf[6] == Host_RxDS_NoError)
            {
                ui->Com_DebugTextEdit->append("Response PID Parameters: Data No Error");
                comAsynchronicSend = false;
            }
            else
            {
                ui->Com_DebugTextEdit->append("Response PID Parameters: Incorrect data.");
            }
        }
    }
    if(buf[4] == Host_FT_ArmModel) // odbiór potwierdzenia Arm model
    {
        if(buf[5] == Host_RxFS_NoError)
        {
            if(buf[6] == Host_RxDS_NoError)
            {
                ui->Com_DebugTextEdit->append("Response Arm model: Data No Error");
                comAsynchronicSend = false;
            }
            else
            {
                ui->Com_DebugTextEdit->append("Response Arm model: Incorrect data.");
            }
        }
    }
    if(buf[4] == Host_FT_ArmModelUseDefault) // odbiór potwierdzenia Arm model Use Default
    {
        if(buf[5] == Host_RxFS_NoError)
        {
            if(buf[6] == Host_RxDS_NoError)
            {
                ui->Com_DebugTextEdit->append("Response Arm model Use Default: Data No Error");
                comAsynchronicSend = false;
            }
            else
            {
                ui->Com_DebugTextEdit->append("Response Arm model Use Default: Incorrect data. Restart transmision.");
            }
        }
    }
    if(buf[4] == Host_FT_TrajSetExecStatus) // odbiór potwierdzenia TrajSetExecStatus
    {
        if(buf[5] == Host_RxFS_NoError)
        {
            if(buf[6] == Host_RxDS_NoError)
            {
                ui->Com_DebugTextEdit->append("Response TrajSetExecStatus: Data No Error");
                comAsynchronicSend = false;
            }
            else
            {
                ui->Com_DebugTextEdit->append("Response TrajSetExecStatus: Incorrect data . Restart transmision.");
            }
        }
    }
    timerSerialSend->start(COMTIMESEND);
}
void MainWindow::Com_SendTrajectory()
{
    ui->Com_DebugTextEdit->append("Rozpoczynam wysylanie segmentu numer " + QString::number(traj.numOfSegToSend+1));
    serial->write(traj.seg[traj.numOfSegToSend].strToSend, traj.seg[traj.numOfSegToSend].lenBytesInSeg);
    serial->waitForBytesWritten(10000);
    QString str = "Wysłano segment numer " + QString::number(traj.numOfSegToSend+1) + " z " + QString::number(traj.lenSegsInTraj) + " segmentów";
    ui->Com_DebugTextEdit->append(str);
    ui->Com_DebugTextEdit->append("Waiting for response");
}
void MainWindow::Com_ReadFrame(uint8_t* buf)
{
    uint16_t nd = ((uint16_t)buf[2] << 8) + ((uint16_t)buf[3] << 0);
    if(nd > COMBUFREADMAX || nd == 0)
        return;
    uint16_t crc1 = Com_Crc16(buf, nd-2);
    uint16_t crc2 = ((uint16_t)buf[nd-2] << 8) + ((uint16_t)buf[nd-1] << 0);
    if(crc1 == crc2 && buf[0] == Host_FT_Header)
    {
        if(buf[1] == Host_FT_JtcStatus)
        {
            Com_ReadFrameReceivedFrameResponse(buf);
            Com_ReadFrameJtcStatus(buf);
        }
    }
    timerSerialTimeout->stop();
}
void MainWindow::Com_PrepareArmModelToSend()
{
    conv32 x;
    for(int i=0;i<Arm.Joints.length();i++)
    {
        for(int j=0;j<6;j++)
        {
            x.f32 = Arm.Joints[i].origin[j];
            armModelWriteString.append(x.u32 >> 24);
            armModelWriteString.append(x.u32 >> 16);
            armModelWriteString.append(x.u32 >> 8);
            armModelWriteString.append(x.u32 >> 0);
        }
    }
    for(int i=0;i<Arm.Links.length();i++)
    {
        for(int j=0;j<6;j++)
        {
            x.f32 = Arm.Links[i].origin[j];
            armModelWriteString.append(x.u32 >> 24);
            armModelWriteString.append(x.u32 >> 16);
            armModelWriteString.append(x.u32 >> 8);
            armModelWriteString.append(x.u32 >> 0);
        }
        for(int j=0;j<6;j++)
        {
            x.f32 = Arm.Links[i].innertia[j];
            armModelWriteString.append(x.u32 >> 24);
            armModelWriteString.append(x.u32 >> 16);
            armModelWriteString.append(x.u32 >> 8);
            armModelWriteString.append(x.u32 >> 0);
        }
        x.f32 = Arm.Links[i].mass;
        armModelWriteString.append(x.u32 >> 24);
        armModelWriteString.append(x.u32 >> 16);
        armModelWriteString.append(x.u32 >> 8);
        armModelWriteString.append(x.u32 >> 0);
    }
}
void MainWindow::Com_SendCommandClearCurrentErrors()
{
    ui->Com_DebugTextEdit->append("Rozpoczynam wysylanie komendy kasowania bieżących błędów w JTC");
    comWriteString.clear();
    comWriteString.append(Host_FT_Header);
    comWriteString.append(Host_FT_ClearCurrentErrors);
    uint16_t nd = comWriteString.length() + 4;
    comWriteString.insert(2, (uint8_t)(nd >> 8));
    comWriteString.insert(3, (uint8_t)(nd >> 0));
    uint16_t crc = Com_Crc16v2(comWriteString, comWriteString.length());
    comWriteString.append(crc >> 8);
    comWriteString.append(crc >> 0);
    serial->write(comWriteString, comWriteString.length());
    ui->Com_DebugTextEdit->append("Wysłano komendę kasowania bieżących błędów w JTC");
    ui->Com_DebugTextEdit->append("Oczekiwanie na odpowiedź");
}
void MainWindow::Com_SendCommandClearOccuredErrors()
{
    ui->Com_DebugTextEdit->append("Rozpoczynam wysylanie komendy kasowania wszystkich błędów w JTC");
    comWriteString.clear();
    comWriteString.append(Host_FT_Header);
    comWriteString.append(Host_FT_ClearOccuredErrors);
    uint16_t nd = comWriteString.length() + 4;
    comWriteString.insert(2, (uint8_t)(nd >> 8));
    comWriteString.insert(3, (uint8_t)(nd >> 0));
    uint16_t crc = Com_Crc16v2(comWriteString, comWriteString.length());
    comWriteString.append(crc >> 8);
    comWriteString.append(crc >> 0);
    serial->write(comWriteString, comWriteString.length());
    ui->Com_DebugTextEdit->append("Wysłano komendę kasowania wszystkich błędów w JTC");
    ui->Com_DebugTextEdit->append("Oczekiwanie na odpowiedź");
}
void MainWindow::Com_Send()
{
    if(comAsynchronicSend == true)
    {
        timerSerialSend->stop();
        if(numFrameToAsynchroSend == Host_FTAS_Trajectory)
        {
            Com_SendTrajectory();
            timerSerialTimeout->start(COMTIMEOUT);
        }
        else if(numFrameToAsynchroSend == Host_FTAS_ClearCurrentErrors)
        {
            Com_SendCommandClearCurrentErrors();
            timerSerialTimeout->start(COMTIMEOUT);
        }
        else if(numFrameToAsynchroSend == Host_FTAS_ClearOccuredErrors)
        {
            Com_SendCommandClearOccuredErrors();
            timerSerialTimeout->start(COMTIMEOUT);
        }
        else if(numFrameToAsynchroSend == Host_FTAS_FrictionTable)
        {
            Com_SendFrictionTable();
            timerSerialTimeout->start(COMTIMEOUT);
        }
        else if(numFrameToAsynchroSend == Host_FTAS_FrictionTableUseDefault)
        {
            Com_SendCommandUseDefaultFrictionTable();
            timerSerialTimeout->start(COMTIMEOUT);
        }
        else if(numFrameToAsynchroSend == Host_FTAS_PidParam)
        {
            Com_SendPidParam();
            timerSerialTimeout->start(COMTIMEOUT);
        }
        else if(numFrameToAsynchroSend == Host_FTAS_PidParamUseDefault)
        {
            Com_SendCommandUseDefaultPidParam();
            timerSerialTimeout->start(COMTIMEOUT);
        }
        else if(numFrameToAsynchroSend == Host_FTAS_ArmModel)
        {
            Com_SendArmModel();
            timerSerialTimeout->start(COMTIMEOUT);
        }
        else if(numFrameToAsynchroSend == Host_FTAS_ArmModelUseDefault)
        {
            Com_SendCommandUseDefaultArmModel();
            timerSerialTimeout->start(COMTIMEOUT);
        }
        else if(numFrameToAsynchroSend == Host_FTAS_TrajSetExecStatus)
        {
            Com_SendCommandTrajSetExecStatus();
            timerSerialTimeout->start(COMTIMEOUT);
        }
        numFrameToAsynchroSend = Host_FTAS_Null;
    }
    else if(comSynchroTransmisionEnable == true && comAsynchronicSend == false)
    {
//        Com_SendFrameGet(Host_FT_JtcStatus);
//        if(numFrameToSynchroSend == Host_FTSS_GetJtcStatus)
//        {
//            Com_SendFrameGet(Host_FT_JtcStatus);
//        }
//        numFrameToSynchroSend++;
//        if(numFrameToSynchroSend >= COMFRAMETOSYNCHROSENDMAX)
//            numFrameToSynchroSend = 0;
    }
}
void MainWindow::Com_Read()
{
    comReadString += serial->readAll();
    if(comReadString.length() < 4)
        return;
    if(comReadString.length() > COMBUFREADMAX)
    {
        comReadString.clear();
        return;
    }
    uint32_t idx0 = comReadString.indexOf(Host_FT_Header);
    if(idx0 > (COMBUFREADMAX-4))
    {
        comReadString.clear();
        return;
    }
    uint8_t comReadBuf[COMBUFREADMAX];
    comReadBuf[0] = static_cast<uint8_t>(comReadString[idx0 + 0]);
    comReadBuf[1] = static_cast<uint8_t>(comReadString[idx0 + 1]);
    comReadBuf[2] = static_cast<uint8_t>(comReadString[idx0 + 2]);
    comReadBuf[3] = static_cast<uint8_t>(comReadString[idx0 + 3]);
    uint16_t nd = ((uint16_t)comReadBuf[2] << 8) + ((uint16_t)comReadBuf[3] << 0);
    if(comReadString.length() < (idx0 + nd))
        return;

    if(nd < 4)
        return;

    if(nd > COMBUFREADMAX)
    {
        comReadString.clear();
        return;
    }
    for(uint32_t i=0;i<nd;i++)
        comReadBuf[i] = static_cast<uint8_t>(comReadString[idx0 + i]);

    Com_ReadFrame(comReadBuf);
    QString str = "Response from JTC: ";
    for(uint32_t i=0;i<comReadString.length();i++)
    {
        str += QString::number(comReadBuf[i]);
        str += " ";
    }
    ui->Com_ReadTextEdit->append(str);
    comReadString.remove(0, idx0+nd);
    if(comReadString.length() > 4)
        Com_Read();
}
void MainWindow::Com_RefreshSerialPort()
{
    serial->close();
    serial->open(QIODevice::ReadWrite);
}
void MainWindow::on_Com_OpenButt_clicked()
{
    serial->setPortName(ui->Com_NameComboBox->currentText());
    serial->setBaudRate(QSerialPort::Baud115200);
    serial->setDataBits(QSerialPort::Data8);
    serial->setParity(QSerialPort::NoParity);
    serial->setStopBits(QSerialPort::OneStop);
    if(!(serial->isOpen()))
    {
        serial->open(QIODevice::ReadWrite);
        if(serial->isOpen())
        {
            statusBar()->showMessage("The COM port has been opened.", 2000);
            timerSerialSend->start(COMTIMESEND);
            timerSerialRead->start(COMTIMEREAD);
        }
        else
        {
            statusBar()->showMessage("The COM port has not been opened.", 2000);
        }
    }
    else
    {
        statusBar()->showMessage("The COM port is already open.", 2000);
    }
}
void MainWindow::on_Com_CloseButt_clicked()
{
    timerSerialSend->stop();
    serial->close();
    statusBar()->showMessage("The COM port has been closed.", 2000);
}
void MainWindow::on_Com_ReadTrajectoryInt16_clicked()
{
    traj.ReadTrajectoryInt16();
    ui->Com_TrajTextEdit->setText(traj.trajString);
    ShowTrajectory();
}
void MainWindow::on_Com_SendTrajectoryToJtc_clicked()
{
    if(numFrameToAsynchroSend == Host_FTAS_Null)
    {
        traj.PrepareTrajectoryToSend();
        comAsynchronicSend = true;
        numFrameToAsynchroSend = Host_FTAS_Trajectory;
    }
}
void MainWindow::on_Com_ReadFrictionTableFloat_clicked()
{
    for(uint8_t i=0;i<JOINTS_MAX;i++)
        joints[i].ReadFrictionTableFloat();
    ui->Com_FrictionTableTextEdit->setText(joints[0].fricTableString);
    ShowFrictionTable(0);
}
void MainWindow::on_Com_ReadArmModel_clicked()
{
}
void MainWindow::ShowPidParametersTable()
{
    pidParametersStandardItemModel->clear();
    ui->Com_PidParametersTableView->setModel(pidParametersStandardItemModel);
    int rows;
    QList<QList<QStandardItem*>> itemTable;
    QStringList columnHeader = {"Kp","Ki","Kd","Error Int Min","Error Int Max"};

    pidParametersStandardItemModel->insertColumns(0, columnHeader.length());
    for(int i=0;i<columnHeader.length();i++)
    {
        pidParametersStandardItemModel->setHeaderData(i, Qt::Horizontal, columnHeader[i]);
    }

    for(int i=0;i<JOINTS_MAX;i++)
    {
        rows = pidParametersStandardItemModel->rowCount();
        pidParametersStandardItemModel->insertRow(rows);
        QString rowHeaderStr = "Joint " + QString::number(i);
        pidParametersStandardItemModel->setHeaderData(i, Qt::Vertical, rowHeaderStr);


        QList<QStandardItem*> itemList;
        itemTable.append(itemList);

        for(int j=0;j<joints[i].pidParamTableValue.length();j++)
        {
            QStandardItem* item = new QStandardItem;
            item->setText(QString::number(joints[i].pidParamTableValue[j],'f',4));
            item->setTextAlignment(Qt::AlignRight);
            itemList.append(item);
            pidParametersStandardItemModel->setItem(i, j, itemList[j]);
        }
    }
    ui->Com_PidParametersTableView->show();
    ui->Com_PidParametersTableView->resizeColumnsToContents();
}
void MainWindow::on_Com_ReadPidParam_clicked()
{
    ui->Com_PidParametersTextEdit->clear();
    for(uint8_t i=0;i<JOINTS_MAX;i++)
    {
        joints[i].ReadPidParametersFloat();
        ui->Com_PidParametersTextEdit->append(joints[i].pidParamString);
    }
    ShowPidParametersTable();
}
void MainWindow::on_Com_ReadTextEdit_2_clicked()
{
    ui->Com_ReadTextEdit->clear();
}
void MainWindow::on_Com_ReadDebugTextedit_clicked()
{
    ui->Com_DebugTextEdit->clear();
}
void MainWindow::on_Com_SendPidParamlToJtc_clicked()
{
    if(numFrameToAsynchroSend == Host_FTAS_Null)
    {
        comAsynchronicSend = true;
        numFrameToAsynchroSend = Host_FTAS_PidParam;
    }
}
void MainWindow::on_Com_SendCommandUseDefaultPidParamlToJtc_clicked()
{
    if(numFrameToAsynchroSend == Host_FTAS_Null)
    {
        comAsynchronicSend = true;
        numFrameToAsynchroSend = Host_FTAS_PidParamUseDefault;
    }
}
void MainWindow::on_Com_SendFrictionTableToJtc_clicked()
{
    if(numFrameToAsynchroSend == Host_FTAS_Null)
    {
        comAsynchronicSend = true;
        numFrameToAsynchroSend = Host_FTAS_FrictionTable;
    }
}
void MainWindow::on_Com_SendCommanUsDefaultFrictionTableToJtc_clicked()
{
    if(numFrameToAsynchroSend == Host_FTAS_Null)
    {
        comAsynchronicSend = true;
        numFrameToAsynchroSend = Host_FTAS_FrictionTableUseDefault;
    }
}
void MainWindow::on_Com_SendArmModelToJtc_clicked()
{
    if(numFrameToAsynchroSend == Host_FTAS_Null)
    {
        comAsynchronicSend = true;
        numFrameToAsynchroSend = Host_FTAS_ArmModel;
    }
}
void MainWindow::on_Com_SendCommandUseDefaultArmModelToJtc_clicked()
{
    if(numFrameToAsynchroSend == Host_FTAS_Null)
    {
        comAsynchronicSend = true;
        numFrameToAsynchroSend = Host_FTAS_ArmModelUseDefault;
    }
}
void MainWindow::Com_SendPidParam()
{
    ui->Com_DebugTextEdit->append("Rozpoczynam wysylanie nastaw regulatorów PID dla wszystkich jointów");
    pidParamWriteString.clear();
    pidParamWriteString.append(Host_FT_Header);
    pidParamWriteString.append(Host_FT_PidParam);
    for(uint32_t i=0;i<JOINTS_MAX;i++)
    {
        joints[i].PreparePidParamToSend();
        pidParamWriteString += joints[i].pidParamStrToSend;
    }
    uint16_t nd = pidParamWriteString.length() + 4;
    pidParamWriteString.insert(2, (uint8_t)(nd >> 8));
    pidParamWriteString.insert(3, (uint8_t)(nd >> 0));
    uint16_t crc = Com_Crc16v2(pidParamWriteString, pidParamWriteString.length());
    pidParamWriteString.append(crc >> 8);
    pidParamWriteString.append(crc >> 0);
    serial->write(pidParamWriteString, pidParamWriteString.length());
    ui->Com_DebugTextEdit->append("Wysłano nastawy regulatorów PID dla wszystkich jointów");
    ui->Com_DebugTextEdit->append("Waiting for response");
}
void MainWindow::Com_SendCommandUseDefaultPidParam()
{
    ui->Com_DebugTextEdit->append("Rozpoczynam wysylanie komendy użycia domyślnych nastaw PID dla wszystkich jointów");
    comWriteString.clear();
    comWriteString.append(Host_FT_Header);
    comWriteString.append(Host_FT_PidParamUseDefault);
    uint16_t nd = comWriteString.length() + 4;
    comWriteString.insert(2, (uint8_t)(nd >> 8));
    comWriteString.insert(3, (uint8_t)(nd >> 0));
    uint16_t crc = Com_Crc16v2(comWriteString, comWriteString.length());
    comWriteString.append(crc >> 8);
    comWriteString.append(crc >> 0);
    serial->write(comWriteString, comWriteString.length());
    ui->Com_DebugTextEdit->append("Wysłano komendę użycia domyślnych nastaw PID dla wszystkich jointów");
    ui->Com_DebugTextEdit->append("Waiting for response");
}
void MainWindow::Com_SendFrictionTable()
{
    ui->Com_DebugTextEdit->append("Rozpoczynam wysylanie tablicy współczynników kompensacyjnych tarcia dla wszystkich jointów");
    fricTableWriteString.clear();
    fricTableWriteString.append(Host_FT_Header);
    fricTableWriteString.append(Host_FT_FrictionTable);
    for(uint32_t i=0;i<JOINTS_MAX;i++)
    {
        joints[i].PrepareFrictionTableToSend();
        fricTableWriteString += joints[i].fricTableStrToSend;
    }
    uint16_t nd = fricTableWriteString.length() + 4;
    fricTableWriteString.insert(2, (uint8_t)(nd >> 8));
    fricTableWriteString.insert(3, (uint8_t)(nd >> 0));
    uint16_t crc = Com_Crc16v2(fricTableWriteString, fricTableWriteString.length());
    fricTableWriteString.append(crc >> 8);
    fricTableWriteString.append(crc >> 0);
    serial->write(fricTableWriteString, fricTableWriteString.length());
    serial->waitForBytesWritten(5000);
    ui->Com_DebugTextEdit->append("Wysłano tablice współczynników kompensacyjnych tarcia dla wszystkich jointów");
    ui->Com_DebugTextEdit->append("Waiting for response");
}
void MainWindow::Com_SendCommandUseDefaultFrictionTable()
{
    ui->Com_DebugTextEdit->append("Rozpoczynam wysylanie komendy użycia domyślnych tablic kompensacji tarcia dla wszystkich jointów");
    comWriteString.clear();
    comWriteString.append(Host_FT_Header);
    comWriteString.append(Host_FT_FrictionTableUseDefault);
    uint16_t nd = comWriteString.length() + 4;
    comWriteString.insert(2, (uint8_t)(nd >> 8));
    comWriteString.insert(3, (uint8_t)(nd >> 0));
    uint16_t crc = Com_Crc16v2(comWriteString, comWriteString.length());
    comWriteString.append(crc >> 8);
    comWriteString.append(crc >> 0);
    serial->write(comWriteString, comWriteString.length());
    serial->waitForBytesWritten(1000);
    ui->Com_DebugTextEdit->append("Wysłano komendę użycia domyślnych tablic kompensacji tarcia dla wszystkich jointów");
    ui->Com_DebugTextEdit->append("Waiting for response");
}
void MainWindow::Com_SendArmModel()
{
    ui->Com_DebugTextEdit->append("Rozpoczynam wysylanie modelu manipulatora");
    armModelWriteString.clear();
    armModelWriteString.append(Host_FT_Header);
    armModelWriteString.append(Host_FT_ArmModel);
    Com_PrepareArmModelToSend();
    uint16_t nd = armModelWriteString.length() + 4;
    armModelWriteString.insert(2, (uint8_t)(nd >> 8));
    armModelWriteString.insert(3, (uint8_t)(nd >> 0));
    uint16_t crc = Com_Crc16v2(armModelWriteString, armModelWriteString.length());
    armModelWriteString.append(crc >> 8);
    armModelWriteString.append(crc >> 0);
    serial->write(armModelWriteString, armModelWriteString.length());
    serial->waitForBytesWritten(5000);
    ui->Com_DebugTextEdit->append("Wysłano model manipulatora");
    ui->Com_DebugTextEdit->append("Waiting for response");
}
void MainWindow::Com_Timeout()
{
    timerSerialSend->stop();
    timerSerialTimeout->stop();
    Com_RefreshSerialPort();
    comAsynchronicSend = false;
    numFrameToSynchroSend = 0;
    numFrameToAsynchroSend = Host_FTAS_Null;
    comSynchroTransmisionEnable = true;
    timerSerialSend->start(COMTIMESEND);
    ui->Com_DebugTextEdit->append("Error!!! Timeout!!! Restart serialport.");
}
void MainWindow::Com_SendCommandUseDefaultArmModel()
{
    ui->Com_DebugTextEdit->append("Rozpoczynam wysylanie komendy użycia domyślnych wartościmodelu manipulatora");
    comWriteString.clear();
    comWriteString.append(Host_FT_Header);
    comWriteString.append(Host_FT_ArmModelUseDefault);
    uint16_t nd = comWriteString.length() + 4;
    comWriteString.insert(2, (uint8_t)(nd >> 8));
    comWriteString.insert(3, (uint8_t)(nd >> 0));
    uint16_t crc = Com_Crc16v2(comWriteString, comWriteString.length());
    comWriteString.append(crc >> 8);
    comWriteString.append(crc >> 0);
    serial->write(comWriteString, comWriteString.length());
    serial->waitForBytesWritten(1000);
    ui->Com_DebugTextEdit->append("Wysłano komendę użycia domyślnych wartościmodelu manipulatora");
    ui->Com_DebugTextEdit->append("Waiting for response");
}
void MainWindow::Com_SendCommandTrajSetExecStatus()
{
    QString val;
    if(traj.targetStatus == TES_Stop)  val = "Stop";
    if(traj.targetStatus == TES_Pause)  val = "Pause";
    if(traj.targetStatus == TES_Execute)  val = "Execute";
    ui->Com_DebugTextEdit->append("Rozpoczynam wysylanie komendy zmiany trybu wykonywania trajektorii na: " + val);
    comWriteString.clear();
    comWriteString.append(Host_FT_Header);
    comWriteString.append(Host_FT_TrajSetExecStatus);
    comWriteString.append(traj.targetStatus);
    uint16_t nd = comWriteString.length() + 4;
    comWriteString.insert(2, (uint8_t)(nd >> 8));
    comWriteString.insert(3, (uint8_t)(nd >> 0));
    uint16_t crc = Com_Crc16v2(comWriteString, comWriteString.length());
    comWriteString.append(crc >> 8);
    comWriteString.append(crc >> 0);
    serial->write(comWriteString, comWriteString.length());
    ui->Com_DebugTextEdit->append("Wysłano komendę zmiany trybu wykonywania trajektorii na: " + val);
    ui->Com_DebugTextEdit->append("Waiting for response");
}
void MainWindow::on_Traj_TrajectoryStop_clicked()
{
    if(numFrameToAsynchroSend == Host_FTAS_Null)
    {
        traj.targetStatus = TES_Stop;
        comAsynchronicSend = true;
        numFrameToAsynchroSend = Host_FTAS_TrajSetExecStatus;
    }
}
void MainWindow::on_Traj_TrajectoryPause_clicked()
{
    if(numFrameToAsynchroSend == Host_FTAS_Null)
    {
        traj.targetStatus = TES_Pause;
        comAsynchronicSend = true;
        numFrameToAsynchroSend = Host_FTAS_TrajSetExecStatus;
    }
}
void MainWindow::on_Traj_TrajectoryExecute_clicked()
{
    if(numFrameToAsynchroSend == Host_FTAS_Null)
    {
        traj.targetStatus = TES_Execute;
        comAsynchronicSend = true;
        numFrameToAsynchroSend = Host_FTAS_TrajSetExecStatus;
    }
}
void MainWindow::on_Com_OnOffSynchroTransmision_clicked()
{
    if(comSynchroTransmisionEnable == true)
    {
        comSynchroTransmisionEnable = false;
    }
    else
    {
        comSynchroTransmisionEnable = true;
    }
}
void MainWindow::on_JTC_ClearErrors_clicked()
{
    if(numFrameToAsynchroSend == Host_FTAS_Null)
    {
        comAsynchronicSend = true;
        numFrameToAsynchroSend = Host_FTAS_ClearCurrentErrors;
    }
}
void MainWindow::on_JTC_ClearOccuredErrors_clicked()
{
    if(numFrameToAsynchroSend == Host_FTAS_Null)
    {
        comAsynchronicSend = true;
        numFrameToAsynchroSend = Host_FTAS_ClearOccuredErrors;
    }
}

