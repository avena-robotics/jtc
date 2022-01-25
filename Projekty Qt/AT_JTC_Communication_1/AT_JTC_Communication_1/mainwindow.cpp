#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent): QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    foreach (const QSerialPortInfo &serialPortInfo, QSerialPortInfo::availablePorts())
        ui->Com_NameComboBox->addItem(serialPortInfo.portName());
    connect(Com.serial, SIGNAL(readyRead()), this, SLOT(ShowReadedString()));
    trajStandardItemModel = new QStandardItemModel(this);
    frictionTableStandardItemModel = new QStandardItemModel(this);
    on_Com_ReadTrajectory_clicked();
    on_Com_ReadFrictionTableFloat_clicked();
    timerSerial = new QTimer(this);
    connect(timerSerial, SIGNAL(timeout()), this, SLOT(ShowReadedString()));

}
MainWindow::~MainWindow()
{
    delete ui;
}
void MainWindow::ShowReadedString()
{
    Com.readStr += Com.serial->readAll();
    if(Com.readStr.length() >= 4)
    {
        uint8_t buf[1000];
        for(uint32_t i=0;i<Com.readStr.size();i++)
        {
            buf[i] = static_cast<uint8_t>(Com.readStr[i]);
        }
        uint16_t nd = ((uint16_t)buf[2] << 8);
        nd += ((uint16_t)buf[3] << 0);

        if(Com.readStr.length() >= nd)
        {
            Com.str.clear();
            for(uint32_t i=0;i<nd;i++)
            {
                Com.str += QString::number(buf[i]);
                Com.str += " ";
            }
            ui->Com_ReadTextEdit->append(Com.str);
            Com.readStr.clear();
            Com.Com_ReadFrame(buf);
        }
    }
}
void MainWindow::on_Com_OpenButt_clicked()
{
    if(Com.Com_OpenPort(ui->Com_NameComboBox->currentText()) == true)
    {
        timerSerial->start(100);
        statusBar()->showMessage("Port is opened", 2000);
    }
    else
        statusBar()->showMessage("Error", 2000);

}
void MainWindow::on_Com_CloseButt_clicked()
{
    Com.Com_ClosePort();
    timerSerial->stop();
}
void MainWindow::on_Com_ReadTrajectory_clicked()
{
    Com.Com_ReadTrajInt16();
    ui->Com_TrajTextEdit->setText(Com.trajectory.trajString);
    ShowTrajectory();

}
void MainWindow::on_Com_ReadTrajectoryFloat_clicked()
{
    Com.Com_ReadTrajFloat();
    ui->Com_TrajTextEdit->setText(Com.trajectory.trajString);
    ShowTrajectory();
}
void MainWindow::ShowTrajectory()
{
    ui->Com_TrajTableView->setModel(trajStandardItemModel);
    int rows;
    QList<QList<QStandardItem*>> itemTable;
    QStringList strList = { "Time",
                            "J 0 Pos","J 1 Pos","J 2 Pos","J 3 Pos","J 4 Pos","J 5 Pos","J 6 Pos",
                            "J 0 Vel","J 1 Vel","J 2 Vel","J 3 Vel","J 4 Vel","J 5 Vel","J 6 Vel",
                            "J 0 Acc","J 1 Acc","J 2 Acc","J 3 Acc","J 4 Acc","J 5 Acc","J 6 Acc"};

    trajStandardItemModel->insertColumns(0, strList.length());
    for(int i=0;i<strList.length();i++)
    {
        trajStandardItemModel->setHeaderData(i, Qt::Horizontal, strList[i]);
    }

    for(int i=0;i<Com.trajectory.value.length();i++)
    {
        rows = trajStandardItemModel->rowCount();
        trajStandardItemModel->insertRow(rows);

        QList<QStandardItem*> itemList;
        itemTable.append(itemList);
        for(int j=0;j<Com.trajectory.value[i].length();j++)
        {
            QStandardItem* item = new QStandardItem;
            item->setText(QString::number(Com.trajectory.value[i][j],'f',6));
            item->setTextAlignment(Qt::AlignRight);
            itemList.append(item);
            trajStandardItemModel->setItem(i, j, itemList[j]);
        }
    }

    ui->Com_TrajTableView->show();
    ui->Com_TrajTableView->resizeColumnsToContents();
}
void MainWindow::on_Com_SendTrajectoryToJtc_clicked()
{
    Com.Com_PrepareTrajectoryToSend();
    Com.Com_SendTrajectory();
    if (Com.serial->waitForBytesWritten(5000))
    {
        ui->statusbar->showMessage("Serial Port sent", 2000);
    }
    else
    {
        ui->statusbar->showMessage("Serial Port timeout", 2000);
    }
}
void MainWindow::on_Com_ReadFrictionTableFloat_clicked()
{
    Com.Com_ReadFricTableFloat();
    ui->Com_FrictionTableTextEdit->setText(Com.frictTableString);
    ShowFrictionTable();
}
void MainWindow::ShowFrictionTable()
{
    ui->Com_FrictionTableTableView->setModel(frictionTableStandardItemModel);
    int rows;
    QList<QList<QStandardItem*>> itemTable;
    QStringList strList = {"Torque","Torque","Torque","Torque","Torque","Torque","Torque","Torque","Torque","Torque",
                          "Torque","Torque","Torque","Torque","Torque","Torque","Torque","Torque","Torque","Torque"};

    frictionTableStandardItemModel->insertColumns(0, strList.length());
    for(int i=0;i<strList.length();i++)
    {
        frictionTableStandardItemModel->setHeaderData(i, Qt::Horizontal, strList[i]);
    }

    for(int i=0;i<Com.frictableValue.length();i++)
    {
        rows = frictionTableStandardItemModel->rowCount();
        frictionTableStandardItemModel->insertRow(rows);

        QList<QStandardItem*> itemList;
        itemTable.append(itemList);
        for(int j=0;j<Com.frictableValue[i].length();j++)
        {
            QStandardItem* item = new QStandardItem;
            item->setText(QString::number(Com.frictableValue[i][j],'f',4));
            item->setTextAlignment(Qt::AlignRight);
            itemList.append(item);
            frictionTableStandardItemModel->setItem(i, j, itemList[j]);
        }
    }

    ui->Com_FrictionTableTableView->show();
    ui->Com_FrictionTableTableView->resizeColumnsToContents();
}
void MainWindow::on_Com_SendFrictionTableToJtc_clicked()
{
    Com.Com_PrepareFrictionTableToSend(JOINTS_MAX);
    Com.Com_SendFrictionTable();
}
