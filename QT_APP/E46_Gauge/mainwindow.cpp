#include "mainwindow.h"
#include "ui_mainwindow.h"

//User includes

#include <QDebug>
#include <QCanBus>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // Timer init
    timer10ms = new QTimer(this);
    timer1000ms = new QTimer(this);

    //Default frame data

    const uint8_t data[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    // Byte array converted to QByteArray
    QByteArray payload(reinterpret_cast<const char*>(data), sizeof(data));

    frame1Data = payload;
    frame2Data = payload;
    frame545Data = payload;

    // Connect

    connect(timer10ms, &QTimer::timeout, this, &MainWindow::sendCanMessage10ms);
    connect(timer1000ms, &QTimer::timeout, this, &MainWindow::sendCanMessage1000ms);
}

MainWindow::~MainWindow()
{
    if (device) {
        device->disconnectDevice();
        delete device;
    }
    delete ui;
}

void MainWindow::on_Send_clicked()
{
    if (!device) {
        ui->textEditLog->append("No CAN connection first click - 'Connect'.");
        return;
    }

    if (!isSending) {
        timer10ms->start(10);   // timer start 10ms
        timer1000ms->start(1000); // timer start 1000ms
        isSending = true;
        ui->textEditLog->append("CAN cyclic sending started.");
        ui->Send->setText("Stop");
    } else {
        timer10ms->stop();   // timer 10ms stop
        timer1000ms->stop(); // timer 1000ms stop
        isSending = false;
        ui->textEditLog->append("CAN sending stopped.");
        ui->Send->setText("Start");
    }
}


void MainWindow::on_pushButtonConnect_clicked()
{
    if (device) {
        ui->textEditLog->append("Device already connected.");
        return;
    }

    QString errorString;
    device = QCanBus::instance()->createDevice(QStringLiteral("peakcan"), QStringLiteral("usb0"), &errorString);
    if (!device) {
        ui->textEditLog->append("Error: " + errorString);
        return; // finish if error
    }

    if (!device->connectDevice()) {
        ui->textEditLog->append("Connecting to CAN device failed.");
        delete device; // free memory
        device = nullptr;
    } else {
        ui->textEditLog->append("CAN device connected");
    }
}

void MainWindow::sendCanMessage10ms()
{
    if (!device || device->state() != QCanBusDevice::ConnectedState) {
        return;
    }

    QCanBusFrame frame1;
    frame1.setFrameId(0x316);
    frame1.setPayload(frame1Data);
    device->writeFrame(frame1);

    QCanBusFrame frame2;
    frame2.setFrameId(0x329);
    frame2.setPayload(frame2Data);
    device->writeFrame(frame2);

    QCanBusFrame frame545;
    frame545.setFrameId(0x545);
    frame545.setPayload(frame545Data);
    device->writeFrame(frame545);
}

void MainWindow::sendCanMessage1000ms()
{
    if (!device || device->state() != QCanBusDevice::ConnectedState) {
        return;
    }

}

void MainWindow::on_textEditLog_copyAvailable(bool b)
{

}



void MainWindow::on_pushButtonEngineCheck_toggled(bool checked)
{
    if(checked)
    {    const uint8_t data[] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

        // Byte array converted to QByteArray
        QByteArray payload(reinterpret_cast<const char*>(data), sizeof(data));

        // Frame 545 load
        frame545Data = payload;

        // print in log
        ui->textEditLog->append("Frame 545 modified: " + frame545Data.toHex());
    }
    else
    {    const uint8_t data[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

        // Byte array converted to QByteArray
        QByteArray payload(reinterpret_cast<const char*>(data), sizeof(data));

        // Frame 545 load
        frame545Data = payload;

        // print in log
        ui->textEditLog->append("Frame 545 modified: " + frame545Data.toHex());
    }

}

