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

    frame316Data = payload;
    frame329Data = payload;
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

    QCanBusFrame frame316;
    frame316.setFrameId(0x316);
    frame316.setPayload(frame316Data);
    device->writeFrame(frame316);

    QCanBusFrame frame329;
    frame329.setFrameId(0x329);
    frame329.setPayload(frame329Data);
    device->writeFrame(frame329);

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
    modifyCanFrameBit(frame545Data, 0, 1, checked);
}

void MainWindow::on_pushButtonDDE_toggled(bool checked)
{
    modifyCanFrameBit(frame545Data, 0, 4, checked);
}

void MainWindow::on_pushButtonCruise_toggled(bool checked)
{
    modifyCanFrameBit(frame545Data, 0, 3, checked);
}

void MainWindow::on_pushButtonFuelCap_toggled(bool checked)
{
    modifyCanFrameBit(frame545Data, 0, 6, checked);
}


void MainWindow::modifyCanFrameBit(QByteArray &frame, uint8_t byteIndex, uint8_t bit, bool state)
{
    if (byteIndex >= frame.size())  // Sprawdzenie, czy bajt mieści się w ramce
        return;

    if (state)
        frame[byteIndex] |= (1 << bit);  // Ustawienie bitu
    else
        frame[byteIndex] &= ~(1 << bit); // Wyczyszczenie bitu

    // Wypisanie zmodyfikowanej ramki w logu
    ui->textEditLog->append("Frame modified: " + frame.toHex());
}

void MainWindow::modifyCanFrameByte(QByteArray &frame, uint8_t byteIndex, uint8_t value)
{
    if (byteIndex >= frame.size())
        return;

    frame[byteIndex] = value; // Nadpisanie całego bajtu

    ui->textEditLog->append("Frame modified: " + frame.toHex());
}



void MainWindow::on_horizontalSliderFuel_valueChanged(int value)
{
    ui->labelFuelLevel->setText(QString("Fuel Level: %1%").arg(value));
}


void MainWindow::on_horizontalSliderCoolantTemp_valueChanged(int value)
{
    double temperature = (value * 0.75) - 48.0;

    ui->labelCoolantTemp->setText(QString("Temp: %1 °C").arg(temperature, 0, 'f', 1));

    modifyCanFrameByte(frame329Data, 1, value);
}


void MainWindow::on_horizontalSliderSpeed_valueChanged(int value)
{

}


void MainWindow::on_horizontalSliderRPM_valueChanged(int value)
{
    modifyCanFrameBit(frame316Data, 0, 0, true);
    modifyCanFrameBit(frame316Data, 0, 1, false);
    modifyCanFrameBit(frame316Data, 0, 2, true);
    modifyCanFrameBit(frame316Data, 0, 3, true);
    modifyCanFrameBit(frame316Data, 0, 4, false);
    modifyCanFrameBit(frame316Data, 0, 5, false);
    modifyCanFrameBit(frame316Data, 0, 6, false);
    modifyCanFrameBit(frame316Data, 0, 7, false);

    int rpm = value;
    ui->labelRPM->setText(QString("RPM: %1").arg(rpm));

    // RPM to HEX (MSB + LSB)
    uint16_t hexValue = static_cast<uint16_t>(rpm / 0.15625);
    uint8_t lsb = hexValue & 0xFF;
    uint8_t msb = (hexValue >> 8) & 0xFF;

    modifyCanFrameByte(frame316Data, 2, lsb);
    modifyCanFrameByte(frame316Data, 3, msb);
}

