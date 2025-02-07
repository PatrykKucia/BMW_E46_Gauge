#include "mainwindow.h"
#include "ui_mainwindow.h"

//User includes

#include <QDebug>
#include <QCanBus>
#include <QUdpSocket>

#pragma pack(push, 1) // Zapewnia brak optymalizacji struktury (ważne dla odbioru pakietów UDP)
struct TelemetryData {
    unsigned       time;
    char           car[4];
    unsigned short flags;
    char           gear;
    char           plid;
    float          speed;
    float          rpm;
    float          turbo;
    float          engTemp;
    float          fuel;
    float          oilPressure;
    float          oilTemp;
    unsigned       dashLights;
    unsigned       showLights;
    float          throttle;
    float          brake;
    float          clutch;
    char           display1[16];
    char           display2[16];
    int            id;
};
#pragma pack(pop)

// OG_x - Bity dla flags
#define OG_SHIFT    (1 << 0)   // Klawisz SHIFT
#define OG_CTRL     (1 << 1)   // Klawisz CTRL
#define OG_TURBO    (1 << 13)  // Wskaźnik turbo
#define OG_KM       (1 << 14)  // Jeśli nie ustawione - preferuje MILE
#define OG_BAR      (1 << 15)  // Jeśli nie ustawione - preferuje PSI

// DL_x - Bity dla dashLights i showLights
#define DL_SHIFT        (1 << 0)  // Shift light
#define DL_FULLBEAM     (1 << 1)  // Światła drogowe
#define DL_HANDBRAKE    (1 << 2)  // Hamulec ręczny
#define DL_PITSPEED     (1 << 3)  // Ogranicznik prędkości pit (N/A)
#define DL_TC           (1 << 4)  // Kontrola trakcji
#define DL_SIGNAL_L     (1 << 5)  // Kierunkowskaz lewy
#define DL_SIGNAL_R     (1 << 6)  // Kierunkowskaz prawy
#define DL_SIGNAL_ANY   (1 << 7)  // Kierunkowskaz ogólny (N/A)
#define DL_OILWARN      (1 << 8)  // Ostrzeżenie o niskim ciśnieniu oleju
#define DL_BATTERY      (1 << 9)  // Ostrzeżenie o akumulatorze
#define DL_ABS          (1 << 10) // ABS aktywny lub wyłączony
#define DL_SPARE        (1 << 11) // N/A

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

    // Inicjalizacja gniazda UDP
    udpSocket = new QUdpSocket(this);
    udpSocket->bind(QHostAddress::Any, 4444);  // Port 4444, akceptujemy wszystkie adresy

    connect(udpSocket, &QUdpSocket::readyRead, this, &MainWindow::readPendingUdpDatagrams);
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

   // modifyCanFrameByte(frame316Data, 2, lsb);
   // modifyCanFrameByte(frame316Data, 3, msb);
}

void MainWindow::readPendingUdpDatagrams()
{
    while (udpSocket->hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(udpSocket->pendingDatagramSize());
        udpSocket->readDatagram(datagram.data(), datagram.size());

        // Przetwarzanie otrzymanych danych
        processUdpData(datagram);
    }
}

void MainWindow::processUdpData(QByteArray &datagram)
{
    if (datagram.size() != sizeof(TelemetryData)) {
        qDebug() << "Otrzymano błędny pakiet UDP, rozmiar: " << datagram.size();
        return;
    }

    // Konwersja QByteArray na strukturę TelemetryData
    TelemetryData telemetry;
    memcpy(&telemetry, datagram.data(), sizeof(TelemetryData));

    // Debug - wypisanie odebranych wartości
    qDebug() << "Speed: " << telemetry.speed << " m/s";
    qDebug() << "RPM: " << telemetry.rpm;
    qDebug() << "Throttle: " << telemetry.throttle;
    qDebug() << "Brake: " << telemetry.brake;
    qDebug() << "Clutch: " << telemetry.clutch;

    // Możesz zaktualizować UI, np. ustawić wskaźniki na podstawie wartości
    ui->labelSpeed->setText(QString("Speed: %1 m/s").arg(telemetry.speed));
    ui->labelRPM->setText(QString("RPM: %1").arg(telemetry.rpm));

    uint16_t hexValue = static_cast<uint16_t>(telemetry.rpm / 0.15625);
    uint8_t lsb = hexValue & 0xFF;
    uint8_t msb = (hexValue >> 8) & 0xFF;

    modifyCanFrameByte(frame316Data, 2, lsb);
    modifyCanFrameByte(frame316Data, 3, msb);


    // Przykłady obsługi flag i świateł
    bool isTurboActive = telemetry.flags & OG_TURBO;
    bool isMetric = telemetry.flags & OG_KM;
    bool prefersBar = telemetry.flags & OG_BAR;

    bool isShiftLightOn = telemetry.showLights & DL_SHIFT;
    bool isFullBeam = telemetry.showLights & DL_FULLBEAM;
    bool isHandbrakeOn = telemetry.showLights & DL_HANDBRAKE;
    bool isTractionCtrl = telemetry.showLights & DL_TC;
    bool isABSActive = telemetry.showLights & DL_ABS;
    bool isOilWarning = telemetry.showLights & DL_OILWARN;
    bool isBatteryWarning = telemetry.showLights & DL_BATTERY;
    bool isLeftSignal = telemetry.showLights & DL_SIGNAL_L;
    bool isRightSignal = telemetry.showLights & DL_SIGNAL_R;

   // Debug - wypisanie wartości
   // qDebug() << "Turbo Aktywne:" << isTurboActive;
   // qDebug() << "Preferuje km/h:" << isMetric;
   // qDebug() << "Preferuje BAR:" << prefersBar;
   // qDebug() << "Shift Light On:" << isShiftLightOn;
   // qDebug() << "Handbrake On:" << isHandbrakeOn;
   // qDebug() << "Beam On:" << isFullBeam;
   // qDebug() << "ABS Active:" << isABSActive;
   // qDebug() << "Oil Warning:" << isOilWarning;
   //qDebug() << "Left Signal:" << isLeftSignal;
   // qDebug() << "Right Signal:" << isRightSignal;
    qDebug() << "ramka" << telemetry.dashLights;

    //ui->lcdNumberTime->display(static_cast<int>(telemetry.time));
    //ui->lcdNumberSpeed->display(static_cast<int>(telemetry.car));
    ui->lcdNumberGear->display(static_cast<double>(telemetry.gear));
    ui->lcdNumberSpeed->display(static_cast<double>(telemetry.speed));
    ui->lcdNumberRPM->display(static_cast<double>(telemetry.rpm));
    ui->lcdNumberTurbo->display(static_cast<double>(telemetry.turbo));
    ui->lcdNumberEnginetemp->display(static_cast<double>(telemetry.engTemp));
    ui->lcdNumberFuel->display(static_cast<double>(telemetry.fuel));
    // ui->lcdNumberOilPresure->display(static_cast<double>(telemetry.oilPressure));
    ui->lcdNumberOilTemp->display(static_cast<double>(telemetry.oilTemp));
    ui->lcdNumberDashlights->display(static_cast<double>(telemetry.dashLights));
    ui->lcdNumberShowlights->display(static_cast<double>(telemetry.showLights));
    ui->lcdNumberThrotrle->display(static_cast<double>(telemetry.throttle));
    ui->lcdNumberBrake->display(static_cast<double>(telemetry.brake));
    ui->lcdNumberClutch->display(static_cast<double>(telemetry.clutch));


    // Aktualizacja UI
    ui->Turbo_ValueLabel->setText(isTurboActive ? "ON" : "OFF");
    ui->KM_ValueLabel->setText(QString("%1 %2").arg(isMetric ? "km/h" : "mph"));
    ui->BAR_ValueLabel->setText(prefersBar ? "ON" : "OFF");
    ui->ShiftLight_ValueLabel->setText(isShiftLightOn ? "ON" : "OFF");
    ui->FullBeam_ValueLabel->setText(isFullBeam ? "ON" : "OFF");
    ui->HandBrake_ValueLabel->setText(isHandbrakeOn ? "ON" : "OFF");
    ui->TractionCtrl_ValueLabel->setText(isTractionCtrl ? "ON" : "OFF");
    ui->LeftTurnSignal_ValueLabel->setText(isLeftSignal ? "ON" : "OFF");
    ui->RightTurnSignal_ValueLabel->setText(isRightSignal ? "ON" : "OFF");
    ui->OilPresureWarning_ValueLabel->setText(isOilWarning ? "ON" : "OFF");
    ui->BatteryWarning_ValueLabel->setText(isBatteryWarning ? "ON" : "OFF");
    ui->ABS_ValueLabel->setText(isABSActive ? "ON" : "OFF");
}

