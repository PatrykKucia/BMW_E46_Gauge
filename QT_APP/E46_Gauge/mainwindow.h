#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

//User Includes

#include <QCanBus>
#include <QCanBusDevice>
#include <QTimer>

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_pushButtonConnect_clicked();
    void on_textEditLog_copyAvailable(bool b);
    void sendCanMessage10ms();
    void sendCanMessage1000ms();
    void on_Send_clicked();
    void on_pushButtonEngineCheck_clicked();



    void on_Send_toggled(bool checked);

    void on_pushButtonEngineCheck_toggled(bool checked);

    void on_pushButtonDDE_clicked();

    void on_pushButtonDDE_toggled(bool checked);

    void modifyCanFrameBit(QByteArray &frame, uint8_t byteIndex, uint8_t bit, bool state);
    void modifyCanFrameByte(QByteArray &frame, uint8_t byteIndex, uint8_t value);

    void on_pushButtonCruise_toggled(bool checked);

    void on_pushButton_toggled(bool checked);

    void on_pushButtonFuelCap_toggled(bool checked);

    void on_horizontalSlider_valueChanged(int value);

    void on_horizontalSliderFuel_valueChanged(int value);

    void on_horizontalSliderCoolantTemp_valueChanged(int value);

    void on_horizontalSliderSpeed_valueChanged(int value);

    void on_horizontalSliderRPM_valueChanged(int value);

private:
    Ui::MainWindow *ui;
    QCanBusDevice *device = nullptr;  // CAN device pointer
    QTimer *timer10ms;  // timer for can frame
    QTimer *timer1000ms;  // timer for can frame

    bool isSending = false; // CAN sending flag

    // Data frames
    QByteArray frame316Data;
    QByteArray frame329Data;
    QByteArray frame545Data;
};
#endif // MAINWINDOW_H
