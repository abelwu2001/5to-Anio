#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSerialPort>
#include <QTimer>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_btnConnect_clicked();
    void on_cbMode_currentIndexChanged(int idx);
    void onBaudRateChanged(const QString &baud);
    void readSerialData();
    void onPollTimeout();
    void handleError(QSerialPort::SerialPortError err);
    void on_chkOut1_toggled(bool);
    void on_chkOut2_toggled(bool);
    void on_chkOut3_toggled(bool);
    void on_sldPWM1_valueChanged(int value);
    void on_sldPWM2_valueChanged(int value);

private:
    enum class CommMode { RS232, RS485 };
    CommMode mode;

    Ui::MainWindow *ui;
    QSerialPort   *serial;
    QTimer        *pollTimer;
    QByteArray     recvBuffer;

    void processFrame(const QByteArray &frame);
    quint8 computeCRC8(const QByteArray &data);
    void sendControlFrame();
};

#endif // MAINWINDOW_H
