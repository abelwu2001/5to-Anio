#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QSerialPortInfo>
#include <QDebug>
#include <QThread>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , serial(new QSerialPort(this))
    , pollTimer(new QTimer(this))
    , mode(CommMode::RS232)
{
    ui->setupUi(this);

    // Rango 0–1024 en sliders (10 bits)
    ui->sldPWM1->setRange(0, 1024);
    ui->sldPWM2->setRange(0, 1024);

    // Puertos y baudrates
    for (const QSerialPortInfo &p : QSerialPortInfo::availablePorts())
        ui->cbPort->addItem(p.portName());
    ui->cbBaudRate->addItems({ "9600","19200","38400","57600","115200" });

    // Conexiones
    connect(ui->cbMode, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &MainWindow::on_cbMode_currentIndexChanged);
    connect(ui->cbBaudRate, &QComboBox::currentTextChanged,
            this, &MainWindow::onBaudRateChanged);
    connect(serial, &QSerialPort::readyRead, this, &MainWindow::readSerialData);
    connect(serial, &QSerialPort::errorOccurred, this, &MainWindow::handleError);
    connect(pollTimer, &QTimer::timeout, this, &MainWindow::onPollTimeout);

    on_cbMode_currentIndexChanged(0);
}

MainWindow::~MainWindow() {
    if (serial->isOpen()) serial->close();
    delete ui;
}

void MainWindow::on_cbMode_currentIndexChanged(int idx) {
    // Registrar modo pero mantener todos los controles habilitados
    mode = (idx == 0 ? CommMode::RS232 : CommMode::RS485);
    ui->chkOut1->setEnabled(true);
    ui->chkOut2->setEnabled(true);
    ui->chkOut3->setEnabled(true);
    ui->sldPWM1->setEnabled(true);
    ui->sldPWM2->setEnabled(true);
}

void MainWindow::onBaudRateChanged(const QString &baud) {
    int b = baud.toInt();
    if (serial->isOpen()) {
        if (!serial->setBaudRate(b))
            ui->statusbar->showMessage("Error cambiando baudrate", 3000);
        else
            ui->statusbar->showMessage("Baudrate=" + baud, 2000);
    }
}

void MainWindow::on_btnConnect_clicked() {
    if (serial->isOpen()) {
        pollTimer->stop();
        serial->close();
        ui->btnConnect->setText("Conectar");
        ui->statusbar->showMessage("Puerto cerrado", 2000);
    } else {
        serial->setPortName(ui->cbPort->currentText());
        serial->setDataBits(QSerialPort::Data8);
        serial->setParity(QSerialPort::NoParity);
        serial->setStopBits(QSerialPort::OneStop);
        serial->setFlowControl(QSerialPort::NoFlowControl);
        serial->setBaudRate(ui->cbBaudRate->currentText().toInt());

        if (!serial->open(QIODevice::ReadWrite)) {
            ui->statusbar->showMessage("No se pudo abrir puerto", 3000);
            return;
        }
        ui->btnConnect->setText("Desconectar");
        ui->statusbar->showMessage("Conectado a " + serial->portName(), 2000);
        setRs485Direction(false); // iniciar en modo recepción

        sendControlFrame();
        pollTimer->start(1000);
    }
}

void MainWindow::handleError(QSerialPort::SerialPortError err) {
    if (err == QSerialPort::ResourceError) {
        ui->statusbar->showMessage("¡Error crítico! " + serial->errorString(), 5000);
        serial->close(); pollTimer->stop(); ui->btnConnect->setText("Conectar");
    } else {
        ui->statusbar->showMessage(serial->errorString(), 3000);
    }
}

void MainWindow::readSerialData() {
    recvBuffer.append(serial->readAll());
    // Tramas de 13 bytes: 1 cabecera + 10 datos + 2 CRC16
    while (recvBuffer.size() >= 13) {
        if (quint8(recvBuffer[0]) != 0x02) {
            recvBuffer.remove(0,1);
            continue;
        }
        QByteArray frame = recvBuffer.left(13);
        quint16 crc_recv = quint8(frame[11]) | (quint8(frame[12]) << 8);
        if (computeCRC16(frame.left(11)) == crc_recv) {
            processFrame(frame);
            recvBuffer.remove(0,13);
        } else {
            recvBuffer.remove(0,1);
            ui->statusbar->showMessage("CRC Error",2000);
        }
    }
}

void MainWindow::processFrame(const QByteArray &f) {
    quint16 a0 = quint8(f[1]) | (quint8(f[2])<<8);
    quint16 a1 = quint8(f[3]) | (quint8(f[4])<<8);
    quint16 a2 = quint8(f[5]) | (quint8(f[6])<<8);
    quint8  inp = quint8(f[7]);
    qint8   t   = qint8(f[8]);
    quint16 p   = (quint8(f[9]) << 8) | quint8(f[10]);

    ui->lcdADC0->display(a0);
    ui->lcdADC1->display(a1);
    ui->lcdADC2->display(a2);
    ui->lcdTemp->display(t);
    ui->lcdPress->display(p);

    auto setLed=[&](QLabel*L,bool on){
        L->setStyleSheet(on?"background-color:green;":"background-color:red;");
    };
    setLed(ui->ledInput1, inp&1);
    setLed(ui->ledInput2, inp&2);
    setLed(ui->ledInput3, inp&4);

    QString hex;
    for (auto b : f) hex += QString("%1 ").arg(quint8(b),2,16,QLatin1Char('0'));
    ui->txtRawData->append(hex.trimmed().toUpper());
}

quint16 MainWindow::computeCRC16(const QByteArray &data) {
    quint16 crc = 0xFFFF;
    for (quint8 b : data) {
        crc ^= quint16(b);
        for (int i = 0; i < 8; ++i) {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

void MainWindow::setRs485Direction(bool transmit) {
    serial->setRequestToSend(transmit);
    serial->setDataTerminalReady(transmit);
}

void MainWindow::sendControlFrame() {
    if (!serial->isOpen()) return;

    quint16 pwm1 = ui->sldPWM1->value();
    quint16 pwm2 = ui->sldPWM2->value();
    quint8 mask = (ui->chkOut1->isChecked()?0x01:0x00)
                | (ui->chkOut2->isChecked()?0x02:0x00)
                | (ui->chkOut3->isChecked()?0x04:0x00);

    QByteArray f;
    f.append(char(0x02));
    f.append(char(mask));
    f.append(char(pwm1 & 0xFF));
    f.append(char((pwm1 >> 8) & 0xFF));
    f.append(char(pwm2 & 0xFF));
    f.append(char((pwm2 >> 8) & 0xFF));
    quint16 crc = computeCRC16(f);
    f.append(char(crc & 0xFF));
    f.append(char((crc >> 8) & 0xFF));

    if (mode == CommMode::RS485) {
        setRs485Direction(true);
        QThread::msleep(1);
    }
    serial->write(f);
    bool ok = serial->waitForBytesWritten(50);
    if (mode == CommMode::RS485) {
        QThread::msleep(1);
        setRs485Direction(false);
    }
    if (!ok) {
        ui->statusbar->showMessage("Timeout de transmisión", 2000);
    }

    ui->lblValPWM1->setText(QString::number(pwm1));
    ui->lblValPWM2->setText(QString::number(pwm2));

    QString txHex;
    for (auto b : f)
        txHex += QString("%1 ").arg(quint8(b),2,16,QLatin1Char('0'));
    ui->txtRawData->append("TX: " + txHex.trimmed().toUpper());
}

void MainWindow::onPollTimeout()              { sendControlFrame(); }
void MainWindow::on_chkOut1_toggled(bool)     { sendControlFrame(); }
void MainWindow::on_chkOut2_toggled(bool)     { sendControlFrame(); }
void MainWindow::on_chkOut3_toggled(bool)     { sendControlFrame(); }
void MainWindow::on_sldPWM1_valueChanged(int v){ ui->lblValPWM1->setText(QString::number(v)); sendControlFrame(); }
void MainWindow::on_sldPWM2_valueChanged(int v){ ui->lblValPWM2->setText(QString::number(v)); sendControlFrame(); }


