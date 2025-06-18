#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QSerialPortInfo>
#include <QDebug>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , serial(new QSerialPort(this))
    , pollTimer(new QTimer(this))
    , mode(CommMode::RS232)
{
    ui->setupUi(this);

    // Rango 0–255 en sliders
    ui->sldPWM1->setRange(0, 255);
    ui->sldPWM2->setRange(0, 255);

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
    mode = (idx == 0 ? CommMode::RS232 : CommMode::RS485);
    bool ctrl = (mode == CommMode::RS485);
    ui->chkOut1->setEnabled(ctrl);
    ui->chkOut2->setEnabled(ctrl);
    ui->chkOut3->setEnabled(ctrl);
    ui->sldPWM1->setEnabled(ctrl);
    ui->sldPWM2->setEnabled(ctrl);
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

        if (mode == CommMode::RS485) {
            sendControlFrame();
            pollTimer->start(1000);
        }
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
    while (recvBuffer.size() >= 12) {
        if (quint8(recvBuffer[0]) != 0x02) { recvBuffer.remove(0,1); continue; }
        QByteArray frame = recvBuffer.left(12);
        if (computeCRC8(frame.left(11)) == quint8(frame[11])) {
            processFrame(frame);
            recvBuffer.remove(0,12);
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
    quint8 inp = quint8(f[7]); qint8 t = qint8(f[8]);
    quint16 p = (quint8(f[9])<<8) | quint8(f[10]);

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

quint8 MainWindow::computeCRC8(const QByteArray &d) {
    quint8 crc=0;
    for(auto b:d){ crc^=quint8(b);
        for(int i=0;i<8;i++) crc=(crc&0x80)?(crc<<1)^0x07:(crc<<1);
    }
    return crc;
}

void MainWindow::sendControlFrame() {
    if(mode!=CommMode::RS485||!serial->isOpen()) return;
    serial->setFlowControl(QSerialPort::HardwareControl);
    serial->setRequestToSend(true);

    QByteArray f;
    f.append(char(0x02));
    quint8 m = (ui->chkOut1->isChecked()?1:0)
               |(ui->chkOut2->isChecked()?2:0)
               |(ui->chkOut3->isChecked()?4:0);
    f.append(char(m));
    f.append(char(ui->sldPWM1->value()));
    f.append(char(ui->sldPWM2->value()));
    f.append(char(computeCRC8(f)));

    serial->write(f);
    serial->flush();
    serial->waitForBytesWritten(50);

    serial->setRequestToSend(false);
    serial->setFlowControl(QSerialPort::NoFlowControl);

    ui->lblValPWM1->setText(QString::number(ui->sldPWM1->value()));
    ui->lblValPWM2->setText(QString::number(ui->sldPWM2->value()));

    QString txHex;
    for(auto b:f) txHex += QString("%1 ").arg(quint8(b),2,16,QLatin1Char('0'));
    ui->txtRawData->append("TX: " + txHex.trimmed().toUpper());
}

void MainWindow::onPollTimeout()              { sendControlFrame(); }
void MainWindow::on_chkOut1_toggled(bool)     { sendControlFrame(); }
void MainWindow::on_chkOut2_toggled(bool)     { sendControlFrame(); }
void MainWindow::on_chkOut3_toggled(bool)     { sendControlFrame(); }
void MainWindow::on_sldPWM1_valueChanged(int v){ ui->lblValPWM1->setText(QString::number(v)); sendControlFrame(); }
void MainWindow::on_sldPWM2_valueChanged(int v){ ui->lblValPWM2->setText(QString::number(v)); sendControlFrame(); }
