#include "serial_controller.h"

#include <utility>

#include "servo_protocol.h"

SerialController::SerialController(QObject *parent)
    : QObject(parent)
{
    connect(&serialPort_, &QSerialPort::readyRead, this, &SerialController::readAvailable);
    connect(&serialPort_, &QSerialPort::errorOccurred, this, &SerialController::handleError);
}

SerialController::~SerialController()
{
    close();
}

bool SerialController::open(const QString &portName)
{
    close();
    serialPort_.setPortName(portName);
    serialPort_.setBaudRate(QSerialPort::Baud115200);
    serialPort_.setDataBits(QSerialPort::Data8);
    serialPort_.setParity(QSerialPort::NoParity);
    serialPort_.setStopBits(QSerialPort::OneStop);
    serialPort_.setFlowControl(QSerialPort::NoFlowControl);

    if (!serialPort_.open(QIODevice::ReadWrite))
    {
        if (errorHandler_)
        {
            errorHandler_(QStringLiteral("无法打开 %1：%2").arg(portName, serialPort_.errorString()));
        }
        return false;
    }

    serialPort_.clear();
    receiveBuffer_.clear();
    if (logHandler_)
    {
        logHandler_(QStringLiteral("已连接 %1（115200 8N1）").arg(portName));
    }
    if (connectionHandler_)
    {
        connectionHandler_(true);
    }
    return true;
}

void SerialController::close()
{
    if (!serialPort_.isOpen())
    {
        return;
    }

    sendShortcutStop();
    serialPort_.waitForBytesWritten(100);
    const QString oldPortName = serialPort_.portName();
    serialPort_.close();
    receiveBuffer_.clear();
    if (logHandler_)
    {
        logHandler_(QStringLiteral("已断开 %1").arg(oldPortName));
    }
    if (connectionHandler_)
    {
        connectionHandler_(false);
    }
}

bool SerialController::isOpen() const
{
    return serialPort_.isOpen();
}

QString SerialController::portName() const
{
    return serialPort_.portName();
}

bool SerialController::sendCommand(const QString &command)
{
    if (!serialPort_.isOpen() || command.isEmpty())
    {
        return false;
    }

    const QByteArray bytes = command.toUtf8() + '\n';
    if (serialPort_.write(bytes) != bytes.size())
    {
        if (errorHandler_)
        {
            errorHandler_(QStringLiteral("串口写入失败：%1").arg(serialPort_.errorString()));
        }
        return false;
    }

    if (logHandler_)
    {
        logHandler_(QStringLiteral("TX  %1").arg(command));
    }
    return true;
}

void SerialController::sendShortcutStop()
{
    if (!serialPort_.isOpen())
    {
        return;
    }
    for (int axis = 0; axis < 5; ++axis)
    {
        sendCommand(ServoProtocol::stop(axis));
    }
    sendCommand(ServoProtocol::stopAllWheels());
}

void SerialController::readAvailable()
{
    receiveBuffer_.append(serialPort_.readAll());
    int newlineIndex = -1;
    while ((newlineIndex = receiveBuffer_.indexOf('\n')) >= 0)
    {
        QByteArray lineBytes = receiveBuffer_.left(newlineIndex);
        receiveBuffer_.remove(0, newlineIndex + 1);
        if (!lineBytes.isEmpty() && lineBytes.endsWith('\r'))
        {
            lineBytes.chop(1);
        }
        const QString line = QString::fromUtf8(lineBytes).trimmed();
        if (!line.isEmpty())
        {
            if (lineHandler_)
            {
                lineHandler_(line);
            }
        }
    }
}

void SerialController::handleError(QSerialPort::SerialPortError error)
{
    if (error == QSerialPort::NoError)
    {
        return;
    }

    if (error == QSerialPort::ResourceError)
    {
        if (errorHandler_)
        {
            errorHandler_(QStringLiteral("串口连接丢失：%1").arg(serialPort_.errorString()));
        }
        close();
    }
}

void SerialController::setConnectionHandler(ConnectionHandler handler)
{
    connectionHandler_ = std::move(handler);
}

void SerialController::setLineHandler(TextHandler handler)
{
    lineHandler_ = std::move(handler);
}

void SerialController::setLogHandler(TextHandler handler)
{
    logHandler_ = std::move(handler);
}

void SerialController::setErrorHandler(TextHandler handler)
{
    errorHandler_ = std::move(handler);
}
