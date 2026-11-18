#ifndef SERIAL_CONTROLLER_H
#define SERIAL_CONTROLLER_H

#include <functional>

#include <QObject>
#include <QSerialPort>

class SerialController : public QObject
{
public:
    using ConnectionHandler = std::function<void(bool)>;
    using TextHandler = std::function<void(const QString &)>;

    explicit SerialController(QObject *parent = nullptr);
    ~SerialController() override;

    bool open(const QString &portName);
    void close();
    bool isOpen() const;
    QString portName() const;
    bool sendCommand(const QString &command);
    void sendShortcutStop();
    void setConnectionHandler(ConnectionHandler handler);
    void setLineHandler(TextHandler handler);
    void setLogHandler(TextHandler handler);
    void setErrorHandler(TextHandler handler);

private:
    void readAvailable();
    void handleError(QSerialPort::SerialPortError error);

    QSerialPort serialPort_;
    QByteArray receiveBuffer_;
    ConnectionHandler connectionHandler_;
    TextHandler lineHandler_;
    TextHandler logHandler_;
    TextHandler errorHandler_;
};

#endif
