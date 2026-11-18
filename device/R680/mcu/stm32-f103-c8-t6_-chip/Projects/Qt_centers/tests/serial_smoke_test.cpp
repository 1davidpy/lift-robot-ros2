#include <QCoreApplication>
#include <QElapsedTimer>
#include <QSerialPort>
#include <QTextStream>
#include <QThread>

#include "servo_protocol.h"

namespace
{
QStringList collectLines(QSerialPort &port, QByteArray &buffer, int timeoutMs)
{
    QStringList lines;
    QElapsedTimer timer;
    timer.start();
    while (timer.elapsed() < timeoutMs)
    {
        port.waitForReadyRead(50);
        buffer.append(port.readAll());
        int newline = -1;
        while ((newline = buffer.indexOf('\n')) >= 0)
        {
            QByteArray raw = buffer.left(newline);
            buffer.remove(0, newline + 1);
            const QString line = QString::fromUtf8(raw).trimmed();
            if (!line.isEmpty())
            {
                lines.append(line);
            }
        }
    }
    return lines;
}

bool exchange(QSerialPort &port,
              QByteArray &buffer,
              const QString &command,
              const QString &expected,
              QStringList *received = nullptr)
{
    port.readAll();
    const QByteArray bytes = command.toUtf8() + '\n';
    if (port.write(bytes) != bytes.size() || !port.waitForBytesWritten(500))
    {
        return false;
    }
    // Offline servo mode/feedback reads can delay the command task.
    const QStringList lines = collectLines(port, buffer, 3000);
    if (received != nullptr)
    {
        *received = lines;
    }
    for (const QString &line : lines)
    {
        if (line == expected)
        {
            return true;
        }
    }
    return false;
}
}

int main(int argc, char *argv[])
{
    QCoreApplication application(argc, argv);
    QTextStream output(stdout);
    QTextStream error(stderr);
    const QStringList arguments = application.arguments();
    const bool protocolOnly = arguments.size() == 3 && arguments.at(2) == QStringLiteral("--protocol-only");
    if (arguments.size() != 2 && !protocolOnly)
    {
        error << "Usage: qt_centers_serial_smoke <COM port> [--protocol-only]\n";
        return 2;
    }

    QSerialPort port(application.arguments().at(1));
    port.setBaudRate(QSerialPort::Baud115200);
    port.setDataBits(QSerialPort::Data8);
    port.setParity(QSerialPort::NoParity);
    port.setStopBits(QSerialPort::OneStop);
    port.setFlowControl(QSerialPort::NoFlowControl);
    if (!port.open(QIODevice::ReadWrite))
    {
        error << "Open failed: " << port.errorString() << '\n';
        return 3;
    }

    port.clear();
    QByteArray buffer;
    collectLines(port, buffer, 250);

    if (!exchange(port, buffer, ServoProtocol::ping(), QStringLiteral("PONG")))
    {
        error << "PING/PONG failed\n";
        return 4;
    }

    QStringList lines;
    port.write((ServoProtocol::status() + QLatin1Char('\n')).toUtf8());
    port.waitForBytesWritten(500);
    lines = collectLines(port, buffer, 3000);
    std::optional<ServoStatus> status;
    for (const QString &line : lines)
    {
        status = ServoProtocol::parseStatus(line);
        if (status.has_value())
        {
            break;
        }
    }
    if (!status.has_value())
    {
        error << "STATUS parse failed\n";
        return 5;
    }
    if (!protocolOnly && status->onlineMask != 191U)
    {
        error << "Unexpected masks: online=" << status->onlineMask
              << " limits=" << status->limitMask << '\n';
        return 6;
    }

    if (!exchange(port, buffer, QStringLiteral("LIMIT 3 0 4095"), QStringLiteral("ERR NO_LIMIT")))
    {
        error << "ID4 no-limit check failed\n";
        return 7;
    }

    if (!protocolOnly)
    {
        port.write("FLASH_LIMITS\n");
        port.waitForBytesWritten(500);
        const auto flashLines = collectLines(port, buffer, 3000);
        int flashMinimum = -1;
        int flashMaximum = -1;
        for (const QString &line : flashLines)
        {
            const auto parts = line.split(QLatin1Char(' '), Qt::SkipEmptyParts);
            if (parts.size() == 10 && parts[0] == QStringLiteral("FLASH_LIMITS") && parts[1] == QStringLiteral("Config2"))
            {
                bool minOk = false, maxOk = false;
                flashMinimum = parts[2].toInt(&minOk);
                flashMaximum = parts[3].toInt(&maxOk);
                if (!minOk || !maxOk) return 11;
            }
        }
        if (flashMinimum < 0 || flashMinimum >= flashMaximum)
        { error << "Flash profile query failed\n"; return 11; }
        const int current = status->positions[0];
        const int minimum = qMax(flashMinimum, current - 10);
        const int maximum = qMin(flashMaximum, current + 10);
        if (!exchange(port, buffer, ServoProtocol::limit(0, minimum, maximum), QStringLiteral("OK LIMIT")) ||
            !exchange(port, buffer, ServoProtocol::clearLimit(0), QStringLiteral("OK CLEAR")))
        {
            error << "ID1 runtime limit check failed\n";
            return 8;
        }
    }

    if (!exchange(port, buffer, ServoProtocol::stop(3), QStringLiteral("OK STOP")) ||
        !exchange(port, buffer, ServoProtocol::stop(4), QStringLiteral("OK STOP_LOCKED")))
    {
        error << "Swing/ID8 stop response failed\n";
        return 10;
    }
    if (!exchange(port, buffer, ServoProtocol::stopAllWheels(), QStringLiteral("OK WHEEL_STOP_ALL")))
    {
        error << "Wheel stop failed\n";
        return 9;
    }

    output << "PASS " << (protocolOnly ? "protocol-only " : "full hardware ")
           << "online=" << status->onlineMask
           << " limits=" << status->limitMask
           << " positions=" << status->positions[0] << ',' << status->positions[1] << ','
           << status->positions[2] << ',' << status->positions[3] << ',' << status->positions[4] << '\n';
    return 0;
}

