#include "servo_protocol.h"

#include <QRegularExpression>

namespace
{
constexpr int kPositionMaximum = 4095;
constexpr int kSpeedMaximum = 3400;
}

std::optional<ServoStatus> ServoProtocol::parseStatus(const QString &line)
{
    static const QRegularExpression expression(
        QStringLiteral(R"(^\s*STATUS2\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(-?\d+)\s+(-?\d+)\s*$)"));

    const QRegularExpressionMatch match = expression.match(line);
    if (!match.hasMatch())
    {
        return std::nullopt;
    }

    ServoStatus result;
    bool ok = false;
    result.onlineMask = match.captured(1).toUInt(&ok);
    if (!ok || (result.onlineMask & ~0xBFU) != 0U)
    {
        return std::nullopt;
    }

    result.limitMask = match.captured(2).toUInt(&ok);
    if (!ok || (result.limitMask & ~0x17U) != 0U)
    {
        return std::nullopt;
    }

    for (int index = 0; index < 5; ++index)
    {
        result.positions[index] = match.captured(3 + index).toInt(&ok);
        if (!ok || result.positions[index] < 0 || result.positions[index] > kPositionMaximum)
        {
            return std::nullopt;
        }
    }

    for (int index = 0; index < 2; ++index)
    {
        result.wheelLevels[index] = match.captured(8 + index).toInt(&ok);
        if (!ok || result.wheelLevels[index] < -128 || result.wheelLevels[index] > 127)
        {
            return std::nullopt;
        }
    }

    return result;
}

QString ServoProtocol::ping()
{
    return QStringLiteral("PING");
}

QString ServoProtocol::status()
{
    return QStringLiteral("STATUS");
}

QString ServoProtocol::move(int axis, int position, int speed)
{
    if (axis < 0 || (axis > 4 || axis == 3) || position < 0 || position > kPositionMaximum ||
        speed < 1 || speed > kSpeedMaximum)
    {
        return {};
    }
    return QStringLiteral("MOVE %1 %2 %3").arg(axis).arg(position).arg(speed);
}

QString ServoProtocol::jog(int axis, int direction, int speed)
{
    if (axis < 0 || axis > 4 || (direction != -1 && direction != 1) ||
        speed < 1 || speed > kSpeedMaximum)
    {
        return {};
    }
    return QStringLiteral("JOG %1 %2 %3").arg(axis).arg(direction).arg(speed);
}

QString ServoProtocol::stop(int axis)
{
    if (axis < 0 || axis > 4)
    {
        return {};
    }
    return QStringLiteral("STOP %1").arg(axis);
}

QString ServoProtocol::limit(int axis, int minimum, int maximum)
{
    if (axis < 0 || (axis > 4 || axis == 3) || minimum < 0 || maximum > kPositionMaximum ||
        minimum >= maximum)
    {
        return {};
    }
    return QStringLiteral("LIMIT %1 %2 %3").arg(axis).arg(minimum).arg(maximum);
}

QString ServoProtocol::clearLimit(int axis)
{
    if (axis < 0 || (axis > 4 || axis == 3))
    {
        return {};
    }
    return QStringLiteral("CLEAR %1").arg(axis);
}

QString ServoProtocol::wheel(int wheel, int signedSpeed)
{
    if (wheel < 0 || wheel > 1 || signedSpeed < -kSpeedMaximum || signedSpeed > kSpeedMaximum)
    {
        return {};
    }
    return QStringLiteral("WHEEL %1 %2").arg(wheel).arg(signedSpeed);
}

QString ServoProtocol::logicalWheel(int wheel, int logicalSpeed)
{
    if (wheel < 0 || wheel > 1 || logicalSpeed < -kSpeedMaximum || logicalSpeed > kSpeedMaximum)
    {
        return {};
    }

    const int rawSpeed = (wheel == 0) ? -logicalSpeed : logicalSpeed;
    return ServoProtocol::wheel(wheel, rawSpeed);
}

QString ServoProtocol::stopAllWheels()
{
    return QStringLiteral("WHEEL_STOP_ALL");
}
