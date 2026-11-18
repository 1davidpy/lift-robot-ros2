#ifndef SERVO_PROTOCOL_H
#define SERVO_PROTOCOL_H

#include <array>
#include <optional>

#include <QString>

struct ServoStatus
{
    quint32 onlineMask = 0;
    quint32 limitMask = 0;
    std::array<int, 5> positions{}; // IDs 1, 2, 3, 4, 8.
    std::array<int, 2> wheelLevels{};
};

class ServoProtocol
{
public:
    static std::optional<ServoStatus> parseStatus(const QString &line);

    static QString ping();
    static QString status();
    static QString move(int axis, int position, int speed);
    static QString jog(int axis, int direction, int speed);
    static QString stop(int axis);
    static QString limit(int axis, int minimum, int maximum);
    static QString clearLimit(int axis);
    static QString wheel(int wheel, int signedSpeed);
    static QString logicalWheel(int wheel, int logicalSpeed);
    static QString stopAllWheels();
};

#endif
