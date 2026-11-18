#include <QCoreApplication>
#include <QTextStream>

#include "servo_protocol.h"

namespace
{
int failures = 0;

void check(bool condition, const QString &message)
{
    if (!condition)
    {
        QTextStream(stderr) << "FAIL: " << message << '\n';
        ++failures;
    }
}
}

int main(int argc, char *argv[])
{
    QCoreApplication application(argc, argv);

    const auto status = ServoProtocol::parseStatus(
        QStringLiteral("STATUS2 191 23 842 1589 448 3852 2048 0 0\r"));
    check(status.has_value(), QStringLiteral("valid STATUS must parse"));
    if (status.has_value())
    {
        check(status->onlineMask == 191U, QStringLiteral("online mask"));
        check(status->limitMask == 23U, QStringLiteral("limit mask"));
        check(status->positions[0] == 842, QStringLiteral("position 0"));
        check(status->positions[3] == 3852, QStringLiteral("position 3"));
        check(status->positions[4] == 2048, QStringLiteral("ID8 feedback"));
    }

    check(!ServoProtocol::parseStatus(QStringLiteral("STATUS2 255 23 1 2 3 4 5 0 0")).has_value(),
          QStringLiteral("reject online mask overflow"));
    check(!ServoProtocol::parseStatus(QStringLiteral("STATUS2 191 23 1 2 3 5000 5 0 0")).has_value(),
          QStringLiteral("reject position overflow"));
    check(!ServoProtocol::parseStatus(QStringLiteral("OK STATUS")).has_value(),
          QStringLiteral("reject malformed status"));
    check(!ServoProtocol::parseStatus(QStringLiteral("STATUS 63 7 1 2 3 4 0 0")).has_value(),
          QStringLiteral("reject legacy six-servo status"));
    check(!ServoProtocol::parseStatus(QStringLiteral("STATUS2 191 8 1 2 3 4 5 0 0")).has_value(),
          QStringLiteral("swing cannot have limits"));
    check(!ServoProtocol::parseStatus(QStringLiteral("STATUS2 191 23 1 2 3 4 4096 0 0")).has_value(),
          QStringLiteral("reject invalid ID8 feedback"));
    check(ServoProtocol::move(4, 2000, 100) == QStringLiteral("MOVE 4 2000 100"),
          QStringLiteral("ID8 uses axis 4"));
    check(ServoProtocol::limit(4, 1800, 2200) == QStringLiteral("LIMIT 4 1800 2200"),
          QStringLiteral("ID8 limit command"));
    check(ServoProtocol::stop(4) == QStringLiteral("STOP 4"), QStringLiteral("ID8 stop"));
    check(ServoProtocol::clearLimit(4) == QStringLiteral("CLEAR 4"), QStringLiteral("ID8 restore"));

    check(ServoProtocol::move(0, 1000, 300) == QStringLiteral("MOVE 0 1000 300"),
          QStringLiteral("MOVE command"));
    check(ServoProtocol::jog(3, -1, 150) == QStringLiteral("JOG 3 -1 150"),
          QStringLiteral("JOG command"));
    check(ServoProtocol::stop(3) == QStringLiteral("STOP 3"),
          QStringLiteral("STOP command"));
    check(ServoProtocol::move(5, 1000, 300).isEmpty(),
          QStringLiteral("reject invalid move axis"));
    check(ServoProtocol::move(3, 1000, 300).isEmpty(),
          QStringLiteral("ID4 is continuous and rejects MOVE"));
    check(ServoProtocol::jog(3, 0, 150).isEmpty(),
          QStringLiteral("reject zero jog direction"));

    check(ServoProtocol::limit(0, 800, 900) == QStringLiteral("LIMIT 0 800 900"),
          QStringLiteral("LIMIT command"));
    check(ServoProtocol::clearLimit(2) == QStringLiteral("CLEAR 2"),
          QStringLiteral("CLEAR command"));
    check(ServoProtocol::limit(3, 0, 4095).isEmpty(),
          QStringLiteral("ID4 has no limit"));
    check(ServoProtocol::limit(0, 900, 800).isEmpty(),
          QStringLiteral("reject inverted limits"));
    check(ServoProtocol::clearLimit(3).isEmpty(),
          QStringLiteral("reject ID4 clear"));

    check(ServoProtocol::wheel(0, -100) == QStringLiteral("WHEEL 0 -100"),
          QStringLiteral("left wheel command"));
    check(ServoProtocol::wheel(1, 100) == QStringLiteral("WHEEL 1 100"),
          QStringLiteral("right wheel command"));
    check(ServoProtocol::logicalWheel(0, 100) == QStringLiteral("WHEEL 0 -100"),
          QStringLiteral("ID5 logical forward mapping"));
    check(ServoProtocol::logicalWheel(1, 100) == QStringLiteral("WHEEL 1 100"),
          QStringLiteral("ID6 logical forward mapping"));
    check(ServoProtocol::logicalWheel(0, -100) == QStringLiteral("WHEEL 0 100"),
          QStringLiteral("ID5 logical reverse mapping"));
    check(ServoProtocol::logicalWheel(1, -100) == QStringLiteral("WHEEL 1 -100"),
          QStringLiteral("ID6 logical reverse mapping"));
    check(ServoProtocol::stopAllWheels() == QStringLiteral("WHEEL_STOP_ALL"),
          QStringLiteral("wheel stop command"));
    check(ServoProtocol::wheel(2, 100).isEmpty(),
          QStringLiteral("reject invalid wheel"));
    check(ServoProtocol::wheel(0, 3401).isEmpty(),
          QStringLiteral("reject wheel overspeed"));

    if (failures == 0)
    {
        QTextStream(stdout) << "PASS: all protocol tests\n";
    }
    return failures == 0 ? 0 : 1;
}
