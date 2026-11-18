#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QPushButton>
#include <QSpinBox>
#include <QTabWidget>
#include <QTextStream>
#include <QTimer>
#include <QTableWidget>
#include <QFontDatabase>

#include "main_window.h"

int main(int argc, char *argv[])
{
    QApplication application(argc, argv);
    // The offscreen platform does not always enumerate Windows system fonts.
    QFontDatabase::addApplicationFont(qEnvironmentVariable("WINDIR") + QStringLiteral("/Fonts/msyh.ttc"));
    MainWindow window;
    window.show();
    application.processEvents();
    QCoreApplication::sendPostedEvents(nullptr, QEvent::DeferredDelete);

    bool ok = true;
    QTextStream error(stderr);
    if (window.windowTitle() != QStringLiteral("挖掘机七舵机限位调试"))
    {
        error << "Unexpected window title\n";
        ok = false;
    }

    const QList<QTabWidget *> tabs = window.findChildren<QTabWidget *>();
    if (tabs.size() != 1 || tabs.first()->count() != 3)
    {
        error << "Expected one tab widget with three pages\n";
        ok = false;
    }

    bool foundDisabledShortcutControl = false;
    for (QCheckBox *checkBox : window.findChildren<QCheckBox *>())
    {
        if (checkBox->text().contains(QStringLiteral("启用键盘控制")) && !checkBox->isChecked())
        {
            foundDisabledShortcutControl = true;
            break;
        }
    }
    if (!foundDisabledShortcutControl)
    {
        error << "Shortcut safety checkbox missing or enabled by default\n";
        ok = false;
    }

    bool foundEmergencyStop = false;
    for (QPushButton *button : window.findChildren<QPushButton *>())
    {
        if (button->text().contains(QStringLiteral("全部停止")))
        {
            foundEmergencyStop = true;
            break;
        }
    }
    if (!foundEmergencyStop)
    {
        error << "Emergency stop control missing\n";
        ok = false;
    }

    QComboBox *profileCombo = window.findChild<QComboBox *>(QStringLiteral("machineProfileCombo"));
    if (profileCombo == nullptr || profileCombo->findText(QStringLiteral("Config2")) < 0)
    {
        error << "Config2 machine profile was not loaded\n";
        ok = false;
    }

    const QSpinBox *positionSpeed = window.findChild<QSpinBox *>(QStringLiteral("positionSpeed0"));
    const QSpinBox *steeringSpeed = window.findChild<QSpinBox *>(QStringLiteral("steeringSpeed"));
    const QSpinBox *wheelSpeed = window.findChild<QSpinBox *>(QStringLiteral("wheelSpeed"));
    const QSpinBox *gripperSpeed = window.findChild<QSpinBox *>(QStringLiteral("positionSpeed4"));
    if (positionSpeed == nullptr || positionSpeed->value() != 100 || positionSpeed->maximum() != 100 ||
        steeringSpeed == nullptr || steeringSpeed->value() != 1000 ||
        wheelSpeed == nullptr || wheelSpeed->value() != 3000 || wheelSpeed->isEnabled() ||
        gripperSpeed == nullptr || gripperSpeed->value() != 100 || gripperSpeed->maximum() != 100)
    {
        error << "Unexpected default servo speeds\n";
        ok = false;
    }
    const auto tables = window.findChildren<QTableWidget *>();
    if (tables.size() != 1 || tables.first()->rowCount() != 5 ||
        tables.first()->cellWidget(4, 4)->findChildren<QPushButton *>().size() != 4 ||
        tables.first()->cellWidget(4, 5)->findChild<QSpinBox *>() == nullptr ||
        tables.first()->cellWidget(0, 4)->isEnabled() || tables.first()->cellWidget(4, 4)->isEnabled())
    {
        error << "ID8 limit/motion controls missing\n";
        ok = false;
    }
    if (application.arguments().size() > 1)
    {
        window.grab().save(application.arguments().at(1));
    }

    QTimer::singleShot(0, &application, &QApplication::quit);
    application.exec();
    if (ok)
    {
        QTextStream(stdout) << "PASS: UI constructed with safety controls\n";
    }
    return ok ? 0 : 1;
}
