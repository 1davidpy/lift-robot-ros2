#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <array>

#include <QMainWindow>

#include "machine_config.h"
#include "servo_protocol.h"

class QCheckBox;
class QComboBox;
class QLabel;
class QPlainTextEdit;
class QPushButton;
class QSpinBox;
class QTabWidget;
class QTimer;
class QLineEdit;
class SerialController;

class MainWindow : public QMainWindow
{
public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

protected:
    bool eventFilter(QObject *watched, QEvent *event) override;
    void closeEvent(QCloseEvent *event) override;

private:
    struct PositionControls
    {
        QLabel *current = nullptr;
        QSpinBox *target = nullptr;
        QSpinBox *speed = nullptr;
        QSpinBox *minimum = nullptr;
        QSpinBox *maximum = nullptr;
        bool targetInitialized = false;
        QCheckBox *defaultEnabled = nullptr;
        QWidget *motion = nullptr;
        QPushButton *prepare = nullptr;
    };

    void buildUi();
    QWidget *buildPositionPage();
    QWidget *buildWheelPage();
    QWidget *buildLogPage();
    void refreshPorts();
    void loadMachineProfiles();
    void applySelectedMachineProfile();
    void saveSelectedMachineProfile();
    void updateConnectionUi(bool connected);
    void handleLine(const QString &line);
    void updateStatus(const ServoStatus &status);
    void updateCalibrationUi();
    void appendLog(const QString &text);
    void sendCommand(const QString &command);
    void sendAllStop();
    void stepAxis(int axis, int delta);
    void resetShortcutState(bool sendStop);
    void refreshSteeringJog();
    void updateWheelShortcutOutput();
    bool handleShortcutKey(int key, bool pressed);
    int shortcutWheelSpeed() const;

    SerialController *serialController_ = nullptr;
    MachineConfigRepository machineConfigs_;
    QString machineConfigPath_;
    QComboBox *machineProfileCombo_ = nullptr;
    QLabel *machineProfileDescription_ = nullptr;
    QComboBox *portCombo_ = nullptr;
    QPushButton *refreshPortsButton_ = nullptr;
    QPushButton *connectButton_ = nullptr;
    QLabel *connectionLabel_ = nullptr;
    std::array<QLabel *, 7> onlineLabels_{};
    std::array<QLabel *, 5> positionLabels_{};
    std::array<QLabel *, 5> limitLabels_{};
    std::array<QLabel *, 2> wheelLevelLabels_{};
    std::array<PositionControls, 5> positionControls_{};
    ServoStatus lastStatus_{};
    qint64 lastStatusTime_ = 0;
    unsigned calibrationReadyMask_ = 0;
    unsigned remoteReadyMask_ = 0;
    unsigned remoteRecoveryMask_ = 0;
    bool remoteEnabled_ = false;
    bool remoteArmed_ = false;
    QSpinBox *wheelSpeedSpin_ = nullptr;
    QSpinBox *steeringJogSpeedSpin_ = nullptr;
    QCheckBox *shortcutsEnabled_ = nullptr;
    QLabel *shortcutStateLabel_ = nullptr;
    QPlainTextEdit *logEdit_ = nullptr;
    QLineEdit *manualCommandEdit_ = nullptr;
    QTimer *steeringRefreshTimer_ = nullptr;
    int steeringDirection_ = 0;
    int lastLeftLogicalSpeed_ = 0;
    int lastRightLogicalSpeed_ = 0;
    bool bothForwardPressed_ = false;
    bool bothReversePressed_ = false;
    bool leftForwardPressed_ = false;
    bool leftReversePressed_ = false;
    bool rightForwardPressed_ = false;
    bool rightReversePressed_ = false;
};

#endif
