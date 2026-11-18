#include "main_window.h"

#include <QAbstractSpinBox>
#include <QApplication>
#include <QCheckBox>
#include <QCloseEvent>
#include <QComboBox>
#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QEvent>
#include <QFileInfo>
#include <QFont>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QKeyEvent>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSerialPortInfo>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QTabWidget>
#include <QTableWidget>
#include <QTimer>
#include <QVBoxLayout>

#include "serial_controller.h"

namespace
{
const std::array<QString, 7> kServoNames = {
    QStringLiteral("1 动臂"),
    QStringLiteral("2 斗杆"),
    QStringLiteral("3 夹爪俯仰"),
    QStringLiteral("4 转向"),
    QStringLiteral("8 夹爪开合"),
    QStringLiteral("5 左轮"),
    QStringLiteral("6 右轮"),
};

const std::array<int, 5> kDefaultMinimum = {0, 0, 0, 0, 0};
const std::array<int, 5> kDefaultMaximum = {4095, 4095, 4095, 4095, 4095};

QLabel *createValueLabel(const QString &text = QStringLiteral("--"))
{
    auto *label = new QLabel(text);
    label->setAlignment(Qt::AlignCenter);
    label->setMinimumWidth(70);
    return label;
}

QString findMachineConfigPath()
{
    QDir searchDirectory(QCoreApplication::applicationDirPath());
    for (int level = 0; level < 6; ++level)
    {
        const QString candidate = searchDirectory.filePath(
            QStringLiteral("excavator_controlled_servo/config/config2.json"));
        if (QFileInfo::exists(candidate))
        {
            return QDir::cleanPath(candidate);
        }
        if (!searchDirectory.cdUp())
        {
            break;
        }
    }
    return QDir(QCoreApplication::applicationDirPath())
        .filePath(QStringLiteral("config/config2.json"));
}
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
      serialController_(new SerialController(this)),
      steeringRefreshTimer_(new QTimer(this))
{
    setFont(QFont(QStringLiteral("Microsoft YaHei"), 9));
    buildUi();
    setWindowTitle(QStringLiteral("挖掘机七舵机限位调试"));
    resize(1350, 900);

    serialController_->setConnectionHandler([this](bool connected) {
        updateConnectionUi(connected);
    });
    serialController_->setLineHandler([this](const QString &line) {
        handleLine(line);
    });
    serialController_->setLogHandler([this](const QString &message) {
        appendLog(message);
    });
    serialController_->setErrorHandler([this](const QString &message) {
        appendLog(QStringLiteral("错误：%1").arg(message));
        QMessageBox::warning(this, QStringLiteral("串口错误"), message);
    });

    steeringRefreshTimer_->setInterval(100);
    connect(steeringRefreshTimer_, &QTimer::timeout, this, &MainWindow::refreshSteeringJog);

    qApp->installEventFilter(this);
    machineConfigPath_ = findMachineConfigPath();
    loadMachineProfiles();
    refreshPorts();
    updateConnectionUi(false);
}

MainWindow::~MainWindow()
{
    resetShortcutState(true);
    serialController_->close();
    serialController_->setConnectionHandler({});
    serialController_->setLineHandler({});
    serialController_->setLogHandler({});
    serialController_->setErrorHandler({});
    qApp->removeEventFilter(this);
}

void MainWindow::buildUi()
{
    auto *central = new QWidget(this);
    auto *layout = new QVBoxLayout(central);

    auto *profileGroup = new QGroupBox(QStringLiteral("装配机器配置"), central);
    auto *profileLayout = new QGridLayout(profileGroup);
    machineProfileCombo_ = new QComboBox(profileGroup);
    machineProfileCombo_->setObjectName(QStringLiteral("machineProfileCombo"));
    auto *reloadProfileButton = new QPushButton(QStringLiteral("重新读取"), profileGroup);
    auto *applyProfileButton = new QPushButton(QStringLiteral("加载参数"), profileGroup);
    auto *saveProfileButton = new QPushButton(QStringLiteral("保存当前参数"), profileGroup);
    machineProfileDescription_ = new QLabel(QStringLiteral("尚未加载配置"), profileGroup);
    machineProfileDescription_->setWordWrap(true);
    profileLayout->addWidget(new QLabel(QStringLiteral("配置："), profileGroup), 0, 0);
    profileLayout->addWidget(machineProfileCombo_, 0, 1);
    profileLayout->addWidget(reloadProfileButton, 0, 2);
    profileLayout->addWidget(applyProfileButton, 0, 3);
    profileLayout->addWidget(saveProfileButton, 0, 4);
    profileLayout->addWidget(machineProfileDescription_, 1, 0, 1, 5);
    layout->addWidget(profileGroup);

    connect(reloadProfileButton, &QPushButton::clicked, this, &MainWindow::loadMachineProfiles);
    connect(applyProfileButton, &QPushButton::clicked, this, &MainWindow::applySelectedMachineProfile);
    connect(saveProfileButton, &QPushButton::clicked, this, &MainWindow::saveSelectedMachineProfile);

    auto *connectionGroup = new QGroupBox(QStringLiteral("串口连接"), central);
    auto *connectionLayout = new QHBoxLayout(connectionGroup);
    portCombo_ = new QComboBox(connectionGroup);
    refreshPortsButton_ = new QPushButton(QStringLiteral("刷新串口"), connectionGroup);
    connectButton_ = new QPushButton(QStringLiteral("连接"), connectionGroup);
    connectionLabel_ = new QLabel(QStringLiteral("未连接"), connectionGroup);
    connectionLayout->addWidget(new QLabel(QStringLiteral("端口："), connectionGroup));
    connectionLayout->addWidget(portCombo_, 1);
    connectionLayout->addWidget(refreshPortsButton_);
    connectionLayout->addWidget(connectButton_);
    connectionLayout->addWidget(connectionLabel_);
    connectionLayout->addWidget(new QLabel(QStringLiteral("115200 8N1"), connectionGroup));
    layout->addWidget(connectionGroup);

    connect(refreshPortsButton_, &QPushButton::clicked, this, &MainWindow::refreshPorts);
    connect(connectButton_, &QPushButton::clicked, this, [this]() {
        if (serialController_->isOpen())
        {
            resetShortcutState(true);
            serialController_->close();
            return;
        }
        const QString portName = portCombo_->currentData().toString();
        if (portName.isEmpty())
        {
            QMessageBox::information(this, QStringLiteral("串口"), QStringLiteral("没有可用串口。"));
            return;
        }
        if (serialController_->open(portName))
        {
            sendCommand(ServoProtocol::ping());
            sendCommand(ServoProtocol::status());
        }
    });

    auto *statusGroup = new QGroupBox(QStringLiteral("实时状态"), central);
    auto *statusLayout = new QGridLayout(statusGroup);
    statusLayout->addWidget(new QLabel(QStringLiteral("舵机"), statusGroup), 0, 0);
    statusLayout->addWidget(new QLabel(QStringLiteral("在线"), statusGroup), 0, 1);
    statusLayout->addWidget(new QLabel(QStringLiteral("位置/轮速状态"), statusGroup), 0, 2);
    statusLayout->addWidget(new QLabel(QStringLiteral("限位"), statusGroup), 0, 3);
    for (int index = 0; index < 7; ++index)
    {
        statusLayout->addWidget(new QLabel(kServoNames[index], statusGroup), index + 1, 0);
        onlineLabels_[index] = createValueLabel();
        statusLayout->addWidget(onlineLabels_[index], index + 1, 1);
        if (index < 5)
        {
            positionLabels_[index] = createValueLabel();
            limitLabels_[index] = createValueLabel(index != 3 ? QStringLiteral("等待")
                                                              : QStringLiteral("不使用"));
            statusLayout->addWidget(positionLabels_[index], index + 1, 2);
            statusLayout->addWidget(limitLabels_[index], index + 1, 3);
        }
        else
        {
            wheelLevelLabels_[index - 5] = createValueLabel(QStringLiteral("0"));
            statusLayout->addWidget(wheelLevelLabels_[index - 5], index + 1, 2);
            statusLayout->addWidget(createValueLabel(QStringLiteral("不使用")), index + 1, 3);
        }
    }
    layout->addWidget(statusGroup);

    auto *tabs = new QTabWidget(central);
    tabs->addTab(buildPositionPage(), QStringLiteral("位置与限位"));
    tabs->addTab(buildWheelPage(), QStringLiteral("4/5/6 控制与快捷键"));
    tabs->addTab(buildLogPage(), QStringLiteral("串口日志"));
    layout->addWidget(tabs, 1);

    setCentralWidget(central);
}

QWidget *MainWindow::buildPositionPage()
{
    auto *page = new QWidget;
    auto *layout = new QVBoxLayout(page);
    auto *table = new QTableWidget(5, 10, page);
    table->setHorizontalHeaderLabels({
        QStringLiteral("舵机"), QStringLiteral("当前位置"), QStringLiteral("目标位置"),
        QStringLiteral("速度"), QStringLiteral("移动"), QStringLiteral("最小值"),
        QStringLiteral("最大值"), QStringLiteral("设置限位"), QStringLiteral("恢复默认"), QStringLiteral("固件默认启用")});
    table->verticalHeader()->setVisible(false);
    table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    table->setSelectionMode(QAbstractItemView::NoSelection);
    table->setEditTriggers(QAbstractItemView::NoEditTriggers);

    for (int axis = 0; axis < 5; ++axis)
    {
        table->setCellWidget(axis, 0, createValueLabel(kServoNames[axis]));
        positionControls_[axis].current = createValueLabel();
        table->setCellWidget(axis, 1, positionControls_[axis].current);

        positionControls_[axis].target = new QSpinBox(table);
        positionControls_[axis].target->setRange(0, 4095);
        table->setCellWidget(axis, 2, positionControls_[axis].target);

        positionControls_[axis].speed = new QSpinBox(table);
        positionControls_[axis].speed->setObjectName(
            QStringLiteral("positionSpeed%1").arg(axis));
        positionControls_[axis].speed->setRange(1, 100);
        positionControls_[axis].speed->setValue(100);
        table->setCellWidget(axis, 3, positionControls_[axis].speed);

        auto *moveButton = new QPushButton(QStringLiteral("移动"), table);
        if (axis != 3)
        {
            auto *motionBox = new QWidget(table);
            positionControls_[axis].motion = motionBox;
            auto *motionLayout = new QGridLayout(motionBox);
            motionLayout->setContentsMargins(0, 0, 0, 0);
            auto *negativeStep = new QPushButton(QStringLiteral("-10"), motionBox);
            auto *positiveStep = new QPushButton(QStringLiteral("+10"), motionBox);
            auto *stopAxis = new QPushButton(QStringLiteral("停"), motionBox);
            motionBox->setEnabled(false);
            positionControls_[axis].target->setEnabled(false);
            positionControls_[axis].speed->setEnabled(false);
            motionBox->setToolTip(QStringLiteral("先设置安全小范围并准备标定。每次最多移动10，速度不超过100；遥控位置轴仍锁定。"));
            moveButton->setText(QStringLiteral("移动"));
            motionLayout->addWidget(negativeStep, 0, 0);
            motionLayout->addWidget(positiveStep, 0, 1);
            motionLayout->addWidget(moveButton, 1, 0);
            motionLayout->addWidget(stopAxis, 1, 1);
            table->setCellWidget(axis, 4, motionBox);
            connect(negativeStep, &QPushButton::clicked, this, [this, axis]() { stepAxis(axis, -10); });
            connect(positiveStep, &QPushButton::clicked, this, [this, axis]() { stepAxis(axis, 10); });
            connect(stopAxis, &QPushButton::clicked, this, [this, axis]() {
                sendCommand(ServoProtocol::stop(axis));
            });
            connect(moveButton, &QPushButton::clicked, this, [this, axis]() {
                sendCommand(ServoProtocol::move(axis,
                                                 positionControls_[axis].target->value(),
                                                 positionControls_[axis].speed->value()));
            });
        }
        else
        {
            positionControls_[axis].target->setEnabled(false);
            moveButton->setText(QStringLiteral("连续模式"));
            moveButton->setEnabled(false);
            table->setCellWidget(axis, 4, moveButton);
        }

        if (axis != 3)
        {
            positionControls_[axis].minimum = new QSpinBox(table);
            positionControls_[axis].minimum->setRange(0, 4095);
            positionControls_[axis].minimum->setValue(kDefaultMinimum[axis]);

            positionControls_[axis].maximum = new QSpinBox(table);
            positionControls_[axis].maximum->setRange(0, 4095);
            positionControls_[axis].maximum->setValue(kDefaultMaximum[axis]);

            for (int boundary = 0; boundary < 2; ++boundary)
            {
                auto *box = new QWidget(table);
                auto *boxLayout = new QVBoxLayout(box);
                boxLayout->setContentsMargins(0, 0, 0, 0);
                auto *spin = boundary == 0 ? positionControls_[axis].minimum
                                          : positionControls_[axis].maximum;
                auto *capture = new QPushButton(QStringLiteral("取当前位置"), box);
                boxLayout->addWidget(spin);
                boxLayout->addWidget(capture);
                table->setCellWidget(axis, 5 + boundary, box);
                connect(capture, &QPushButton::clicked, this, [this, axis, spin]() {
                    const int id = axis == 4 ? 8 : axis + 1;
                    if (!serialController_->isOpen() ||
                        QDateTime::currentMSecsSinceEpoch() - lastStatusTime_ > 500 ||
                        (lastStatus_.onlineMask & (1U << (id - 1))) == 0U)
                    {
                        appendLog(QStringLiteral("当前位置不可用，请先连接并确认该舵机在线。"));
                        return;
                    }
                    spin->setValue(lastStatus_.positions[axis]);
                });
            }

            positionControls_[axis].defaultEnabled = new QCheckBox(QStringLiteral("已标定"), table);
            positionControls_[axis].defaultEnabled->setChecked(false);
            table->setCellWidget(axis, 9, positionControls_[axis].defaultEnabled);

            auto *limitButton = new QPushButton(QStringLiteral("设置"), table);
            auto *clearButton = new QPushButton(QStringLiteral("恢复"), table);
            auto *limitBox = new QWidget(table);
            auto *limitLayout = new QVBoxLayout(limitBox);
            limitLayout->setContentsMargins(0, 0, 0, 0);
            auto *prepareButton = new QPushButton(QStringLiteral("准备标定"), limitBox);
            prepareButton->setObjectName(QStringLiteral("calibrationPrepare%1").arg(axis));
            prepareButton->setEnabled(false);
            prepareButton->setToolTip(QStringLiteral("仅本轴：暂时卸力切换位置模式，以新读数设置保持目标后恢复力矩。先支撑机构，避免卸力下落。"));
            positionControls_[axis].prepare = prepareButton;
            limitLayout->addWidget(limitButton);
            limitLayout->addWidget(prepareButton);
            table->setCellWidget(axis, 7, limitBox);
            connect(prepareButton, &QPushButton::clicked, this, [this, axis]() {
                if (QDateTime::currentMSecsSinceEpoch() - lastStatusTime_ > 500) return;
                sendCommand(QStringLiteral("CAL_PREPARE %1").arg(axis));
            });
            table->setCellWidget(axis, 8, clearButton);
            connect(limitButton, &QPushButton::clicked, this, [this, axis]() {
                const QString command = ServoProtocol::limit(
                    axis,
                    positionControls_[axis].minimum->value(),
                    positionControls_[axis].maximum->value());
                if (command.isEmpty())
                {
                    QMessageBox::warning(this, QStringLiteral("限位值错误"),
                                         QStringLiteral("最小值必须小于最大值。"));
                    return;
                }
                sendCommand(command);
            });
            connect(clearButton, &QPushButton::clicked, this, [this, axis]() {
                sendCommand(ServoProtocol::clearLimit(axis));
            });
        }
        else
        {
            auto *noLimit = createValueLabel(QStringLiteral("4号不使用软件限位"));
            table->setSpan(axis, 5, 1, 4);
            table->setCellWidget(axis, 5, noLimit);
        }
        table->setRowHeight(axis, 75);
    }

    table->setMinimumHeight(410);
    layout->addWidget(table);

    auto *buttonLayout = new QHBoxLayout;
    auto *refreshButton = new QPushButton(QStringLiteral("读取状态"), page);
    auto *prepareRemoteButton = new QPushButton(QStringLiteral("准备遥控"), page);
    auto *resumeRemoteButton = new QPushButton(QStringLiteral("恢复遥控"), page);
    auto *pauseRemoteButton = new QPushButton(QStringLiteral("暂停遥控"), page);
    prepareRemoteButton->setToolTip(QStringLiteral("先支撑大小臂与夹爪并保持遥控中立。短暂卸力设置各轴模式，以实时位置保持；不回中。"));
    resumeRemoteButton->setToolTip(QStringLiteral("模式和目标已准备时恢复遥控；需全部输入中立1秒。不自动切换模式。"));
    connect(prepareRemoteButton, &QPushButton::clicked, this, [this]() { sendCommand(QStringLiteral("PREPARE_REMOTE")); });
    connect(resumeRemoteButton, &QPushButton::clicked, this, [this]() { sendCommand(QStringLiteral("RESUME_REMOTE")); });
    connect(pauseRemoteButton, &QPushButton::clicked, this, [this]() { sendCommand(QStringLiteral("PAUSE_REMOTE")); });
    auto *stopButton = new QPushButton(QStringLiteral("全部停止"), page);
    stopButton->setStyleSheet(QStringLiteral("font-weight: bold; color: #b00020;"));
    buttonLayout->addWidget(refreshButton);
    buttonLayout->addWidget(prepareRemoteButton);
    buttonLayout->addWidget(resumeRemoteButton);
    buttonLayout->addWidget(pauseRemoteButton);
    buttonLayout->addStretch();
    buttonLayout->addWidget(stopButton);
    connect(refreshButton, &QPushButton::clicked, this,
            [this]() { sendCommand(ServoProtocol::status()); });
    connect(stopButton, &QPushButton::clicked, this, &MainWindow::sendAllStop);
    layout->addLayout(buttonLayout);
    auto *controlHelp = new QLabel(
        QStringLiteral("配置2：LY→2、RY→1、RX→8、BL/BR单按→3、LX→4；BL+BR后退。遥控大臂250、小臂500、俯仰/夹爪200；回转1000、履带3000。Qt标定速度最多100，先暂停遥控。"),
        page);
    controlHelp->setWordWrap(true);
    layout->addWidget(controlHelp);
    layout->addStretch();
    return page;
}

QWidget *MainWindow::buildWheelPage()
{
    // Continuous motion remains inhibited during the limit-setup phase.
    auto *page = new QWidget;
    auto *layout = new QVBoxLayout(page);

    auto *settingsGroup = new QGroupBox(QStringLiteral("快捷键速度"), page);
    page->setEnabled(false);
    layout->addWidget(new QLabel(QStringLiteral("4/5/6号由遥控器控制：回转1000、履带固定3000。BL/BR单按控制3号俯仰，两键同时按后退，不调速；本页底盘运动按钮禁用。"), page));
    auto *settingsLayout = new QFormLayout(settingsGroup);
    wheelSpeedSpin_ = new QSpinBox(settingsGroup);
    wheelSpeedSpin_->setObjectName(QStringLiteral("wheelSpeed"));
    wheelSpeedSpin_->setRange(1, 3400);
    wheelSpeedSpin_->setValue(3000);
    steeringJogSpeedSpin_ = new QSpinBox(settingsGroup);
    steeringJogSpeedSpin_->setObjectName(QStringLiteral("steeringSpeed"));
    steeringJogSpeedSpin_->setRange(1, 3400);
    steeringJogSpeedSpin_->setValue(1000);
    settingsLayout->addRow(QStringLiteral("左右轮速度："), wheelSpeedSpin_);
    settingsLayout->addRow(QStringLiteral("4号转向速度："), steeringJogSpeedSpin_);
    layout->addWidget(settingsGroup);

    auto *manualGroup = new QGroupBox(QStringLiteral("手动控制"), page);
    auto *manualLayout = new QGridLayout(manualGroup);
    manualLayout->addWidget(new QLabel(QStringLiteral("对象"), manualGroup), 0, 0);
    manualLayout->addWidget(new QLabel(QStringLiteral("反向"), manualGroup), 0, 1);
    manualLayout->addWidget(new QLabel(QStringLiteral("停止"), manualGroup), 0, 2);
    manualLayout->addWidget(new QLabel(QStringLiteral("正向"), manualGroup), 0, 3);

    auto addWheelRow = [this, manualGroup, manualLayout](int row, int wheel, const QString &name) {
        auto *reverse = new QPushButton(QStringLiteral("反向"), manualGroup);
        auto *stop = new QPushButton(QStringLiteral("停止"), manualGroup);
        auto *forward = new QPushButton(QStringLiteral("正向"), manualGroup);
        manualLayout->addWidget(new QLabel(name, manualGroup), row, 0);
        manualLayout->addWidget(reverse, row, 1);
        manualLayout->addWidget(stop, row, 2);
        manualLayout->addWidget(forward, row, 3);
        connect(reverse, &QPushButton::pressed, this, [this, wheel]() {
            const int logical = -wheelSpeedSpin_->value();
            sendCommand(ServoProtocol::logicalWheel(wheel, logical));
        });
        connect(reverse, &QPushButton::released, this, [this, wheel]() {
            sendCommand(ServoProtocol::wheel(wheel, 0));
        });
        connect(stop, &QPushButton::pressed, this, [this, wheel]() {
            sendCommand(ServoProtocol::wheel(wheel, 0));
        });
        connect(forward, &QPushButton::pressed, this, [this, wheel]() {
            const int logical = wheelSpeedSpin_->value();
            sendCommand(ServoProtocol::logicalWheel(wheel, logical));
        });
        connect(forward, &QPushButton::released, this, [this, wheel]() {
            sendCommand(ServoProtocol::wheel(wheel, 0));
        });
    };
    addWheelRow(1, 0, QStringLiteral("5号左轮"));
    addWheelRow(2, 1, QStringLiteral("6号右轮"));

    auto *steerNegative = new QPushButton(QStringLiteral("位置减小"), manualGroup);
    auto *steerStop = new QPushButton(QStringLiteral("停止"), manualGroup);
    auto *steerPositive = new QPushButton(QStringLiteral("位置增大"), manualGroup);
    manualLayout->addWidget(new QLabel(QStringLiteral("4号转向"), manualGroup), 3, 0);
    manualLayout->addWidget(steerNegative, 3, 1);
    manualLayout->addWidget(steerStop, 3, 2);
    manualLayout->addWidget(steerPositive, 3, 3);
    connect(steerNegative, &QPushButton::pressed, this, [this]() {
        steeringDirection_ = -1;
        refreshSteeringJog();
        steeringRefreshTimer_->start();
    });
    connect(steerNegative, &QPushButton::released, this, [this]() {
        steeringDirection_ = 0;
        steeringRefreshTimer_->stop();
        sendCommand(ServoProtocol::stop(3));
    });
    connect(steerPositive, &QPushButton::pressed, this, [this]() {
        steeringDirection_ = 1;
        refreshSteeringJog();
        steeringRefreshTimer_->start();
    });
    connect(steerPositive, &QPushButton::released, this, [this]() {
        steeringDirection_ = 0;
        steeringRefreshTimer_->stop();
        sendCommand(ServoProtocol::stop(3));
    });
    connect(steerStop, &QPushButton::clicked, this, [this]() {
        steeringDirection_ = 0;
        steeringRefreshTimer_->stop();
        sendCommand(ServoProtocol::stop(3));
    });
    layout->addWidget(manualGroup);

    auto *shortcutGroup = new QGroupBox(QStringLiteral("键盘快捷键"), page);
    auto *shortcutLayout = new QVBoxLayout(shortcutGroup);
    shortcutsEnabled_ = new QCheckBox(QStringLiteral("启用键盘控制（请先架空左右轮）"), shortcutGroup);
    shortcutStateLabel_ = new QLabel(QStringLiteral("快捷键未启用"), shortcutGroup);
    shortcutLayout->addWidget(shortcutsEnabled_);
    shortcutLayout->addWidget(new QLabel(
        QStringLiteral("←/→：4号位置减小/增大    W/S：双轮前进/后退    Q/A：仅左轮前进/后退    E/D：仅右轮前进/后退    空格：全部停止"),
        shortcutGroup));
    shortcutLayout->addWidget(shortcutStateLabel_);
    connect(shortcutsEnabled_, &QCheckBox::toggled, this, [this](bool checked) {
        if (!checked)
        {
            resetShortcutState(true);
        }
        shortcutStateLabel_->setText(checked ? QStringLiteral("快捷键已启用，松开按键即停止")
                                             : QStringLiteral("快捷键未启用"));
    });
    layout->addWidget(shortcutGroup);

    auto *stopButton = new QPushButton(QStringLiteral("空格 / 点击此处：全部停止"), page);
    stopButton->setMinimumHeight(48);
    stopButton->setStyleSheet(QStringLiteral("font-weight: bold; color: white; background: #b00020;"));
    connect(stopButton, &QPushButton::clicked, this, &MainWindow::sendAllStop);
    layout->addWidget(stopButton);
    layout->addStretch();
    return page;
}

QWidget *MainWindow::buildLogPage()
{
    auto *page = new QWidget;
    auto *layout = new QVBoxLayout(page);
    logEdit_ = new QPlainTextEdit(page);
    logEdit_->setReadOnly(true);
    logEdit_->setMaximumBlockCount(3000);
    layout->addWidget(logEdit_, 1);

    auto *commandLayout = new QHBoxLayout;
    manualCommandEdit_ = new QLineEdit(page);
    manualCommandEdit_->setPlaceholderText(QStringLiteral("输入原始命令，例如 STATUS"));
    auto *sendButton = new QPushButton(QStringLiteral("发送"), page);
    auto *clearButton = new QPushButton(QStringLiteral("清空日志"), page);
    commandLayout->addWidget(manualCommandEdit_, 1);
    commandLayout->addWidget(sendButton);
    commandLayout->addWidget(clearButton);
    layout->addLayout(commandLayout);

    auto sendManual = [this]() {
        const QString command = manualCommandEdit_->text().trimmed();
        if (!command.isEmpty())
        {
            sendCommand(command);
            manualCommandEdit_->clear();
        }
    };
    connect(sendButton, &QPushButton::clicked, this, sendManual);
    connect(manualCommandEdit_, &QLineEdit::returnPressed, this, sendManual);
    connect(clearButton, &QPushButton::clicked, logEdit_, &QPlainTextEdit::clear);
    return page;
}

void MainWindow::refreshPorts()
{
    const QString previous = portCombo_->currentData().toString();
    portCombo_->clear();
    const QList<QSerialPortInfo> ports = QSerialPortInfo::availablePorts();
    for (const QSerialPortInfo &port : ports)
    {
        const QString description = port.description().isEmpty()
                                        ? QStringLiteral("串口设备")
                                        : port.description();
        portCombo_->addItem(QStringLiteral("%1 — %2").arg(port.portName(), description),
                            port.portName());
    }

    int selected = portCombo_->findData(previous);
    if (selected < 0)
    {
        selected = portCombo_->findData(QStringLiteral("COM6"));
    }
    if (selected >= 0)
    {
        portCombo_->setCurrentIndex(selected);
    }
    appendLog(QStringLiteral("发现 %1 个串口").arg(ports.size()));
}

void MainWindow::loadMachineProfiles()
{
    QString error;
    if (!machineConfigs_.load(machineConfigPath_, &error))
    {
        machineProfileCombo_->clear();
        machineProfileDescription_->setText(QStringLiteral("配置读取失败：%1").arg(error));
        appendLog(QStringLiteral("配置读取失败：%1").arg(error));
        return;
    }

    const QSignalBlocker blocker(machineProfileCombo_);
    machineProfileCombo_->clear();
    machineProfileCombo_->addItems(machineConfigs_.profileIds());
    const int activeIndex = machineProfileCombo_->findText(machineConfigs_.activeProfileId());
    machineProfileCombo_->setCurrentIndex(activeIndex >= 0 ? activeIndex : 0);
    applySelectedMachineProfile();
    appendLog(QStringLiteral("已读取机器配置：%1").arg(machineConfigPath_));
}

void MainWindow::applySelectedMachineProfile()
{
    const QString profileId = machineProfileCombo_->currentText();
    const std::optional<MachineProfile> selected = machineConfigs_.profile(profileId);
    if (!selected.has_value())
    {
        return;
    }

    const MachineProfile &profile = *selected;
    for (int index = 0; index < 4; ++index)
    {
        const int axis = index == 3 ? 4 : index;
        const ServoLimitProfile &limit = profile.limits[index];
        positionControls_[axis].minimum->setValue(limit.minimum);
        positionControls_[axis].maximum->setValue(limit.maximum);
        positionControls_[axis].defaultEnabled->setChecked(limit.enabled);
    }
    machineProfileDescription_->setText(
        QStringLiteral("%1 / %2　%3（配置包含1/2/3/8号限位；加载不会自动下发）")
            .arg(profile.assemblyVersion, profile.machineNumber, profile.description));

    machineConfigs_.setActiveProfileId(profileId);
    appendLog(QStringLiteral("已加载配置 %1 到界面，未向控制板发送命令").arg(profileId));
}

void MainWindow::saveSelectedMachineProfile()
{
    const QString profileId = machineProfileCombo_->currentText();
    const std::optional<MachineProfile> selected = machineConfigs_.profile(profileId);
    if (!selected.has_value())
    {
        QMessageBox::warning(this, QStringLiteral("机器配置"), QStringLiteral("没有可保存的配置。"));
        return;
    }

    MachineProfile profile = *selected;
    for (int index = 0; index < 4; ++index)
    {
        const int axis = index == 3 ? 4 : index;
        ServoLimitProfile &limit = profile.limits[index];
        limit.minimum = positionControls_[axis].minimum->value();
        limit.maximum = positionControls_[axis].maximum->value();
        limit.enabled = positionControls_[axis].defaultEnabled->isChecked();
        if (limit.minimum >= limit.maximum)
        {
            QMessageBox::warning(this, QStringLiteral("限位值错误"),
                                 QStringLiteral("%1 的最小值必须小于最大值。")
                                     .arg(limit.name));
            return;
        }
        if (limit.referenceMode == 1 && limit.minimum < limit.encoderOffset)
        {
            QMessageBox::warning(this, QStringLiteral("参考坐标错误"),
                                 QStringLiteral("%1 的最小值不能低于编码器偏移%2，以免跨零换算。")
                                     .arg(limit.name).arg(limit.encoderOffset));
            return;
        }
    }
    machineConfigs_.updateProfile(profile);
    machineConfigs_.setActiveProfileId(profileId);

    QString error;
    if (!machineConfigs_.save(&error))
    {
        QMessageBox::warning(this, QStringLiteral("机器配置"), error);
        return;
    }
    appendLog(QStringLiteral("已保存配置 %1 的1/2/3/8号限位").arg(profileId));
}

void MainWindow::updateConnectionUi(bool connected)
{
    connectionLabel_->setText(connected
                                  ? QStringLiteral("已连接 %1").arg(serialController_->portName())
                                  : QStringLiteral("未连接"));
    connectionLabel_->setStyleSheet(connected ? QStringLiteral("color: #15803d; font-weight: bold;")
                                               : QStringLiteral("color: #b00020; font-weight: bold;"));
    connectButton_->setText(connected ? QStringLiteral("断开") : QStringLiteral("连接"));
    portCombo_->setEnabled(!connected);
    refreshPortsButton_->setEnabled(!connected);
    if (!connected)
    {
        lastStatusTime_ = 0;
        calibrationReadyMask_ = 0;
        remoteReadyMask_ = remoteRecoveryMask_ = 0;
        remoteEnabled_ = remoteArmed_ = false;
        lastStatus_ = {};
        for (auto &controls : positionControls_)
        {
            controls.targetInitialized = false;
        }
        shortcutsEnabled_->setChecked(false);
        resetShortcutState(false);
    }
    updateCalibrationUi();
}

void MainWindow::handleLine(const QString &line)
{
    appendLog(QStringLiteral("RX  %1").arg(line));
    const std::optional<ServoStatus> status = ServoProtocol::parseStatus(line);
    if (status.has_value())
    {
        updateStatus(*status);
    }
    if (line.startsWith(QStringLiteral("CAL_STATE ")))
    {
        bool valid = false;
        const unsigned mask = line.mid(10).toUInt(&valid);
        if (valid && (mask & ~23U) == 0U)
        {
            calibrationReadyMask_ = mask;
            updateCalibrationUi();
        }
    }
    if (line.startsWith(QStringLiteral("REMOTE_STATE ")))
    {
        const auto parts = line.split(QLatin1Char(' '), Qt::SkipEmptyParts);
        if (parts.size() == 5)
        {
            std::array<unsigned, 4> values{};
            bool valid = true;
            for (int index = 0; index < 4; ++index)
            {
                bool parsed = false;
                values[index] = parts[index + 1].toUInt(&parsed);
                valid = valid && parsed;
            }
            if (valid && values[0] <= 1U && values[1] <= 1U && ((values[2] | values[3]) & ~23U) == 0U)
            {
                remoteEnabled_ = values[0] != 0U;
                remoteArmed_ = values[1] != 0U;
                remoteReadyMask_ = values[2];
                remoteRecoveryMask_ = values[3];
                updateCalibrationUi();
            }
        }
    }
}

void MainWindow::updateCalibrationUi()
{
    for (int axis : {0, 1, 2, 4})
    {
        auto &controls = positionControls_[axis];
        if (controls.motion == nullptr) continue;
        const int id = axis == 4 ? 8 : axis + 1;
        const bool available = serialController_->isOpen() &&
            QDateTime::currentMSecsSinceEpoch() - lastStatusTime_ <= 500 &&
            (lastStatus_.onlineMask & (1U << (id - 1))) != 0U &&
            (lastStatus_.limitMask & (1U << axis)) != 0U;
        const bool ready = available && (calibrationReadyMask_ & (1U << axis)) != 0U;
        controls.prepare->setEnabled(available && !ready);
        controls.motion->setEnabled(ready);
        controls.target->setEnabled(ready);
        controls.speed->setEnabled(ready);
        const bool recovery = (remoteRecoveryMask_ & (1U << axis)) != 0U;
        const bool radio = available && remoteEnabled_ && (remoteReadyMask_ & (1U << axis)) != 0U;
        limitLabels_[axis]->setText(ready ? (recovery ? QStringLiteral("手动标定/仅向内回退") : QStringLiteral("低速手动标定/遥控暂停")) :
            radio ? (recovery ? QStringLiteral("遥控仅向内回退") : remoteArmed_ ? QStringLiteral("限位启用/遥控可用") : QStringLiteral("遥控等待中立1秒")) :
            available ? QStringLiteral("限位启用/待准备遥控") : QStringLiteral("未标定/禁止移动"));
    }
}

void MainWindow::updateStatus(const ServoStatus &status)
{
    lastStatus_ = status;
    lastStatusTime_ = QDateTime::currentMSecsSinceEpoch();
    for (int index = 0; index < 7; ++index)
    {
        const int id = index < 4 ? index + 1 : (index == 4 ? 8 : index);
        const bool online = (status.onlineMask & (1U << (id - 1))) != 0U;
        onlineLabels_[index]->setText(online ? QStringLiteral("在线") : QStringLiteral("离线"));
        onlineLabels_[index]->setStyleSheet(online ? QStringLiteral("color: #15803d; font-weight: bold;")
                                                   : QStringLiteral("color: #b00020; font-weight: bold;"));
    }

    for (int axis = 0; axis < 5; ++axis)
    {
        const int servoId = axis == 4 ? 8 : axis + 1;
        const bool online = (status.onlineMask & (1U << (servoId - 1))) != 0U;
        const QString position = online ? QString::number(status.positions[axis]) : QStringLiteral("--");
        positionLabels_[axis]->setText(position);
        positionControls_[axis].current->setText(position);
        if (!positionControls_[axis].targetInitialized &&
            (status.onlineMask & (1U << (servoId - 1))) != 0U)
        {
            positionControls_[axis].target->setValue(status.positions[axis]);
            if (axis != 3 && !positionControls_[axis].defaultEnabled->isChecked() &&
                (status.limitMask & (1U << axis)) == 0U)
            {
                positionControls_[axis].minimum->setValue(qMax(0, status.positions[axis] - 20));
                positionControls_[axis].maximum->setValue(qMin(4095, status.positions[axis] + 20));
            }
            positionControls_[axis].targetInitialized = true;
        }
        if (axis != 3)
        {
            limitLabels_[axis]->setText((status.limitMask & (1U << axis)) != 0U
                                            ? QStringLiteral("限位已设置/动作锁定")
                                            : QStringLiteral("未标定/禁止移动"));
        }
    }
    limitLabels_[3]->setText(QStringLiteral("不使用"));
    wheelLevelLabels_[0]->setText(QString::number(status.wheelLevels[0]));
    wheelLevelLabels_[1]->setText(QString::number(status.wheelLevels[1]));
    updateCalibrationUi();
}

void MainWindow::appendLog(const QString &text)
{
    if (logEdit_ == nullptr)
    {
        return;
    }
    logEdit_->appendPlainText(QStringLiteral("[%1] %2")
                                  .arg(QDateTime::currentDateTime().toString(QStringLiteral("HH:mm:ss.zzz")), text));
}

void MainWindow::sendCommand(const QString &command)
{
    if (command.isEmpty())
    {
        appendLog(QStringLiteral("命令参数无效，未发送"));
        return;
    }
    if (!serialController_->isOpen())
    {
        appendLog(QStringLiteral("串口未连接，未发送：%1").arg(command));
        return;
    }
    serialController_->sendCommand(command);
}

void MainWindow::sendAllStop()
{
    resetShortcutState(false);
    for (int axis = 0; axis < 5; ++axis)
    {
        sendCommand(ServoProtocol::stop(axis));
    }
    sendCommand(ServoProtocol::stopAllWheels());
}

void MainWindow::stepAxis(int axis, int delta)
{
    const int id = axis == 4 ? 8 : axis + 1;
    if (!serialController_->isOpen() ||
        QDateTime::currentMSecsSinceEpoch() - lastStatusTime_ > 500 ||
        (lastStatus_.onlineMask & (1U << (id - 1))) == 0U ||
        (lastStatus_.limitMask & (1U << axis)) == 0U)
    {
        appendLog(QStringLiteral("步进需要最新在线状态和已设置限位。"));
        return;
    }
    if ((calibrationReadyMask_ & (1U << axis)) == 0U) return;
    const int target = qBound(0, lastStatus_.positions[axis] + delta, 4095);
    positionControls_[axis].target->setValue(target);
    sendCommand(ServoProtocol::move(axis, target, positionControls_[axis].speed->value()));
}

void MainWindow::resetShortcutState(bool sendStop)
{
    steeringDirection_ = 0;
    steeringRefreshTimer_->stop();
    bothForwardPressed_ = false;
    bothReversePressed_ = false;
    leftForwardPressed_ = false;
    leftReversePressed_ = false;
    rightForwardPressed_ = false;
    rightReversePressed_ = false;
    lastLeftLogicalSpeed_ = 0;
    lastRightLogicalSpeed_ = 0;
    if (sendStop && serialController_->isOpen())
    {
        serialController_->sendShortcutStop();
    }
}

void MainWindow::refreshSteeringJog()
{
    if (steeringDirection_ == 0 || !serialController_->isOpen())
    {
        return;
    }
    sendCommand(ServoProtocol::jog(3, steeringDirection_, steeringJogSpeedSpin_->value()));
}

int MainWindow::shortcutWheelSpeed() const
{
    return wheelSpeedSpin_->value();
}

void MainWindow::updateWheelShortcutOutput()
{
    const auto direction = [](bool positive, bool negative) {
        if (positive == negative)
        {
            return 0;
        }
        return positive ? 1 : -1;
    };

    const int common = direction(bothForwardPressed_, bothReversePressed_);
    const int leftOverride = direction(leftForwardPressed_, leftReversePressed_);
    const int rightOverride = direction(rightForwardPressed_, rightReversePressed_);
    const int speed = shortcutWheelSpeed();
    const int leftLogical = (leftOverride != 0 ? leftOverride : common) * speed;
    const int rightLogical = (rightOverride != 0 ? rightOverride : common) * speed;

    if (leftLogical != lastLeftLogicalSpeed_)
    {
        sendCommand(ServoProtocol::logicalWheel(0, leftLogical));
        lastLeftLogicalSpeed_ = leftLogical;
    }
    if (rightLogical != lastRightLogicalSpeed_)
    {
        sendCommand(ServoProtocol::logicalWheel(1, rightLogical));
        lastRightLogicalSpeed_ = rightLogical;
    }
}

bool MainWindow::handleShortcutKey(int key, bool pressed)
{
    if (key == Qt::Key_Space && pressed)
    {
        sendAllStop();
        return true;
    }

    if (!shortcutsEnabled_->isChecked() || !serialController_->isOpen())
    {
        return false;
    }

    switch (key)
    {
    case Qt::Key_Left:
        steeringDirection_ = pressed ? -1 : (steeringDirection_ == -1 ? 0 : steeringDirection_);
        break;
    case Qt::Key_Right:
        steeringDirection_ = pressed ? 1 : (steeringDirection_ == 1 ? 0 : steeringDirection_);
        break;
    case Qt::Key_W:
        bothForwardPressed_ = pressed;
        updateWheelShortcutOutput();
        return true;
    case Qt::Key_S:
        bothReversePressed_ = pressed;
        updateWheelShortcutOutput();
        return true;
    case Qt::Key_Q:
        leftForwardPressed_ = pressed;
        updateWheelShortcutOutput();
        return true;
    case Qt::Key_A:
        leftReversePressed_ = pressed;
        updateWheelShortcutOutput();
        return true;
    case Qt::Key_E:
        rightForwardPressed_ = pressed;
        updateWheelShortcutOutput();
        return true;
    case Qt::Key_D:
        rightReversePressed_ = pressed;
        updateWheelShortcutOutput();
        return true;
    default:
        return false;
    }

    if (steeringDirection_ != 0)
    {
        refreshSteeringJog();
        steeringRefreshTimer_->start();
    }
    else
    {
        steeringRefreshTimer_->stop();
        sendCommand(ServoProtocol::stop(3));
    }
    return true;
}

bool MainWindow::eventFilter(QObject *watched, QEvent *event)
{
    Q_UNUSED(watched)
    if (event->type() == QEvent::ApplicationDeactivate || event->type() == QEvent::WindowDeactivate)
    {
        resetShortcutState(true);
        return QMainWindow::eventFilter(watched, event);
    }

    if (event->type() != QEvent::KeyPress && event->type() != QEvent::KeyRelease)
    {
        return QMainWindow::eventFilter(watched, event);
    }

    auto *keyEvent = static_cast<QKeyEvent *>(event);
    if (keyEvent->isAutoRepeat())
    {
        return false;
    }

    QWidget *focus = QApplication::focusWidget();
    if (qobject_cast<QLineEdit *>(focus) != nullptr || qobject_cast<QAbstractSpinBox *>(focus) != nullptr)
    {
        return false;
    }

    return handleShortcutKey(keyEvent->key(), event->type() == QEvent::KeyPress) ||
           QMainWindow::eventFilter(watched, event);
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    resetShortcutState(true);
    serialController_->close();
    QMainWindow::closeEvent(event);
}
