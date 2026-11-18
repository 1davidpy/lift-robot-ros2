#ifndef MACHINE_CONFIG_H
#define MACHINE_CONFIG_H

#include <array>
#include <optional>

#include <QString>
#include <QStringList>
#include <QVector>

struct ServoLimitProfile
{
    int servoId = 0;
    int axis = 0;
    QString name;
    int minimum = 0;
    int maximum = 4095;
    bool enabled = false;
    int referenceMode = 0;
    int encoderOffset = 0;
};

struct MachineProfile
{
    QString id;
    QString assemblyVersion;
    QString machineNumber;
    QString description;
    std::array<ServoLimitProfile, 4> limits{}; // Axes 0, 1, 2, 4.
};

class MachineConfigRepository
{
public:
    bool load(const QString &filePath, QString *errorMessage);
    bool save(QString *errorMessage) const;

    QString filePath() const;
    QString activeProfileId() const;
    void setActiveProfileId(const QString &profileId);
    QStringList profileIds() const;
    std::optional<MachineProfile> profile(const QString &profileId) const;
    bool updateProfile(const MachineProfile &profile);

private:
    QString filePath_;
    QString activeProfileId_;
    QVector<MachineProfile> profiles_;
};

#endif
