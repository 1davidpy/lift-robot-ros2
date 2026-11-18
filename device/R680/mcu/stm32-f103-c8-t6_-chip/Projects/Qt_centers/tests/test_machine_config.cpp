#include <QCoreApplication>
#include <QTemporaryDir>
#include <QFile>
#include <QTextStream>

#include "machine_config.h"

int main(int argc, char *argv[])
{
    QCoreApplication application(argc, argv);
    QTextStream error(stderr);
    const bool config2Only = application.arguments().size() == 3 && application.arguments().at(2) == QStringLiteral("--config2-only");
    if (application.arguments().size() != 2 && !config2Only)
    {
        error << "Usage: test_machine_config <machines.json>\n";
        return 2;
    }

    MachineConfigRepository repository;
    QString message;
    if (!repository.load(application.arguments().at(1), &message))
    {
        error << message << '\n';
        return 3;
    }
    if (config2Only)
    {
        const auto profile2 = repository.profile(QStringLiteral("Config2"));
        const std::array<int, 4> minimum = {1001, 2223, 1547, 172};
        const std::array<int, 4> maximum = {2950, 4076, 3609, 1292};
        if (!profile2.has_value() || repository.activeProfileId() != QStringLiteral("Config2")) return 14;
        for (int index = 0; index < 4; ++index)
            if (profile2->limits[index].minimum != minimum[index] || profile2->limits[index].maximum != maximum[index] ||
                !profile2->limits[index].enabled || profile2->limits[index].referenceMode != (index < 3 ? 1 : 0) ||
                profile2->limits[index].encoderOffset != 85) return 15;
        QTemporaryDir config2Directory;
        const QString config2Copy = config2Directory.filePath(QStringLiteral("config2.json"));
        if (!QFile::copy(application.arguments().at(1), config2Copy)) return 16;
        MachineConfigRepository saved2, reloaded2;
        if (!saved2.load(config2Copy, &message) || !saved2.save(&message) || !reloaded2.load(config2Copy, &message)) return 17;
        const auto roundTrip2 = reloaded2.profile(QStringLiteral("Config2"));
        if (!roundTrip2.has_value()) return 18;
        for (int index = 0; index < 4; ++index)
            if (roundTrip2->limits[index].referenceMode != (index < 3 ? 1 : 0) || roundTrip2->limits[index].encoderOffset != 85) return 19;
        QTextStream(stdout) << "PASS: Config2 exact limits and enabled flags\n";
        return 0;
    }
    if (repository.activeProfileId() != QStringLiteral("V0.2_No1_reinstall"))
    {
        error << "Unexpected active profile\n";
        return 4;
    }
    const auto installation = repository.profile(repository.activeProfileId());
    if (!installation.has_value()) return 12;
    for (const auto &axis : installation->limits)
        if (axis.enabled) { error << "Reinstallation axes must start uncalibrated\n"; return 13; }

    const auto profile = repository.profile(QStringLiteral("V0.2_No1"));
    if (!profile.has_value() ||
        profile->limits[0].minimum != 1000 ||
        profile->limits[0].maximum != 3000 ||
        profile->limits[1].minimum != 1100 ||
        profile->limits[1].maximum != 3200 ||
        profile->limits[2].enabled || profile->limits[3].enabled ||
        profile->limits[3].axis != 4 || profile->limits[3].servoId != 8)
    {
        error << "V0.2_No1 limit values do not match the defaults\n";
        return 5;
    }

    QTemporaryDir temporaryDirectory;
    const QString copyPath = temporaryDirectory.filePath(QStringLiteral("machines.json"));
    if (!QFile::copy(application.arguments().at(1), copyPath))
    {
        error << "Cannot create temporary configuration copy\n";
        return 6;
    }

    MachineConfigRepository writableRepository;
    if (!writableRepository.load(copyPath, &message))
    {
        error << message << '\n';
        return 7;
    }
    auto updated = writableRepository.profile(QStringLiteral("V0.2_No1"));
    if (!updated.has_value())
    {
        error << "Configuration profile disappeared\n";
        return 8;
    }
    updated->limits[0].maximum = 2999;
    updated->limits[3].minimum = 1700;
    updated->limits[3].maximum = 2100;
    updated->limits[3].enabled = true;
    writableRepository.updateProfile(*updated);
    if (!writableRepository.save(&message))
    {
        error << message << '\n';
        return 9;
    }

    MachineConfigRepository reloaded;
    if (!reloaded.load(copyPath, &message))
    {
        error << "Configuration round-trip failed\n";
        return 10;
    }
    const auto reloadedProfile = reloaded.profile(QStringLiteral("V0.2_No1"));
    if (!reloadedProfile.has_value() || reloadedProfile->limits[0].maximum != 2999 ||
        reloadedProfile->limits[3].minimum != 1700 ||
        reloadedProfile->limits[3].maximum != 2100 || !reloadedProfile->limits[3].enabled)
    {
        error << "Configuration round-trip value mismatch\n";
        return 11;
    }

    QTextStream(stdout) << "PASS: V0.2_No1 loaded and round-trip saved\n";
    return 0;
}
