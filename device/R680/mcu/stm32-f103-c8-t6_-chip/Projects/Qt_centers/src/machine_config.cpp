#include "machine_config.h"

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>

namespace
{
constexpr int kPositionMaximum = 4095;

bool readRequiredInt(const QJsonObject &object,
                     const QString &key,
                     int minimum,
                     int maximum,
                     int *value,
                     QString *errorMessage)
{
    const QJsonValue jsonValue = object.value(key);
    if (!jsonValue.isDouble())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("字段 %1 必须是整数").arg(key);
        }
        return false;
    }

    const int parsed = jsonValue.toInt(minimum - 1);
    if (parsed < minimum || parsed > maximum)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("字段 %1 超出范围 %2..%3")
                                .arg(key)
                                .arg(minimum)
                                .arg(maximum);
        }
        return false;
    }
    *value = parsed;
    return true;
}

bool parseAxis(const QJsonObject &object,
               int expectedAxis,
               ServoLimitProfile *result,
               QString *errorMessage)
{
    if (!readRequiredInt(object, QStringLiteral("servo_id"), 1, 8, &result->servoId, errorMessage) ||
        !readRequiredInt(object, QStringLiteral("axis"), 0, 4, &result->axis, errorMessage) ||
        !readRequiredInt(object, QStringLiteral("minimum"), 0, kPositionMaximum,
                         &result->minimum, errorMessage) ||
        !readRequiredInt(object, QStringLiteral("maximum"), 0, kPositionMaximum,
                         &result->maximum, errorMessage))
    {
        return false;
    }

    result->name = object.value(QStringLiteral("name")).toString();
    const int expectedId = expectedAxis == 4 ? 8 : expectedAxis + 1;
    if (result->axis != expectedAxis || result->servoId != expectedId ||
        !object.value(QStringLiteral("enabled")).isBool())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("限位须按轴0/1/2/4、ID1/2/3/8排列，并包含布尔值enabled");
        }
        return false;
    }
    result->enabled = object.value(QStringLiteral("enabled")).toBool();
    if (object.contains(QStringLiteral("reference_mode")) &&
        !readRequiredInt(object, QStringLiteral("reference_mode"), 0, 1, &result->referenceMode, errorMessage)) return false;
    if (object.contains(QStringLiteral("encoder_offset")) &&
        !readRequiredInt(object, QStringLiteral("encoder_offset"), 0, 2047, &result->encoderOffset, errorMessage)) return false;
    if (result->referenceMode == 1 && result->minimum < result->encoderOffset) return false;
    if (result->minimum >= result->maximum)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("轴 %1 的 minimum 必须小于 maximum").arg(expectedAxis);
        }
        return false;
    }
    return true;
}

QJsonObject axisToJson(const ServoLimitProfile &axis)
{
    return {
        {QStringLiteral("servo_id"), axis.servoId},
        {QStringLiteral("axis"), axis.axis},
        {QStringLiteral("name"), axis.name},
        {QStringLiteral("minimum"), axis.minimum},
        {QStringLiteral("maximum"), axis.maximum},
        {QStringLiteral("enabled"), axis.enabled},
        {QStringLiteral("reference_mode"), axis.referenceMode},
        {QStringLiteral("encoder_offset"), axis.encoderOffset},
    };
}

QJsonObject profileToJson(const MachineProfile &profile)
{
    QJsonArray limits;
    for (const ServoLimitProfile &limit : profile.limits)
    {
        limits.append(axisToJson(limit));
    }

    return {
        {QStringLiteral("id"), profile.id},
        {QStringLiteral("assembly_version"), profile.assemblyVersion},
        {QStringLiteral("machine_number"), profile.machineNumber},
        {QStringLiteral("description"), profile.description},
        {QStringLiteral("limits"), limits},
    };
}
}

bool MachineConfigRepository::load(const QString &filePath, QString *errorMessage)
{
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly))
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("无法打开配置文件 %1：%2").arg(filePath, file.errorString());
        }
        return false;
    }

    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(file.readAll(), &parseError);
    if (parseError.error != QJsonParseError::NoError || !document.isObject())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("JSON 格式错误：%1").arg(parseError.errorString());
        }
        return false;
    }

    const QJsonObject root = document.object();
    if (root.value(QStringLiteral("schema_version")).toInt() != 2)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("需要schema_version 2；旧铲斗配置不可用于新的夹爪俯仰机构");
        }
        return false;
    }

    const QJsonArray profileArray = root.value(QStringLiteral("profiles")).toArray();
    if (profileArray.isEmpty())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("profiles 不能为空");
        }
        return false;
    }

    QVector<MachineProfile> loadedProfiles;
    QStringList loadedIds;
    for (const QJsonValue &profileValue : profileArray)
    {
        const QJsonObject object = profileValue.toObject();
        MachineProfile profile;
        profile.id = object.value(QStringLiteral("id")).toString().trimmed();
        profile.assemblyVersion = object.value(QStringLiteral("assembly_version")).toString();
        profile.machineNumber = object.value(QStringLiteral("machine_number")).toString();
        profile.description = object.value(QStringLiteral("description")).toString();
        if (profile.id.isEmpty() || loadedIds.contains(profile.id))
        {
            if (errorMessage != nullptr)
            {
                *errorMessage = QStringLiteral("配置 ID 为空或重复：%1").arg(profile.id);
            }
            return false;
        }

        const QJsonArray limits = object.value(QStringLiteral("limits")).toArray();
        if (limits.size() != 4)
        {
            if (errorMessage != nullptr)
            {
                *errorMessage = QStringLiteral("配置 %1 必须包含ID1/2/3/8四个位置轴").arg(profile.id);
            }
            return false;
        }
        for (int index = 0; index < 4; ++index)
        {
            const int axis = index == 3 ? 4 : index;
            if (!parseAxis(limits.at(index).toObject(), axis, &profile.limits[index], errorMessage))
            {
                return false;
            }
        }

        loadedIds.append(profile.id);
        loadedProfiles.append(profile);
    }

    const QString active = root.value(QStringLiteral("active_profile")).toString();
    if (!loadedIds.contains(active))
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("active_profile 不存在：%1").arg(active);
        }
        return false;
    }

    filePath_ = filePath;
    activeProfileId_ = active;
    profiles_ = loadedProfiles;
    return true;
}

bool MachineConfigRepository::save(QString *errorMessage) const
{
    if (filePath_.isEmpty())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("尚未加载配置文件");
        }
        return false;
    }

    QJsonArray profiles;
    for (const MachineProfile &profile : profiles_)
    {
        profiles.append(profileToJson(profile));
    }
    const QJsonObject root = {
        {QStringLiteral("schema_version"), 2},
        {QStringLiteral("active_profile"), activeProfileId_},
        {QStringLiteral("profiles"), profiles},
    };

    QSaveFile file(filePath_);
    if (!file.open(QIODevice::WriteOnly) ||
        file.write(QJsonDocument(root).toJson(QJsonDocument::Indented)) < 0 ||
        !file.commit())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("保存配置失败：%1").arg(file.errorString());
        }
        return false;
    }
    return true;
}

QString MachineConfigRepository::filePath() const
{
    return filePath_;
}

QString MachineConfigRepository::activeProfileId() const
{
    return activeProfileId_;
}

void MachineConfigRepository::setActiveProfileId(const QString &profileId)
{
    if (profileIds().contains(profileId))
    {
        activeProfileId_ = profileId;
    }
}

QStringList MachineConfigRepository::profileIds() const
{
    QStringList ids;
    for (const MachineProfile &profile : profiles_)
    {
        ids.append(profile.id);
    }
    return ids;
}

std::optional<MachineProfile> MachineConfigRepository::profile(const QString &profileId) const
{
    for (const MachineProfile &profile : profiles_)
    {
        if (profile.id == profileId)
        {
            return profile;
        }
    }
    return std::nullopt;
}

bool MachineConfigRepository::updateProfile(const MachineProfile &profile)
{
    for (MachineProfile &existing : profiles_)
    {
        if (existing.id == profile.id)
        {
            existing = profile;
            return true;
        }
    }
    return false;
}
