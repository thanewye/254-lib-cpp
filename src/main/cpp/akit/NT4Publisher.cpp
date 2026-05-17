#include <networktables/NetworkTableInstance.h>
#include <wpi/json.h>

#include "NT4Publisher.h"

namespace akit::networktables {
    NT4Publisher::NT4Publisher() {
        akitTable_ = nt::NetworkTableInstance::GetDefault().GetTable("/AdvantageKit");
        timestampPublisher_ = akitTable_->GetIntegerTopic(timestampKey.substr(1)).Publish(nt::PubSubOptions{
            .sendAll = true
        });
    }

    void NT4Publisher::PutTable(const LogTable& table) {
        const int64_t timestamp = table.GetTimestamp();
        const auto& values = table.GetAll();

        timestampPublisher_.Set(timestamp, timestamp);

        for (const auto& [key, value] : values) {
            if (key.starts_with("/.schema/")) continue;
            auto oldValue = lastStorage_.values.find(key);
            if (oldValue != lastStorage_.values.end() && value == oldValue->second) continue;

            auto& publisher = GetOrCreatePublisher(key, value);

            if (value.unitStr.has_value() && !value.unitStr->empty()) {
                auto existing = units_.find(key);
                if (existing == units_.end() || existing->second != *value.unitStr) {
                    akitTable_->GetTopic(key).SetProperty("unit", "\"" + *value.unitStr + "\"");
                    units_.insert_or_assign(key, *value.unitStr);
                }
            }

            std::visit(
                [&]<typename T_>(const T_& typedValue) {
                    using T = std::decay_t<T_>;
                    if constexpr (std::is_same_v<T, bool>)
                        publisher.SetBoolean(typedValue, timestamp);
                    else if constexpr (std::is_same_v<T, int64_t>)
                        publisher.SetInteger(typedValue, timestamp);
                    else if constexpr (std::is_same_v<T, float>)
                        publisher.SetFloat(typedValue, timestamp);
                    else if constexpr (std::is_same_v<T, double>)
                        publisher.SetDouble(typedValue, timestamp);
                    else if constexpr (std::is_same_v<T, std::string>)
                        publisher.SetString(typedValue, timestamp);
                    else if constexpr (std::is_same_v<T, std::vector<uint8_t>>)
                        publisher.SetRaw(typedValue, timestamp);
                    else if constexpr (std::is_same_v<T, std::vector<int>>)
                        publisher.SetBooleanArray(typedValue, timestamp);
                    else if constexpr (std::is_same_v<T, std::vector<int64_t>>)
                        publisher.SetIntegerArray(typedValue, timestamp);
                    else if constexpr (std::is_same_v<T, std::vector<float>>)
                        publisher.SetFloatArray(typedValue, timestamp);
                    else if constexpr (std::is_same_v<T, std::vector<double>>)
                        publisher.SetDoubleArray(typedValue, timestamp);
                    else if constexpr (std::is_same_v<T, std::vector<std::string>>)
                        publisher.SetStringArray(typedValue, timestamp);
                }, value.value
            );
        }
        lastStorage_.values = values;
        lastStorage_.timestamp = timestamp;
    }

    std::string NT4Publisher::GetNT4Type(const LogValue& val) const {
        return val.GetNT4Type();
    }

    nt::GenericPublisher& NT4Publisher::GetOrCreatePublisher(const std::string& key, const LogValue& value) {
        auto publisher = publishers_.find(key);

        // publisher does not exist, create
        if (publisher == publishers_.end()) {
            nt::Topic topic = akitTable_->GetTopic(key);
            auto unit = value.unitStr;

            // construct publisher within the emplace (black magic)
            auto [createdPublisher, inserted] = publishers_.emplace(
                key,
                topic.GenericPublish(GetNT4Type(value), nt::PubSubOptions{.sendAll = true})
            );

            if (unit.has_value() && !unit->empty()) {
                topic.SetProperty("unit", "\"" + unit.value() + "\"");
                units_.emplace(key, *unit);
            }

            return createdPublisher->second;
        }
        return publisher->second;
    }
}
