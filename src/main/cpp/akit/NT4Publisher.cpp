#include "NT4Publisher.h"
#include <networktables/NetworkTableInstance.h>

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
            auto oldValue = lastStorage_.values.find(key);
            if (oldValue != lastStorage_.values.end() && value == oldValue->second) continue;

            auto& publisher = GetOrCreatePublisher(key, value);

            std::visit(
                [&]<typename T_>(const T_& typedValue) {
                    using T = std::decay_t<T_>;
                    if constexpr (std::is_same_v<T, bool>) {
                        publisher.SetBoolean(typedValue, timestamp);
                    } else if constexpr (std::is_same_v<T, int64_t>) {
                        publisher.SetInteger(typedValue, timestamp);
                    } else if constexpr (std::is_same_v<T, double>) {
                        publisher.SetDouble(typedValue, timestamp);
                    } else if constexpr (std::is_same_v<T, std::string>) {
                        publisher.SetString(typedValue, timestamp);
                    } else if constexpr (std::is_same_v<T, std::vector<double>>) {
                        publisher.SetDoubleArray(typedValue, timestamp);
                    }
                }, value
            );
        }
        lastStorage_.values = values;
        lastStorage_.timestamp = timestamp;
    }
    std::string NT4Publisher::GetNT4Type(const LogValue& val) const {
        return std::visit([]<typename T>(const T&) -> std::string {
            if constexpr (std::is_same_v<T, bool>) {
                return "boolean";
            } else if constexpr (std::is_same_v<T, int64_t>) {
                return "int";
            } else if constexpr (std::is_same_v<T, double>) {
                return "double";
            } else if constexpr (std::is_same_v<T, std::string>) {
                return "string";
            } else if constexpr (std::is_same_v<T, std::vector<double>>) {
                return "double[]";
            } else {
                return "";
            }
        }, val);
    }

    nt::GenericPublisher &NT4Publisher::GetOrCreatePublisher(const std::string &key, const LogValue &value) {
        auto publisher = publishers_.find(key);
        if (publisher == publishers_.end()) {
            nt::Topic topic = akitTable_->GetTopic(key);
            auto [createdPublisher, inserted] = publishers_.emplace(
                key,
                topic.GenericPublish(GetNT4Type(value), nt::PubSubOptions{.sendAll = true})
            );
            return createdPublisher->second;
        }
        return publisher->second;
    }

}
