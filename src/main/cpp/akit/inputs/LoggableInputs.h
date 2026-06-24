// LoggableInputs but using TMP for no vtable lookup that would come with using java interface style
#pragma once

#include <cctype>
#include <concepts>
#include <string>
#include <string_view>
#include <utility>

#include <boost/pfr.hpp>

#include "akit/log/LogTable.h"

namespace akit {
    namespace magic_log {
        inline std::string CapitalizeFirst(std::string_view name) {
            std::string result(name);
            if (!result.empty()) {
                result[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(result[0])));
            }
            return result;
        }

        template<typename T, size_t... Is>
        void to_log_impl(const T& obj, LogTable& table, std::index_sequence<Is...>) {
            (table.Put(CapitalizeFirst(boost::pfr::get_name<Is, T>()),
                       boost::pfr::get<Is>(obj)), ...);
        }

        template<typename T, size_t... Is>
        void from_log_impl(T& obj, const LogTable& table, std::index_sequence<Is...>) {
            T defaults{};
            ([&] {
                auto& field = boost::pfr::get<Is>(obj);
                field = table.Get(CapitalizeFirst(boost::pfr::get_name<Is, T>()),
                                  boost::pfr::get<Is>(defaults));
            }(), ...);
        }
    } // namespace magic_log

    template<typename T>
    concept LoggableAggregate =
            std::is_aggregate_v<T> && !std::is_array_v<T> && !std::derived_from<T, LoggableInputs>;

    template<LoggableAggregate T>
    void ToLog(const T& obj, LogTable& table) {
        magic_log::to_log_impl(obj, table,
                               std::make_index_sequence<boost::pfr::tuple_size_v<T>>{});
    }

    template<LoggableAggregate T>
    void FromLog(T& obj, const LogTable& table) {
        magic_log::from_log_impl(obj, table,
                                 std::make_index_sequence<boost::pfr::tuple_size_v<T>>{});
    }

    template<typename T>
        requires std::is_aggregate_v<T> && (!std::is_array_v<T>) && (!wpi::StructSerializable<T>)
            && (!std::derived_from<T, LoggableInputs>)
    void LogTable::Put(const string& key, const T& value) const {
        auto subtable = GetSubtable(key);
        ToLog(value, subtable);
    }

    template<typename T>
        requires std::is_aggregate_v<T> && (!std::is_array_v<T>) && (!wpi::StructSerializable<T>)
            && (!std::derived_from<T, LoggableInputs>)
    T LogTable::Get(string_view key, T defaultValue) const {
        auto subtable = GetSubtable(key);
        FromLog(defaultValue, subtable);
        return defaultValue;
    }
} // namespace akit
