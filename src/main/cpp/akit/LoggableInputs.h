// LoggableInputs but using TMP for no vtable lookup
#pragma once

#include <string>
#include <utility>

#include <boost/pfr.hpp>

#include "akit/LogTable.h"

namespace akit {
    namespace magic_log {
        template<typename T, size_t... Is>
        void to_log_impl(const T& obj, LogTable& table, std::index_sequence<Is...>) {
            (table.Put(std::string(boost::pfr::get_name<Is, T>()),
                       boost::pfr::get<Is>(obj)), ...);
        }

        template<typename T, size_t... Is>
        void from_log_impl(T& obj, const LogTable& table, std::index_sequence<Is...>) {
            T defaults{};
            ([&] {
                auto& field = boost::pfr::get<Is>(obj);
                field = table.Get(boost::pfr::get_name<Is, T>(),
                                  boost::pfr::get<Is>(defaults));
            }(), ...);
        }
    } // namespace detail

    template<typename T>
    concept LoggableAggregate =
            std::is_aggregate_v<T> && !std::is_array_v<T>;

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
} // namespace akit
