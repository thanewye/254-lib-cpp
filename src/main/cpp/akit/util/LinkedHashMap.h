#pragma once

#include <list>
#include <unordered_map>
#include <utility>

namespace akit::util {
    template<typename key_, typename value_> class LinkedHashMap {
    public:
        void put(const key_& key, value_ value) {
            auto it = index_.find(key);
            if (it != index_.end()) {
                it->second->second = std::move(value);
                return;
            }
            items_.emplace_back(key, std::move(value));
            index_[key] = std::prev(items_.end());
        }

        value_* get(const key_& key) {
            auto it = index_.find(key);
            return it == index_.end() ? nullptr : &it->second->second;
        }

        const value_* get(const key_& key) const {
            auto it = index_.find(key);
            return it == index_.end() ? nullptr : &it->second->second;
        }

        bool erase(const key_& key) {
            auto it = index_.find(key);
            if (it == index_.end()) return false;
            items_.erase(it->second);
            index_.erase(it);
            return true;
        }

        bool contains(const key_& key) const { return index_.contains(key); }

        std::size_t size() const { return items_.size(); }

        std::list<std::pair<key_, value_>>::iterator begin() { return items_.begin(); }
        std::list<std::pair<key_, value_>>::iterator end() { return items_.end(); }

        std::list<std::pair<key_, value_>>::const_iterator begin() const { return items_.begin(); }
        std::list<std::pair<key_, value_>>::const_iterator end() const { return items_.end(); }

        std::list<std::pair<key_, value_>>::const_iterator cbegin() const { return items_.cbegin(); }
        std::list<std::pair<key_, value_>>::const_iterator cend() const { return items_.cend(); }

    private:
        std::list<std::pair<key_, value_>> items_;
        std::unordered_map<key_, typename std::list<std::pair<key_, value_>>::iterator> index_;
    };
} // namespace akit::util