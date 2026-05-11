#pragma once

#include <string>
#include <unordered_map>

class ControllerMapping {
private:
    std::unordered_map<std::string, int> buttonMap;
    std::unordered_map<std::string, int> axesMap;

public:
    ControllerMapping(std::unordered_map<std::string, int> buttons,
                      std::unordered_map<std::string, int> axes);

    [[nodiscard]] int GetButton(const std::string& name) const {
        return buttonMap.at(name);
    }

    [[nodiscard]] int GetAxis(const std::string& name) const {
        return axesMap.at(name);
    }

};
