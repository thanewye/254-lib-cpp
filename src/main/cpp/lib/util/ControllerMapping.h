#pragma once

#include <string>
#include <unordered_map>

class ControllerMapping {
public:
    ControllerMapping(std::unordered_map<std::string, int> buttons, std::unordered_map<std::string, int> axes)
        : buttonMap(buttons)
        , axesMap(axes) {}

    [[nodiscard]] int GetButton(const std::string& name) const { return buttonMap.at(name); }

    [[nodiscard]] int GetAxis(const std::string& name) const { return axesMap.at(name); }

private:
    std::unordered_map<std::string, int> buttonMap;
    std::unordered_map<std::string, int> axesMap;
};
