#pragma once

#include <string>
#include <unordered_map>

class ControllerMapping {
 public:
  ControllerMapping(std::unordered_map<std::string, int> buttons,
                    std::unordered_map<std::string, int> axes);

  int GetButton(const std::string& name) const;
  int GetAxis(const std::string& name) const;

 private:
  std::unordered_map<std::string, int> m_buttons;
  std::unordered_map<std::string, int> m_axes;
};
