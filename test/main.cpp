#include <cassert>
#include <iostream>

#include "akit/Logger.h"

struct DriveInputs : public akit::LoggableInputs {
  double position = 0.0;
  bool connected = false;

  void ToLog(akit::LogTable& table) const override {
    table.Put("Position", position);
    table.Put("Connected", connected);
  }

  void FromLog(const akit::LogTable& table) override {
    position = table.Get("Position", position);
    connected = table.Get("Connected", connected);
  }
};

int main() {
  akit::Logger::Clear();

  DriveInputs inputs;
  inputs.position = 3.14;
  inputs.connected = true;

  akit::Logger::SetReplayMode(false);
  akit::Logger::ProcessInputs("Drive", inputs);

  const auto& table = akit::Logger::GetCurrentTable();

  assert(table.Get("Position", 0.0) == 3.14);
  assert(table.Get("Connected", false) == true);

  inputs.position = 0.0;
  inputs.connected = false;

  akit::Logger::SetReplayMode(true);
  akit::Logger::ProcessInputs("Drive", inputs);

  assert(inputs.position == 3.14);
  assert(inputs.connected == true);

  std::cout << "All tests passed\n";
}
