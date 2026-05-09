#pragma once

class LatchedBoolean {
 public:
  bool Update(bool value);

private:
    bool m_last = false;
};
