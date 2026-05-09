#pragma once

class LatchedBoolean {
public:
    bool Update(bool value) {
        const bool latched = value && !m_last;
        m_last = value;
        return latched;
    }

private:
    bool m_last = false;
};
