#pragma once

#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <thread>

#include <frc/Errors.h>

namespace OSUtil {
    inline std::tm LocalTime(std::time_t t) {
        std::tm localTime{};
#ifdef _WIN32
        localtime_s(&localTime, &t);
#else
        localtime_r(&t, &localTime);
#endif
        return localTime;
    }

    inline void FsSync() {
        std::printf("Executing filesystem sync...\n");
        int ret = std::system("sync");
        if (ret != 0) {
            FRC_ReportError(ret, "Failed to manually execute filesystem 'sync' command to flush logs to disk");
        } else {
            std::printf("Done\n");
        }
    }

    inline void FsSyncAsync() {
        std::thread([] { FsSync(); }).detach();
    }
} // namespace OSUtil
