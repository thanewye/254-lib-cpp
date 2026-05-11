#pragma once

#include <cstdio>
#include <cstdlib>
#include <thread>

#include <frc/Errors.h>

namespace OSUtil {
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
