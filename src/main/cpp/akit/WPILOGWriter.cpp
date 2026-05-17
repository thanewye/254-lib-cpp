#include <algorithm>
#include <cctype>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <utility>

#include <frc/Errors.h>
#include <frc/RobotBase.h>
#include <frc/RobotController.h>

#include "Logger.h"
#include "WPILOGConstants.h"
#include "WPILOGWriter.h"

namespace akit::wpilog {
    LoggableType WPILOGWriter::GetType(const LogValue& value) const {
        return value.type;
    }

    void WPILOGWriter::AppendValue(const int64_t entryID, const LogValue& lv, const int64_t timestamp) {
        std::visit([this, entryID, timestamp]<typename T>(const T& v) {
            if constexpr (std::is_same_v<T, bool>)
                log_->AppendBoolean(entryID, v, timestamp);
            else if constexpr (std::is_same_v<T, int64_t>)
                log_->AppendInteger(entryID, v, timestamp);
            else if constexpr (std::is_same_v<T, float>)
                log_->AppendFloat(entryID, v, timestamp);
            else if constexpr (std::is_same_v<T, double>)
                log_->AppendDouble(entryID, v, timestamp);
            else if constexpr (std::is_same_v<T, std::string>)
                log_->AppendString(entryID, v, timestamp);
            else if constexpr (std::is_same_v<T, std::vector<uint8_t>>)
                log_->AppendRaw(entryID, v, timestamp);
            else if constexpr (std::is_same_v<T, std::vector<int>>)
                log_->AppendBooleanArray(entryID, v, timestamp);
            else if constexpr (std::is_same_v<T, std::vector<int64_t>>)
                log_->AppendIntegerArray(entryID, v, timestamp);
            else if constexpr (std::is_same_v<T, std::vector<float>>)
                log_->AppendFloatArray(entryID, v, timestamp);
            else if constexpr (std::is_same_v<T, std::vector<double>>)
                log_->AppendDoubleArray(entryID, v, timestamp);
            else if constexpr (std::is_same_v<T, std::vector<std::string>>)
                log_->AppendStringArray(entryID, v, timestamp);
        }, lv.value);
    }

    WPILOGWriter::WPILOGWriter(const std::string& path, AdvantageScopeOpenBehavior openBehavior) {
        openBehavior_ = openBehavior;
        std::mt19937 generator(std::random_device{}());
        std::uniform_int_distribution<unsigned int> distribution(0, 0xFFFF);
        for (int i = 0; i < 4; ++i) {
            std::ostringstream stream;
            stream << std::hex << std::setw(4) << std::setfill('0') << distribution(generator);
            randomIdentifier_ += stream.str();
        }
        if (path.ends_with(".wpilog")) {
            std::filesystem::path pathFile(path);
            folder_ = pathFile.parent_path().string();
            fileName_ = pathFile.filename().string();
            autoRename_ = false;
        } else {
            folder_ = path;
            fileName_ = "akit" + randomIdentifier_ + ".wpilog";
            autoRename_ = true;
        }
    }

    WPILOGWriter::WPILOGWriter(const std::string& path) : WPILOGWriter(path, AUTO) {}

    WPILOGWriter::WPILOGWriter(AdvantageScopeOpenBehavior openBehavior)
        : WPILOGWriter(frc::RobotBase::IsSimulation() ? defaultPathSim_ : defaultPathRio_, openBehavior) {}

    WPILOGWriter::WPILOGWriter()
        : WPILOGWriter(frc::RobotBase::IsSimulation() ? defaultPathSim_ : defaultPathRio_, AUTO) {}

    void WPILOGWriter::Start() {
        namespace fs = std::filesystem;
        fs::create_directory(folder_);
        if (const fs::path logFile = fs::path(folder_) / fileName_; fs::exists(logFile)) fs::remove(logFile);

        const std::string logPath = (fs::path(folder_) / fileName_).string();
        std::cout << "[AdvantageKit] Logging to \"" << logPath << "\"\n";

        std::error_code ec;
        log_ = std::make_unique<wpi::log::DataLogWriter>(logPath, ec, WPILOGConstants::extraHeader);
        if (ec) {
            FRC_ReportError(frc::err::Error, "[AdvantageKit] Failed to open output log file.");
            return;
        }
        isOpen_ = true;
        timestampID_ = log_->Start(
            timestampKey, GetWPILOGType(LoggableType::kInteger), WPILOGConstants::entryMetadata, 0);
        lastStorage_ = LogStorage();

        // reset data
        entryIDs.clear();
        entryTypes.clear();
        dsAttachedTime_ = std::nullopt;
        logDate_ = std::nullopt;
        logMatchText_ = std::nullopt;
    }

    void WPILOGWriter::End() {
        if (!isOpen_ || !log_) return;

        log_->Stop();
        isOpen_ = false;
        bool shouldOpenAscope = false;
        switch (openBehavior_) {
            case ALWAYS:
                shouldOpenAscope = frc::RobotBase::IsSimulation();
                break;
            case AUTO:
                shouldOpenAscope = frc::RobotBase::IsSimulation() && Logger::HasReplaySource();
                break;
            case NEVER:
                shouldOpenAscope = false;
                break;
        }
        if (shouldOpenAscope) {
            try {
                namespace fs = std::filesystem;
                std::string fullLogPath = fs::absolute(fs::path(folder_) / fileName_).lexically_normal().string();

                fs::path tempPath = fs::temp_directory_path() / ascopeFileName_;
                std::ofstream writer(tempPath);
                if (!writer) throw std::runtime_error("could not open file");
                writer << fullLogPath << "\n";

                std::cout << "[AdvantageKit] Log sent to AdvantageScope\n";
            } catch (const std::exception& e) {
                FRC_ReportError(frc::err::Error, "[AdvantageKit] Failed to send log to AdvantageScope.");
            }
        }
    }

    void WPILOGWriter::PutTable(const LogTable& table) {
        if (!isOpen_) return;
        const int64_t timestamp = table.GetTimestamp();
        if (autoRename_) {
            if (!logDate_.has_value()) {
                if (table.Get("DriverStation/DSAttached", false) && table.Get("SystemStats/SystemTimeValid", false) ||
                    frc::RobotBase::IsSimulation()) {
                    if (!dsAttachedTime_.has_value()) {
                        dsAttachedTime_ = static_cast<double>(frc::RobotController::GetFPGATime()) / 1000000.0;
                    } else if (static_cast<double>(frc::RobotController::GetFPGATime()) / 1000000.0 - dsAttachedTime_.
                               value() > timestampUpdateDelay_ || frc::RobotBase::IsSimulation()) {
                        const auto now = std::chrono::system_clock::now();
                        std::time_t t = std::chrono::system_clock::to_time_t(now);
                        std::tm localTime{};
                        localtime_r(&t, &localTime);
                        logDate_ = localTime;
                    }
                } else {
                    dsAttachedTime_ = std::nullopt;
                }
            }
            HAL_MatchType matchType;
            switch (table.Get("DriverStation/MatchType", 0LL)) {
                case 1:
                    matchType = HAL_kMatchType_practice;
                    break;
                case 2:
                    matchType = HAL_kMatchType_qualification;
                    break;
                case 3:
                    matchType = HAL_kMatchType_elimination;
                    break;
                default:
                    matchType = HAL_kMatchType_none;
                    break;
            }
            if (!logMatchText_.has_value() && matchType != HAL_kMatchType_none) {
                logMatchText_ = "";
                switch (matchType) {
                    case HAL_kMatchType_practice:
                        logMatchText_ = "p";
                        break;
                    case HAL_kMatchType_qualification:
                        logMatchText_ = "q";
                        break;
                    case HAL_kMatchType_elimination:
                        logMatchText_ = "e";
                        break;
                    default:
                        break;
                }
                *logMatchText_ += std::to_string(table.Get("DriverStation/MatchNumber", 0LL));
            }

            // Update filename
            std::string newFilename = "akit_";
            if (!logDate_.has_value()) {
                newFilename += randomIdentifier_;
            } else {
                std::ostringstream timeStream;
                timeStream << std::put_time(&logDate_.value(), kTimeFormat);
                newFilename += timeStream.str();
            }

            std::string eventName = table.Get("DriverStation/EventName", std::string{});
            std::ranges::transform(eventName
                                   , eventName.begin(),
                                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

            if (!eventName.empty()) {
                newFilename += "_";
                newFilename += eventName;
            }

            if (logMatchText_.has_value() && !logMatchText_->empty()) {
                newFilename += "_";
                newFilename += *logMatchText_;
            }

            newFilename += ".wpilog";

            if (newFilename != fileName_) {
                std::filesystem::path oldPath = std::filesystem::path(folder_) / fileName_;
                std::filesystem::path newPath = std::filesystem::path(folder_) / newFilename;

                std::cout << "[AdvantageKit] Renaming log to \"" << newPath.string() << "\"\n";

                std::error_code ec;
                std::filesystem::rename(oldPath, newPath, ec);
                if (!ec) {
                    fileName_ = newFilename;
                }
            }
        }

        log_->AppendInteger(timestampID_, timestamp, timestamp);
        const auto& newMap = table.GetAll();
        for (const auto& [key, value] : newMap) {
            const auto existingID = entryIDs.find(key);
            bool appendData = false;

            if (existingID == entryIDs.end()) {
                const std::string metadata = value.unitStr.has_value()
                                                 ? [&] {
                                                     std::string s = WPILOGConstants::entryMetadataUnits;
                                                     s.replace(s.find("$UNITSTR"), 8, *value.unitStr);
                                                     return s;
                                                 }()
                                                 : WPILOGConstants::entryMetadata;
                entryIDs.emplace(key, log_->Start(key, value.GetWPILOGType(), metadata, timestamp));
                entryTypes.emplace(key, value.type);
                appendData = true;
            } else {
                auto oldValue = lastStorage_.values.find(key);
                appendData = oldValue == lastStorage_.values.end() || oldValue->second != value;
            }

            if (appendData) {
                AppendValue(entryIDs.at(key), value, timestamp);
            }
        }

        log_->Flush();
        lastStorage_.values = newMap;
        lastStorage_.timestamp = timestamp;
    }
}
