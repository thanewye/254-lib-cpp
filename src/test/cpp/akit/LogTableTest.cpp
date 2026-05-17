#include <cstdint>
#include <span>
#include <string>
#include <vector>

#include <frc/geometry/Rotation2d.h>
#include <units/angle.h>
#include <wpi/struct/Struct.h>

#include "gtest/gtest.h"
#include "akit/LogTable.h"

namespace {

using akit::LogStorage;
using akit::LogTable;
using akit::LogValue;

std::vector<uint8_t> PackRotation(const frc::Rotation2d& rotation) {
    std::vector<uint8_t> bytes(wpi::Struct<frc::Rotation2d>::GetSize());
    wpi::PackStruct(std::span{bytes}, rotation);
    return bytes;
}

std::vector<uint8_t> PackRotations(const std::vector<frc::Rotation2d>& rotations) {
    const size_t elemSize = wpi::Struct<frc::Rotation2d>::GetSize();
    std::vector<uint8_t> bytes(elemSize * rotations.size());
    for (size_t i = 0; i < rotations.size(); ++i) {
        wpi::PackStruct(std::span{bytes}.subspan(i * elemSize, elemSize), rotations[i]);
    }
    return bytes;
}

TEST(LogTableStructTest, StructRoundTripSucceedsWhenTypeMatches) {
    LogStorage storage;
    LogTable table(storage);
    frc::Rotation2d expected{units::radian_t{1.234}};

    table.Put("rotation", expected);

    frc::Rotation2d actual = table.Get("rotation", frc::Rotation2d{});

    EXPECT_DOUBLE_EQ(actual.Radians().value(), expected.Radians().value());
}

TEST(LogTableStructTest, SingleStructReplayReturnsDefaultOnWrongCustomType) {
    LogStorage storage;
    LogTable table(storage);
    frc::Rotation2d expectedDefault{units::radian_t{0.75}};

    table.Put("rotation", LogValue{PackRotation(frc::Rotation2d{units::radian_t{1.234}}), "wrong:type"});

    frc::Rotation2d actual = table.Get("rotation", expectedDefault);

    EXPECT_DOUBLE_EQ(actual.Radians().value(), expectedDefault.Radians().value());
}

TEST(LogTableStructTest, SingleStructReplayReturnsDefaultOnWrongRawSize) {
    LogStorage storage;
    LogTable table(storage);
    frc::Rotation2d expectedDefault{units::radian_t{0.75}};
    std::vector<uint8_t> bytes(wpi::Struct<frc::Rotation2d>::GetSize() + 1, 0);

    table.Put("rotation", LogValue{std::move(bytes), std::string(wpi::GetStructTypeString<frc::Rotation2d>())});

    frc::Rotation2d actual = table.Get("rotation", expectedDefault);

    EXPECT_DOUBLE_EQ(actual.Radians().value(), expectedDefault.Radians().value());
}

TEST(LogTableStructTest, StructArrayReplayReturnsDefaultOnSingleStructTypeString) {
    LogStorage storage;
    LogTable table(storage);
    std::vector<frc::Rotation2d> expectedDefault{
        frc::Rotation2d{units::radian_t{0.25}},
        frc::Rotation2d{units::radian_t{0.5}},
    };
    std::vector<frc::Rotation2d> values{
        frc::Rotation2d{units::radian_t{1.0}},
        frc::Rotation2d{units::radian_t{2.0}},
    };

    table.Put("rotations", LogValue{PackRotations(values), std::string(wpi::GetStructTypeString<frc::Rotation2d>())});

    std::vector<frc::Rotation2d> actual = table.Get("rotations", expectedDefault);

    ASSERT_EQ(actual.size(), expectedDefault.size());
    for (size_t i = 0; i < actual.size(); ++i) {
        EXPECT_DOUBLE_EQ(actual[i].Radians().value(), expectedDefault[i].Radians().value());
    }
}

TEST(LogTableStructTest, StructArrayReplayReturnsDefaultOnNonDivisibleRawSize) {
    LogStorage storage;
    LogTable table(storage);
    std::vector<frc::Rotation2d> expectedDefault{frc::Rotation2d{units::radian_t{0.25}}};
    std::vector<uint8_t> bytes(wpi::Struct<frc::Rotation2d>::GetSize() + 1, 0);

    table.Put("rotations", LogValue{
        std::move(bytes),
        std::string(wpi::GetStructTypeString<frc::Rotation2d>()) + "[]"
    });

    std::vector<frc::Rotation2d> actual = table.Get("rotations", expectedDefault);

    ASSERT_EQ(actual.size(), expectedDefault.size());
    EXPECT_DOUBLE_EQ(actual[0].Radians().value(), expectedDefault[0].Radians().value());
}

TEST(LogTableWriteAllowedTest, TypedRawRejectsUntypedOverwrite) {
    LogStorage storage;
    LogTable table(storage);
    std::vector<uint8_t> original{1, 2, 3};
    std::vector<uint8_t> replacement{4, 5, 6};

    table.Put("raw", LogValue{original, "struct:test"});
    table.Put("raw", std::span<const uint8_t>{replacement});

    const LogValue* actual = table.Get("raw");
    ASSERT_NE(actual, nullptr);
    EXPECT_EQ(actual->customTypeStr, "struct:test");
    EXPECT_EQ(std::get<std::vector<uint8_t>>(actual->value), original);
}

TEST(LogTableWriteAllowedTest, UntypedRawRejectsTypedOverwrite) {
    LogStorage storage;
    LogTable table(storage);
    std::vector<uint8_t> original{1, 2, 3};
    std::vector<uint8_t> replacement{4, 5, 6};

    table.Put("raw", std::span<const uint8_t>{original});
    table.Put("raw", LogValue{replacement, "struct:test"});

    const LogValue* actual = table.Get("raw");
    ASSERT_NE(actual, nullptr);
    EXPECT_TRUE(actual->customTypeStr.empty());
    EXPECT_EQ(std::get<std::vector<uint8_t>>(actual->value), original);
}

TEST(LogTableWriteAllowedTest, SameKeyRewriteWithIdenticalMetadataSucceeds) {
    LogStorage storage;
    LogTable table(storage);
    std::vector<uint8_t> original{1, 2, 3};
    std::vector<uint8_t> replacement{4, 5, 6};

    table.Put("raw", LogValue{original, "struct:test"});
    table.Put("raw", LogValue{replacement, "struct:test"});

    const LogValue* actual = table.Get("raw");
    ASSERT_NE(actual, nullptr);
    EXPECT_EQ(actual->customTypeStr, "struct:test");
    EXPECT_EQ(std::get<std::vector<uint8_t>>(actual->value), replacement);
}

}  // namespace
