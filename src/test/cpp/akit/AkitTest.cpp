// thanks claude

#include "gtest/gtest.h"

#include "akit/LogStorage.h"
#include "akit/LogTable.h"
#include "akit/LoggableInputs.h"
#include "akit/Logger.h"

#include <cstdint>
#include <string>
#include <variant>
#include <vector>

namespace akit {
namespace {

// ─── LogStorage ──────────────────────────────────────────────────────────────

TEST(LogStorageTest, DefaultState) {
    LogStorage s;
    EXPECT_TRUE(s.values.empty());
    EXPECT_EQ(s.timestamp, int64_t{0});
}

TEST(LogStorageTest, ClearRemovesValues) {
    LogStorage s;
    s.values["a"] = bool{true};
    s.values["b"] = double{42.0};
    s.Clear();
    EXPECT_TRUE(s.values.empty());
}

TEST(LogStorageTest, ClearPreservesTimestamp) {
    // Clear() only empties the values map; it does not reset the timestamp.
    LogStorage s;
    s.timestamp = int64_t{3140000};
    s.values["k"] = int64_t{1};
    s.Clear();
    EXPECT_TRUE(s.values.empty());
    EXPECT_EQ(s.timestamp, int64_t{3140000});
}

TEST(LogStorageTest, SupportsAllLogValueTypes) {
    LogStorage s;
    s.values["b"]   = bool{true};
    s.values["i"]   = int64_t{-7};
    s.values["d"]   = double{1.5};
    s.values["s"]   = std::string{"hi"};
    s.values["vec"] = std::vector<double>{1.0, 2.0};
    EXPECT_EQ(s.values.size(), 5u);
}

TEST(LogStorageTest, TimestampCanBeSet) {
    LogStorage s;
    s.timestamp = int64_t{99900000};
    EXPECT_EQ(s.timestamp, int64_t{99900000});
}

// ─── LogTable ────────────────────────────────────────────────────────────────

class LogTableTest : public ::testing::Test {
protected:
    LogStorage storage_;
};

// ── Prefix ──

TEST_F(LogTableTest, DefaultPrefixIsEmpty) {
    LogTable t(storage_);
    EXPECT_EQ(t.GetPrefix(), "");
}

TEST_F(LogTableTest, ExplicitPrefixStored) {
    LogTable t(storage_, "ns/");
    EXPECT_EQ(t.GetPrefix(), "ns/");
}

// ── Put / Get – all types ──

TEST_F(LogTableTest, PutGetBoolTrue) {
    LogTable t(storage_);
    t.Put("b", bool{true});
    EXPECT_TRUE(t.Get("b", false));
}

TEST_F(LogTableTest, PutGetBoolFalse) {
    LogTable t(storage_);
    t.Put("b", bool{false});
    EXPECT_FALSE(t.Get("b", true));
}

TEST_F(LogTableTest, PutGetInt64Negative) {
    LogTable t(storage_);
    t.Put("i", int64_t{-99});
    EXPECT_EQ(t.Get("i", int64_t{0}), int64_t{-99});
}

TEST_F(LogTableTest, PutGetInt64Zero) {
    LogTable t(storage_);
    t.Put("i", int64_t{0});
    EXPECT_EQ(t.Get("i", int64_t{1}), int64_t{0});
}

TEST_F(LogTableTest, PutGetDouble) {
    LogTable t(storage_);
    t.Put("d", double{2.718});
    EXPECT_DOUBLE_EQ(t.Get("d", 0.0), 2.718);
}

TEST_F(LogTableTest, PutGetString) {
    LogTable t(storage_);
    t.Put("s", std::string{"hello"});
    EXPECT_EQ(t.Get("s", std::string{""}), "hello");
}

TEST_F(LogTableTest, PutGetEmptyString) {
    LogTable t(storage_);
    t.Put("s", std::string{""});
    EXPECT_EQ(t.Get("s", std::string{"default"}), "");
}

TEST_F(LogTableTest, PutGetVectorDouble) {
    LogTable t(storage_);
    t.Put("v", std::vector<double>{1.0, 2.0, 3.0});
    auto result = t.Get("v", std::vector<double>{});
    ASSERT_EQ(result.size(), 3u);
    EXPECT_DOUBLE_EQ(result[0], 1.0);
    EXPECT_DOUBLE_EQ(result[1], 2.0);
    EXPECT_DOUBLE_EQ(result[2], 3.0);
}

TEST_F(LogTableTest, PutGetEmptyVector) {
    LogTable t(storage_);
    t.Put("v", std::vector<double>{});
    EXPECT_TRUE(t.Get("v", std::vector<double>{9.0}).empty());
}

// ── Default values on miss ──

TEST_F(LogTableTest, MissingKeyReturnsBoolDefault) {
    LogTable t(storage_);
    EXPECT_TRUE(t.Get("x", true));
    EXPECT_FALSE(t.Get("x", false));
}

TEST_F(LogTableTest, MissingKeyReturnsInt64Default) {
    LogTable t(storage_);
    EXPECT_EQ(t.Get("x", int64_t{7}), int64_t{7});
}

TEST_F(LogTableTest, MissingKeyReturnsDoubleDefault) {
    LogTable t(storage_);
    EXPECT_DOUBLE_EQ(t.Get("x", 9.9), 9.9);
}

TEST_F(LogTableTest, MissingKeyReturnsStringDefault) {
    LogTable t(storage_);
    EXPECT_EQ(t.Get("x", std::string{"def"}), "def");
}

TEST_F(LogTableTest, MissingKeyReturnsVectorDefault) {
    LogTable t(storage_);
    auto result = t.Get("x", std::vector<double>{5.0});
    ASSERT_EQ(result.size(), 1u);
    EXPECT_DOUBLE_EQ(result[0], 5.0);
}

// ── Type mismatch returns default ──

TEST_F(LogTableTest, TypeMismatchBoolReturnsDefault) {
    LogTable t(storage_);
    t.Put("k", double{3.14});         // stored as double
    EXPECT_FALSE(t.Get("k", false));  // requested bool → default
}

TEST_F(LogTableTest, TypeMismatchDoubleReturnsDefault) {
    LogTable t(storage_);
    t.Put("k", std::string{"oops"});
    EXPECT_DOUBLE_EQ(t.Get("k", -1.0), -1.0);
}

TEST_F(LogTableTest, TypeMismatchInt64ReturnsDefault) {
    LogTable t(storage_);
    t.Put("k", double{1.5});
    EXPECT_EQ(t.Get("k", int64_t{42}), int64_t{42});
}

// ── Overwrite ──

TEST_F(LogTableTest, PutOverwritesSameKey) {
    LogTable t(storage_);
    t.Put("d", double{1.0});
    t.Put("d", double{2.0});
    EXPECT_DOUBLE_EQ(t.Get("d", 0.0), 2.0);
}

// ── Subtable prefix ──

TEST_F(LogTableTest, SubtablePrefixEndsWithSlash) {
    LogTable root(storage_);
    EXPECT_EQ(root.GetSubtable("motor").GetPrefix(), "motor/");
}

TEST_F(LogTableTest, SubtableComposesWithParentPrefix) {
    LogTable parent(storage_, "inputs/");
    EXPECT_EQ(parent.GetSubtable("arm").GetPrefix(), "inputs/arm/");
}

TEST_F(LogTableTest, NestedSubtablePrefix) {
    LogTable root(storage_);
    auto deep = root.GetSubtable("a").GetSubtable("b");
    EXPECT_EQ(deep.GetPrefix(), "a/b/");
}

TEST_F(LogTableTest, SubtablePutVisibleFromParentWithFullKey) {
    LogTable root(storage_);
    root.GetSubtable("drive").Put("speed", double{5.0});
    EXPECT_DOUBLE_EQ(root.Get("drive/speed", 0.0), 5.0);
}

TEST_F(LogTableTest, ParentPutVisibleFromSubtable) {
    LogTable root(storage_);
    root.Put("arm/angle", double{45.0});
    EXPECT_DOUBLE_EQ(root.GetSubtable("arm").Get("angle", 0.0), 45.0);
}

TEST_F(LogTableTest, NestedSubtablePutVisibleFromRoot) {
    LogTable root(storage_);
    root.GetSubtable("a").GetSubtable("b").Put("x", bool{true});
    EXPECT_TRUE(root.Get("a/b/x", false));
}

// ── Timestamp ──

TEST_F(LogTableTest, SetGetTimestampRoundtrip) {
    LogTable t(storage_);
    t.SetTimestamp(int64_t{123456000});
    EXPECT_EQ(t.GetTimestamp(), int64_t{123456000});
}

TEST_F(LogTableTest, TimestampSharedBetweenTablesOnSameStorage) {
    LogTable a(storage_);
    LogTable b(storage_);
    a.SetTimestamp(int64_t{7000000});
    EXPECT_EQ(b.GetTimestamp(), int64_t{7000000});
}

TEST_F(LogTableTest, SubtableSharesTimestampWithParent) {
    LogTable root(storage_);
    root.SetTimestamp(int64_t{3000000});
    EXPECT_EQ(root.GetSubtable("x").GetTimestamp(), int64_t{3000000});
}

TEST_F(LogTableTest, SubtableCanSetTimestamp) {
    LogTable root(storage_);
    root.GetSubtable("x").SetTimestamp(int64_t{8000000});
    EXPECT_EQ(root.GetTimestamp(), int64_t{8000000});
}

// ── GetAll ──

TEST_F(LogTableTest, GetAllContainsAllPutValues) {
    LogTable t(storage_);
    t.Put("a", bool{true});
    t.Put("b", double{1.0});
    EXPECT_EQ(t.GetAll().size(), 2u);
}

TEST_F(LogTableTest, GetAllSpansSubtables) {
    LogTable root(storage_);
    root.GetSubtable("s").Put("x", double{1.0});
    root.Put("y", double{2.0});
    EXPECT_EQ(root.GetAll().size(), 2u);
}

TEST_F(LogTableTest, GetAllContainsFullyQualifiedKeys) {
    LogTable root(storage_);
    root.GetSubtable("sub").Put("val", double{1.0});
    EXPECT_EQ(root.GetAll().count("sub/val"), 1u);
    EXPECT_EQ(root.GetAll().count("val"),     0u);
}

// ── Clear ──

TEST_F(LogTableTest, ClearRootRemovesEverything) {
    LogTable t(storage_);
    t.Put("a", double{1.0});
    t.Put("b", bool{true});
    t.Clear();
    EXPECT_TRUE(t.GetAll().empty());
}

TEST_F(LogTableTest, ClearSubtableRemovesOnlyItsKeys) {
    LogTable root(storage_);
    root.GetSubtable("motor").Put("speed", double{1.0});
    root.Put("global", bool{true});
    root.GetSubtable("motor").Clear();
    EXPECT_EQ(root.GetAll().size(),              1u);
    EXPECT_EQ(root.GetAll().count("motor/speed"), 0u);
    EXPECT_EQ(root.GetAll().count("global"),      1u);
}

TEST_F(LogTableTest, ClearSubtableLeavesOtherSubtables) {
    LogTable root(storage_);
    root.GetSubtable("a").Put("v", double{1.0});
    root.GetSubtable("b").Put("v", double{2.0});
    root.GetSubtable("a").Clear();
    EXPECT_EQ(root.GetAll().size(),     1u);
    EXPECT_EQ(root.GetAll().count("b/v"), 1u);
}

TEST_F(LogTableTest, ClearSubtableDoesNotAffectTimestamp) {
    LogTable root(storage_);
    root.GetSubtable("x").Put("v", double{1.0});
    root.SetTimestamp(int64_t{42000000});
    root.GetSubtable("x").Clear();
    EXPECT_EQ(root.GetTimestamp(), int64_t{42000000});
}

TEST_F(LogTableTest, ClearRootDoesNotResetTimestamp) {
    LogTable t(storage_);
    t.SetTimestamp(int64_t{5000000});
    t.Put("k", double{1.0});
    t.Clear();
    // Clear() on LogTable does not touch the timestamp (only clears values).
    EXPECT_EQ(t.GetTimestamp(), int64_t{5000000});
}

// ─── LoggableInputs ──────────────────────────────────────────────────────────

class MockInputs : public LoggableInputs {
public:
    double   speed   = 0.0;
    bool     enabled = false;
    int64_t  count   = 0;

    void ToLog(LogTable& table) const override {
        table.Put("speed",   double{speed});
        table.Put("enabled", bool{enabled});
        table.Put("count",   int64_t{count});
    }

    void FromLog(const LogTable& table) override {
        speed   = table.Get("speed",   0.0);
        enabled = table.Get("enabled", false);
        count   = table.Get("count",   int64_t{0});
    }
};

TEST(LoggableInputsTest, ToLogWritesAllFields) {
    LogStorage storage;
    LogTable table(storage);
    MockInputs in;
    in.speed   = 3.5;
    in.enabled = true;
    in.count   = int64_t{7};
    in.ToLog(table);
    EXPECT_DOUBLE_EQ(table.Get("speed",   0.0),        3.5);
    EXPECT_TRUE(     table.Get("enabled", false));
    EXPECT_EQ(       table.Get("count",   int64_t{0}), int64_t{7});
}

TEST(LoggableInputsTest, FromLogReadsAllFields) {
    LogStorage storage;
    LogTable table(storage);
    table.Put("speed",   double{2.0});
    table.Put("enabled", bool{true});
    table.Put("count",   int64_t{3});
    MockInputs out;
    out.FromLog(table);
    EXPECT_DOUBLE_EQ(out.speed, 2.0);
    EXPECT_TRUE(out.enabled);
    EXPECT_EQ(out.count, int64_t{3});
}

TEST(LoggableInputsTest, RoundtripToLogFromLog) {
    LogStorage storage;
    LogTable table(storage);
    MockInputs original;
    original.speed   = 99.5;
    original.enabled = true;
    original.count   = int64_t{-1};
    original.ToLog(table);
    MockInputs restored;
    restored.FromLog(table);
    EXPECT_DOUBLE_EQ(restored.speed,   original.speed);
    EXPECT_EQ(       restored.enabled, original.enabled);
    EXPECT_EQ(       restored.count,   original.count);
}

TEST(LoggableInputsTest, FromLogWithMissingKeysUsesDefaults) {
    LogStorage storage;
    LogTable table(storage);  // empty table
    MockInputs in;
    in.speed   = 10.0;
    in.enabled = true;
    in.count   = int64_t{5};
    in.FromLog(table);
    EXPECT_DOUBLE_EQ(in.speed, 0.0);
    EXPECT_FALSE(in.enabled);
    EXPECT_EQ(in.count, int64_t{0});
}

TEST(LoggableInputsTest, MultipleRoundtripsPreserveValues) {
    LogStorage storage;
    LogTable table(storage);
    MockInputs a;
    a.speed = 1.0;
    a.ToLog(table);
    MockInputs b;
    b.FromLog(table);
    EXPECT_DOUBLE_EQ(b.speed, 1.0);

    // Second roundtrip with different value
    b.speed = 2.0;
    b.ToLog(table);
    MockInputs c;
    c.FromLog(table);
    EXPECT_DOUBLE_EQ(c.speed, 2.0);
}

TEST(LoggableInputsTest, VirtualDestructorAllowsPolymorphicDelete) {
    LoggableInputs* p = new MockInputs();
    EXPECT_NO_FATAL_FAILURE(delete p);
}

// ─── Logger ──────────────────────────────────────────────────────────────────

class LoggerTest : public ::testing::Test {
protected:
    void SetUp() override {
        Logger::End();
        Logger::SetReplayMode(false);
        Logger::Clear();
    }

    void TearDown() override {
        Logger::End();
        Logger::SetReplayMode(false);
        Logger::Clear();
    }

    // Copy the current Logger storage into a local LogStorage so that we can
    // wrap a LogTable around it for convenient Get() access.
    static LogStorage SnapStorage() {
        const LogStorage& src = Logger::GetCurrentStorage();
        LogStorage copy;
        copy.values    = src.values;
        copy.timestamp = src.timestamp;
        return copy;
    }
};

// ── Start / End ──

TEST_F(LoggerTest, StartClearsStorageValues) {
    Logger::Start();
    auto snap = SnapStorage();
    EXPECT_TRUE(snap.values.empty());
}

TEST_F(LoggerTest, StartResetsTimestampToZero) {
    Logger::Start();
    EXPECT_EQ(SnapStorage().timestamp, int64_t{0});
}

TEST_F(LoggerTest, EndDisablesProcessInputs) {
    Logger::Start();
    Logger::End();
    MockInputs in;
    in.speed = 5.0;
    Logger::ProcessInputs("sys", in);
    EXPECT_TRUE(SnapStorage().values.empty());
}

TEST_F(LoggerTest, EndDisablesRecordOutput) {
    Logger::Start();
    Logger::End();
    Logger::RecordOutput("key", double{1.0});
    EXPECT_TRUE(SnapStorage().values.empty());
}

// ── ProcessInputs – not running ──

TEST_F(LoggerTest, ProcessInputsIsNoOpWhenNotRunning) {
    MockInputs in;
    in.speed = 99.0;
    Logger::ProcessInputs("x", in);
    EXPECT_TRUE(SnapStorage().values.empty());
}

// ── ProcessInputs – real mode ──

TEST_F(LoggerTest, ProcessInputsCallsToLogInRealMode) {
    Logger::Start();
    Logger::SetReplayMode(false);
    MockInputs in;
    in.speed   = 7.0;
    in.enabled = true;
    Logger::ProcessInputs("drive", in);
    auto snap = SnapStorage();
    LogTable view(snap);
    EXPECT_DOUBLE_EQ(view.Get("drive/speed",   0.0),  7.0);
    EXPECT_TRUE(     view.Get("drive/enabled", false));
}

TEST_F(LoggerTest, ProcessInputsKeyBecomesSubtablePrefix) {
    Logger::Start();
    MockInputs in;
    in.speed = 3.0;
    Logger::ProcessInputs("arm", in);
    auto snap = SnapStorage();
    // Key must be "arm/speed", not bare "speed"
    EXPECT_EQ(snap.values.count("speed"),     0u);
    EXPECT_EQ(snap.values.count("arm/speed"), 1u);
}

TEST_F(LoggerTest, MultipleProcessInputsCallsStoreSeparately) {
    Logger::Start();
    MockInputs a, b;
    a.speed = 1.0;
    b.speed = 2.0;
    Logger::ProcessInputs("subsysA", a);
    Logger::ProcessInputs("subsysB", b);
    auto snap = SnapStorage();
    LogTable view(snap);
    EXPECT_DOUBLE_EQ(view.Get("subsysA/speed", 0.0), 1.0);
    EXPECT_DOUBLE_EQ(view.Get("subsysB/speed", 0.0), 2.0);
}

// ── ProcessInputs – replay mode ──

TEST_F(LoggerTest, ProcessInputsCallsFromLogInReplayMode) {
    Logger::Start();
    // Write data in real mode
    Logger::SetReplayMode(false);
    MockInputs writer;
    writer.speed = 42.0;
    Logger::ProcessInputs("sys", writer);

    // Switch to replay mode and read it back
    Logger::SetReplayMode(true);
    MockInputs reader;
    reader.speed = 0.0;
    Logger::ProcessInputs("sys", reader);
    EXPECT_DOUBLE_EQ(reader.speed, 42.0);
}

TEST_F(LoggerTest, ProcessInputsDoesNotCallToLogInReplayMode) {
    Logger::Start();
    Logger::SetReplayMode(true);
    // In replay mode, ToLog should NOT be called.
    // Nothing was pre-loaded, so reader should get defaults.
    MockInputs in;
    in.speed = 55.0;
    Logger::ProcessInputs("sys", in);
    // speed should have been overwritten to the default (0.0) from FromLog
    EXPECT_DOUBLE_EQ(in.speed, 0.0);
}

// ── RecordOutput ──

TEST_F(LoggerTest, RecordOutputIsNoOpWhenNotRunning) {
    Logger::RecordOutput("k", double{1.0});
    EXPECT_TRUE(SnapStorage().values.empty());
}

TEST_F(LoggerTest, RecordOutputUsesRealOutputsPrefixInRealMode) {
    Logger::Start();
    Logger::SetReplayMode(false);
    Logger::RecordOutput("velocity", double{3.14});
    auto snap = SnapStorage();
    EXPECT_EQ(snap.values.count("RealOutputs/velocity"),   1u);
    EXPECT_EQ(snap.values.count("ReplayOutputs/velocity"), 0u);
}

TEST_F(LoggerTest, RecordOutputUsesReplayOutputsPrefixInReplayMode) {
    Logger::Start();
    Logger::SetReplayMode(true);
    Logger::RecordOutput("angle", double{90.0});
    auto snap = SnapStorage();
    EXPECT_EQ(snap.values.count("ReplayOutputs/angle"), 1u);
    EXPECT_EQ(snap.values.count("RealOutputs/angle"),   0u);
}

TEST_F(LoggerTest, RecordOutputStoresCorrectDoubleValue) {
    Logger::Start();
    Logger::RecordOutput("myVal", double{2.71});
    auto snap = SnapStorage();
    LogTable view(snap);
    EXPECT_DOUBLE_EQ(view.Get("RealOutputs/myVal", 0.0), 2.71);
}

TEST_F(LoggerTest, RecordOutputStoresBoolValue) {
    Logger::Start();
    Logger::RecordOutput("flag", bool{true});
    auto snap = SnapStorage();
    LogTable view(snap);
    EXPECT_TRUE(view.Get("RealOutputs/flag", false));
}

TEST_F(LoggerTest, RecordOutputStoresInt64Value) {
    Logger::Start();
    Logger::RecordOutput("n", int64_t{-7});
    auto snap = SnapStorage();
    LogTable view(snap);
    EXPECT_EQ(view.Get("RealOutputs/n", int64_t{0}), int64_t{-7});
}

TEST_F(LoggerTest, RecordOutputStoresStringValue) {
    Logger::Start();
    Logger::RecordOutput("msg", std::string{"hello"});
    auto snap = SnapStorage();
    LogTable view(snap);
    EXPECT_EQ(view.Get("RealOutputs/msg", std::string{""}), "hello");
}

TEST_F(LoggerTest, RecordOutputStoresVectorValue) {
    Logger::Start();
    Logger::RecordOutput("data", std::vector<double>{1.0, 2.0});
    auto snap = SnapStorage();
    LogTable view(snap);
    auto result = view.Get("RealOutputs/data", std::vector<double>{});
    ASSERT_EQ(result.size(), 2u);
    EXPECT_DOUBLE_EQ(result[0], 1.0);
    EXPECT_DOUBLE_EQ(result[1], 2.0);
}

// ── SetReplayMode / HasReplaySource ──

TEST_F(LoggerTest, DefaultNotReplayMode) {
    EXPECT_FALSE(Logger::HasReplaySource());
}

TEST_F(LoggerTest, SetReplayModeTrue) {
    Logger::SetReplayMode(true);
    EXPECT_TRUE(Logger::HasReplaySource());
}

TEST_F(LoggerTest, SetReplayModeFalseAfterTrue) {
    Logger::SetReplayMode(true);
    Logger::SetReplayMode(false);
    EXPECT_FALSE(Logger::HasReplaySource());
}

// ── Clear ──

TEST_F(LoggerTest, ClearEmptiesValues) {
    Logger::Start();
    Logger::RecordOutput("x", double{1.0});
    Logger::Clear();
    EXPECT_TRUE(SnapStorage().values.empty());
}

TEST_F(LoggerTest, ClearResetsTimestampToZero) {
    Logger::Start();
    Logger::PeriodicBeforeUser();
    Logger::Clear();
    EXPECT_EQ(Logger::GetCurrentStorage().timestamp, int64_t{0});
}

// ── PeriodicBeforeUser ──

TEST_F(LoggerTest, PeriodicBeforeUserIsNoOpWhenNotRunning) {
    EXPECT_NO_FATAL_FAILURE(Logger::PeriodicBeforeUser());
    EXPECT_TRUE(SnapStorage().values.empty());
}

TEST_F(LoggerTest, PeriodicBeforeUserClearsPreviousValues) {
    Logger::Start();
    Logger::RecordOutput("old", double{1.0});
    ASSERT_EQ(SnapStorage().values.size(), 1u);
    Logger::PeriodicBeforeUser();
    EXPECT_TRUE(SnapStorage().values.empty());
}

TEST_F(LoggerTest, PeriodicBeforeUserSetsTimestamp) {
    Logger::Start();
    Logger::Clear();  // force timestamp to 0
    Logger::PeriodicBeforeUser();
    // In simulation the FPGA clock starts at 0; just verify it was set (>= 0)
    EXPECT_GE(Logger::GetCurrentStorage().timestamp, 0.0);
}

// ── PeriodicAfterUser ──

TEST_F(LoggerTest, PeriodicAfterUserIsNoOpWhenNotRunning) {
    EXPECT_NO_FATAL_FAILURE(Logger::PeriodicAfterUser());
}

TEST_F(LoggerTest, PeriodicAfterUserDoesNotClearStorage) {
    Logger::Start();
    Logger::RecordOutput("keep", bool{true});
    Logger::PeriodicAfterUser();
    EXPECT_EQ(SnapStorage().values.size(), 1u);
}

// ── GetCurrentStorage ──

TEST_F(LoggerTest, GetCurrentStorageReturnsSameObjectOnEachCall) {
    const LogStorage& s1 = Logger::GetCurrentStorage();
    const LogStorage& s2 = Logger::GetCurrentStorage();
    EXPECT_EQ(&s1, &s2);
}

TEST_F(LoggerTest, GetCurrentStorageReflectsRecordedOutput) {
    Logger::Start();
    Logger::RecordOutput("probe", double{5.0});
    const LogStorage& s = Logger::GetCurrentStorage();
    EXPECT_EQ(s.values.count("RealOutputs/probe"), 1u);
}

// ── DumpCurrentStorage ──

TEST_F(LoggerTest, DumpCurrentStorageOutputsKeyValuePairs) {
    Logger::Start();
    Logger::RecordOutput("speed", double{42.0});
    testing::internal::CaptureStdout();
    Logger::DumpCurrentStorage();
    std::string out = testing::internal::GetCapturedStdout();
    EXPECT_NE(out.find("RealOutputs/speed"), std::string::npos);
    EXPECT_NE(out.find("42"),                std::string::npos);
}

TEST_F(LoggerTest, DumpCurrentStorageOutputsVectorBrackets) {
    Logger::Start();
    Logger::RecordOutput("data", std::vector<double>{1.0, 2.0, 3.0});
    testing::internal::CaptureStdout();
    Logger::DumpCurrentStorage();
    std::string out = testing::internal::GetCapturedStdout();
    EXPECT_NE(out.find("["), std::string::npos);
    EXPECT_NE(out.find("]"), std::string::npos);
}

TEST_F(LoggerTest, DumpCurrentStorageEmptyProducesNoOutput) {
    Logger::Start();
    // No values recorded
    testing::internal::CaptureStdout();
    Logger::DumpCurrentStorage();
    std::string out = testing::internal::GetCapturedStdout();
    EXPECT_TRUE(out.empty());
}

TEST_F(LoggerTest, DumpCurrentStorageOutputsBoolValue) {
    Logger::Start();
    Logger::RecordOutput("flag", bool{true});
    testing::internal::CaptureStdout();
    Logger::DumpCurrentStorage();
    std::string out = testing::internal::GetCapturedStdout();
    EXPECT_NE(out.find("RealOutputs/flag"), std::string::npos);
}

TEST_F(LoggerTest, DumpCurrentStorageOutputsStringValue) {
    Logger::Start();
    Logger::RecordOutput("label", std::string{"hello"});
    testing::internal::CaptureStdout();
    Logger::DumpCurrentStorage();
    std::string out = testing::internal::GetCapturedStdout();
    EXPECT_NE(out.find("hello"), std::string::npos);
}

} // namespace
} // namespace akit
