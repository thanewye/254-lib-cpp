#include <string>
#include <utility>
#include <vector>

#include <frc2/command/CommandHelper.h>
#include <units/time.h>

#include "gtest/gtest.h"
#include "lib/commands/ChezySequenceCommandGroup.h"
#include "lib/util/ConcurrentTimeInterpolatableBuffer.h"
#include "lib/util/ExponentialMovingAverage.h"

namespace {

class TraceCommand : public frc2::CommandHelper<frc2::Command, TraceCommand> {
public:
    TraceCommand(std::vector<std::string>* trace, std::string label)
        : m_trace(trace), m_label(std::move(label)) {}

    void Initialize() override {
        m_trace->push_back(m_label + ":init");
    }

    void Execute() override {
        m_trace->push_back(m_label + ":execute");
        m_finished = true;
    }

    void End(bool interrupted) override {
        m_trace->push_back(m_label + (interrupted ? ":end:interrupted" : ":end"));
    }

    bool IsFinished() override {
        return m_finished;
    }

private:
    std::vector<std::string>* m_trace;
    std::string m_label;
    bool m_finished = false;
};

TEST(ExponentialMovingAverageTest, UsesFilteredStateAcrossSamples) {
    ExponentialMovingAverage average(0.5);

    EXPECT_DOUBLE_EQ(average.Calculate(0.0), 0.0);
    EXPECT_DOUBLE_EQ(average.Calculate(10.0), 5.0);
    EXPECT_DOUBLE_EQ(average.Calculate(10.0), 7.5);
}

TEST(ConcurrentTimeInterpolatableBufferTest, ReturnsExactAndInterpolatedSamples) {
    auto buffer = ConcurrentTimeInterpolatableBuffer<double>::Create(2_s);
    buffer.AddSample(1.0, 2.0);
    buffer.AddSample(3.0, 6.0);

    auto exact = buffer.GetSample(1.0);
    ASSERT_TRUE(exact.has_value());
    EXPECT_DOUBLE_EQ(*exact, 2.0);

    auto interpolated = buffer.GetSample(2.0);
    ASSERT_TRUE(interpolated.has_value());
    EXPECT_DOUBLE_EQ(*interpolated, 4.0);
}

TEST(ChezySequenceCommandGroupTest, RunsVectorSuppliedCommandsInOrder) {
    std::vector<std::string> trace;
    std::vector<frc2::CommandPtr> commands;
    commands.emplace_back(std::make_unique<TraceCommand>(&trace, "first"));
    commands.emplace_back(std::make_unique<TraceCommand>(&trace, "second"));

    ChezySequenceCommandGroup group(std::move(commands));
    group.Initialize();
    group.Execute();

    EXPECT_TRUE(group.IsFinished());
    EXPECT_EQ(trace, (std::vector<std::string>{
        "first:init",
        "first:execute",
        "first:end",
        "second:init",
        "second:execute",
        "second:end",
    }));
}

}  // namespace
