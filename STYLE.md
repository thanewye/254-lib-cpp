# C++ Style Guide

## Indentation

4 spaces per level. No tabs.

```cpp
namespace akit {
    class Logger {
    public:
        static void Start();
    };
}
```

## Braces

K&R style throughout — opening brace on the same line as the statement. This applies to classes, functions, control flow, lambdas, and namespaces.

```cpp
void Logger::Start() {
    if (!running_) {
        running_ = true;
    }
}
```

Single-statement bodies may omit braces only when the entire `if` fits on one line:

```cpp
if (!running_) return;
```

## Namespaces

Nested namespaces use the `::` shorthand syntax. Closing braces are always commented.

```cpp
namespace akit::wpilog {
    // ...
} // namespace akit::wpilog
```

`using namespace` is never allowed in headers. In `.cpp` files and unnamed test namespaces, `using` declarations for specific names are fine.

```cpp
namespace {
    using akit::Logger;
    using akit::LogStorage;
}
```

## Naming

| Category | Convention | Examples |
|---|---|---|
| Classes / structs | `PascalCase` | `Logger`, `StateMachine`, `CANStatusLogger` |
| Functions / methods | `PascalCase` | `Start()`, `RecordOutput()`, `GetInstance()` |
| Parameters | `camelCase` | `userCodeUs`, `extraConsoleData` |
| Member variables (library) | `trailingUnderscore_` | `running_`, `cycles_`, `mutex_` |
| Member variables (robot) | `m_prefix` | `m_autonomousCommand`, `m_container` |
| Constants / constexpr | `kPrefixedCamelCase` | `kPose2dZero`, `kSimControllerType` |
| Enum names | `PascalCase` | `TestStructuredMode`, `AdvantageScopeOpenBehavior` |
| Enum values | `kCamelCase` | `kIdle`, `kMoving`, `kNever` |

> **Note:** Core library code (`akit/`, `lib/`) uses trailing-underscore members. Top-level robot code (`frc2026/`) uses `m_` prefix. New code should match whichever convention is already established in that directory.

Older enums use `UPPER_CASE` enumerators — prefer `kCamelCase` in new code.

## Class Layout

Public before private. Constructors at the top of the public section. Static methods before instance methods.

```cpp
class Foo {
public:
    static Foo& GetInstance();
    explicit Foo(std::string name);

    void DoThing();
    [[nodiscard]] int GetValue() const;

private:
    std::string name_;
    int value_ = 0;
};
```

## Includes

`.cpp` files:

1. Corresponding header (no blank line before it)
2. Blank line
3. STL headers (`<angle_brackets>`, alphabetical)
4. Blank line
5. Vendor headers (`<angle_brackets>`, alphabetical)
6. Blank line
7. Project headers (`"quotes"`, alphabetical)

`.h` files:

1. `#pragma once`
2. Blank line
3. STL headers
4. Blank line
5. Vendor headers
6. Blank line
7. Project headers

Sorting within each group uses case-sensitive ASCII order — uppercase letters (A–Z) sort before lowercase (a–z).

Known vendor prefixes: `ctre`, `fmt`, `frc`, `frc2`, `gtest`, `hal`, `magic_enum`, `networktables`, `units`, `wpi`, `wpinet`.

Files with two or fewer total includes are exempt from the blank-line grouping requirement.

## Line Length

Max 160 characters. Wrap only when a line genuinely can't fit.

## `const`

West const (type before `const`). Applies to member functions, parameters, and locals.

```cpp
[[nodiscard]] std::optional<std::string> GetCurrentState() const;
const std::string& key
```

## `auto`

Used sparingly. Prefer explicit types; use `auto` for structured bindings and when the iterator type would be verbose.

```cpp
for (const auto& [key, value] : map_) { ... }
auto it = states_.find(name);
```

## Pointers and References

`T*` and `T&` — star and ampersand attach to the type, not the name.

```cpp
static void SetReplaySource(LogReplaySource* source);
void ProcessInputs(std::string_view key, T& inputs);
```

## `override` and `final`

Always present on overriding virtual functions. Never omitted.

```cpp
void RobotPeriodic() override;
[[nodiscard]] std::string GetNewData() override;
```

## `[[nodiscard]]`

Applied to functions whose return value carries important information — getters, computed results, factory functions, and functions returning `std::optional`.

```cpp
[[nodiscard]] std::optional<std::string> GetCurrentState() const;
[[nodiscard]] int GetButton(const std::string& name) const;
```

Do not apply to mutating methods (e.g. `AddState`, `TryUntilOk`) where ignoring the return value is intentional.

## `explicit`

Required on all single-parameter constructors. No exceptions.

```cpp
explicit StateMachine(std::string name);
explicit ExponentialMovingAverage(double alpha);
```

## Templates

Parameter names: `T` or descriptive (`Distance`, `Supplier`). C++20 `requires` clauses used for constraints.

```cpp
template<typename Supplier> requires std::invocable<Supplier>
static void RecordOutput(std::string_view key, Supplier&& value);
```

## Header Guards

`#pragma once` only. No `#ifndef` guards.

## Initialization

Member initializer lists in constructors. Uniform init `{}` for member defaults. Assignment `=` acceptable for simple scalar defaults.

```cpp
LoggedRobot::LoggedRobot(double period)
    : IterativeRobotBase(units::second_t{period})
    , periodUs_(static_cast<uint64_t>(period * 1'000'000)) {}

inline static LogStorage currentStorage_{};
bool m_last = false;
```

## Null Pointers

`nullptr` exclusively. Never `NULL` or `0`.

## Casts

`static_cast<T>()` for value conversions. Never C-style casts.

```cpp
static_cast<uint64_t>(period * 1'000'000)
static_cast<double>(frc::RobotController::GetFPGATime())
```

## `static`

Meyer's Singleton pattern for singletons:

```cpp
static CANStatusLogger& GetInstance() {
    static CANStatusLogger instance;
    return instance;
}
```

Static members are `inline static` when initialized in the header.

## Error Handling

- FRC errors: `FRC_ReportError(frc::err::Error, "...")`
- "Maybe" values: `std::optional<T>` instead of nullable pointers
- Exceptions: caught at boundaries, not used for control flow

## Comments

Single-line `//` comments. Write comments only when the *why* is non-obvious — a hidden constraint, a workaround, or behavior that would surprise a reader. Don't describe what the code does.

Namespace closing braces are always commented: `} // namespace foo`.

No multi-line block comment documentation. One-line `/** */` doc comments are acceptable but not required.

## Blank Lines

- Between methods: none
- Between `public`/`private` sections: one blank line
- Within a function body: one blank line between logically distinct steps

## Tests

Google Test. `TEST()` for standalone tests, `TEST_F()` for fixture-based tests. Fixtures use `SetUp()` / `TearDown()`.

`ASSERT_*` for fatal checks (abort the test on failure). `EXPECT_*` for non-fatal checks (continue after failure).

Test names use the format `DescriptionOfWhatIsBeingTested_ExpectedOutcome`.

```cpp
class LoggerReplayParityTest : public ::testing::Test {
protected:
    void SetUp() override { ... }
    void TearDown() override { ... }
};

TEST_F(LoggerReplayParityTest, ReplayStartupLoadsInitialFrameAndWritesReplayOutputs) {
    ASSERT_TRUE(Logger::IsRunning());
    EXPECT_DOUBLE_EQ(std::get<double>(values.at("/Applied").value), 12.5);
}
```
