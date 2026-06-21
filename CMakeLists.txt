cmake_minimum_required(VERSION 3.20)
project(frc_2026_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

file(GLOB_RECURSE ROBOT_SOURCES
        src/main/cpp/*.cpp
        src/main/cpp/*.h
        src/test/cpp/*.cpp
        src/test/cpp/*.h
)

add_executable(frc_2026_cpp ${ROBOT_SOURCES}
        src/main/cpp/lib/subsystems/ServoMotorSubsystem.ipp
        src/main/cpp/lib/subsystems/ServoMotorSubsystemWithFollowers.ipp
        src/main/cpp/lib/subsystems/ServoMotorSubsystemWithCanCoder.ipp
        src/main/cpp/lib/subsystems/TalonFXIO.ipp
        src/main/cpp/lib/subsystems/SimTalonFXIO.ipp
        src/main/cpp/lib/subsystems/SimElevator.ipp)

if(WIN32)
    set(USER_HOME_DIR "$ENV{USERPROFILE}")
else()
    set(USER_HOME_DIR "$ENV{HOME}")
endif()
file(GLOB GRADLE_HEADER_DIRS "${USER_HOME_DIR}/.gradle/caches/*/transforms/*/transformed/*-headers")

target_include_directories(frc_2026_cpp PRIVATE
        ${CMAKE_SOURCE_DIR}/src/main/cpp
        ${CMAKE_SOURCE_DIR}/third_party/pfr/include
        ${CMAKE_SOURCE_DIR}/third_party/magic_enum/include
        ${GRADLE_HEADER_DIRS}
)
