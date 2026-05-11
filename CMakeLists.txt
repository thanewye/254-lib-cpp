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
        src/main/cpp/lib/subsystems/ServoMotorSubsystemWithCanCoder.ipp)

target_include_directories(frc_2026_cpp PRIVATE
        ${CMAKE_SOURCE_DIR}/src/main/cpp

        # WPILib
        /Users/ethanye/.gradle/caches/8.11/transforms/58762833e41f59b7a1a94d3032b654e9/transformed/wpilibNewCommands-cpp-2026.2.1-headers
        /Users/ethanye/.gradle/caches/8.11/transforms/d8a5045724d8f0b0a1c4d392b9f66d90/transformed/wpilibc-cpp-2026.2.1-headers
        /Users/ethanye/.gradle/caches/8.11/transforms/0893c75d135667fc6d1e5c790e448544/transformed/ntcore-cpp-2026.2.1-headers
        /Users/ethanye/.gradle/caches/8.11/transforms/be8569115ef97f2adce25e9c3bb6d904/transformed/hal-cpp-2026.2.1-headers
        /Users/ethanye/.gradle/caches/8.11/transforms/9df5023e435d18241ace65ec192d1adb/transformed/wpimath-cpp-2026.2.1-headers
        /Users/ethanye/.gradle/caches/8.11/transforms/69e7eb3cc206812c85d11928f4b6f88a/transformed/wpinet-cpp-2026.2.1-headers
        /Users/ethanye/.gradle/caches/8.11/transforms/b0060593ad44905028bc65e0686e0d6b/transformed/wpiutil-cpp-2026.2.1-headers
        /Users/ethanye/.gradle/caches/8.11/transforms/e9c4b6b96098d3331195292f912488be/transformed/cameraserver-cpp-2026.2.1-headers
        /Users/ethanye/.gradle/caches/8.11/transforms/2933d1b34da285304e5b651839dfc26b/transformed/cscore-cpp-2026.2.1-headers
        /Users/ethanye/.gradle/caches/8.11/transforms/ac910a4c36943267fb1973d99100a7f9/transformed/apriltag-cpp-2026.2.1-headers
        /Users/ethanye/.gradle/caches/8.11/transforms/f3b111a8c47e2fa61e9595280367bd51/transformed/opencv-cpp-4.10.0-3-headers

        # Photon / Choreo
        /Users/ethanye/.gradle/caches/8.11/transforms/8be4572c491f87d8c5ef6ae431b88db4/transformed/photonlib-cpp-v2026.3.4-headers
        /Users/ethanye/.gradle/caches/8.11/transforms/bb9d1d70677617a62492c17255cfe3b7/transformed/photontargeting-cpp-v2026.3.4-headers
        /Users/ethanye/.gradle/caches/8.11/transforms/f7f14f84e54834fd1732829344b839b9/transformed/ChoreoLib-cpp-2026.0.3-headers

        # CTRE / sim
        /Users/ethanye/.gradle/caches/8.11/transforms/cd5b42add7f5d770d30617fb0d611b88/transformed/wpiapi-cpp-sim-26.1.3-headers
        /Users/ethanye/.gradle/caches/8.11/transforms/4f96d4d679770bb63049ba1cb6953f8e/transformed/tools-sim-26.1.3-headers
        /Users/ethanye/.gradle/caches/8.11/transforms/171e9471e5b36b51f52daa02a8485eb5/transformed/simTalonSRX-26.1.3-headers
        /Users/ethanye/.gradle/caches/8.11/transforms/41c9f7324d9e9f484576ffc3d82726d7/transformed/simVictorSPX-26.1.3-headers
        /Users/ethanye/.gradle/caches/8.11/transforms/ee7102a8c30d9f1b66c4560bdc4f70b8/transformed/simPigeonIMU-26.1.3-headers
        /Users/ethanye/.gradle/caches/8.11/transforms/442408def2a3db09846b00ea74bdad35/transformed/simProTalonFX-26.1.3-headers
        /Users/ethanye/.gradle/caches/8.11/transforms/ce37c91aa8460bd22bff82440eac91e6/transformed/simProTalonFXS-26.1.3-headers
        /Users/ethanye/.gradle/caches/8.11/transforms/2f9254290a0c1625c8c6767e79a53a21/transformed/simProCANcoder-26.1.3-headers
        /Users/ethanye/.gradle/caches/8.11/transforms/81b5d80ef0e1052d324c413062fe2902/transformed/simProPigeon2-26.1.3-headers
        /Users/ethanye/.gradle/caches/8.11/transforms/d7a217db37d6e8cc1ebb317f5040e246/transformed/simProCANrange-26.1.3-headers
        /Users/ethanye/.gradle/caches/8.11/transforms/7216d1791b135cc83a7fd671017698c1/transformed/simProCANdi-26.1.3-headers
        /Users/ethanye/.gradle/caches/8.11/transforms/8b9cd02ed104a5cc94c94e16a880e651/transformed/simProCANdle-26.1.3-headers
)
