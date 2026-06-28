# Third-Party Notices

This repository contains original code by Ethan Ye and third-party or
derived code that remains subject to its own license terms.

## Derived from AdvantageKit

Much of the logger infrastructure under `src/main/cpp/akit` is a C++ port
derived from the original Java AdvantageKit project by Littleton Robotics /
Mechanical Advantage.

- Upstream project: https://github.com/Mechanical-Advantage/AdvantageKit
- Local license copy: `third_party/AdvantageKit-LICENSE.txt`

The upstream license includes a non-endorsement clause covering the names
Littleton Robotics, Mechanical Advantage, and AdvantageKit.

## WPILib

This repository includes WPILib-generated template code and depends on
WPILib libraries.

- Local license copy: `WPILib-License.md`

## Vendored Header Libraries

This repository vendors the following header-only libraries:

- `third_party/magic_enum` under the MIT License
- `third_party/pfr` under the Boost Software License 1.0

Local license copies are included in those directories.

## External Dependencies

Additional libraries may be fetched via vendordep manifests in `vendordeps/`.
Those dependencies are not fully vendored into this repository and may carry
their own redistribution requirements.
