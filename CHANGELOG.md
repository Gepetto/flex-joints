# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

Releases are available on the [github repository](https://github.com/Gepetto/flex-joints/releases).

## [Unreleased]

## [1.3.0] - 2026-04-30

- Install the Python script added in the last release.
- nix: switch to flakoboros
- drop submodule

## [1.2.0] - 2025-10-07

### added

- Python scripts to identify the stiffness of the virtual flexible joints from
  data.

### changed

- ⚠️ The unit-tests framework was changed from boots-unittests to doctest
- Upgrade of the CMake minimal version to 3.22.1
- Upgrade of the handling of the jrl-cmakemodules dependency in the CMake

## [1.1.0] - 2023-02-02

This Release serves as the first release of this package. Though there are other
tags previous to the latest one.

### Added

- Add setter for hip stiffness
- Add specific setter for damping and do not update damping from stiffness value

### Changed

- ⚠️ Disable python builds by default

## [1.0.3] - 2022-08-11

### Changed

- Remove some log prints.
- ⚠️ Add Getter for the deflection values
- Add the option to install only the python bindings.
- sync cmake submodule.
- Format

## [1.0.2] - 2022-08-11

### Added

- Function to correct the posture with force.

### Changed

- Update the submodule
- Update the eigenpy version
- format
- ⚠️ separate the computation of flexible torque

## [1.0.0] - 2022-06-24

Initial release

[Unreleased]: https://github.com/Gepetto/flex-joints/compare/v1.3.0...HEAD
[1.3.0]: https://github.com/Gepetto/flex-joints/compare/v1.2.0...v1.3.0
[1.2.0]: https://github.com/Gepetto/flex-joints/compare/v1.1.0...v1.2.0
[1.1.0]: https://github.com/Gepetto/flex-joints/compare/v1.0.3...v1.1.0
[1.0.3]: https://github.com/Gepetto/flex-joints/compare/v1.0.2...v1.0.3
[1.0.2]: https://github.com/Gepetto/flex-joints/compare/v1.0.0...v1.0.2
[1.0.0]: https://github.com/Gepetto/flex-joints/releases/tag/v1.0.0
