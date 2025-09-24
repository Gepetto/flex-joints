# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

Releases are available on the [github repository](https://github.com/Gepetto/flex-joints/releases).

## [Unreleased]

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

[Unreleased]: https://github.com/Gepetto/flex-joints/compare/v1.1.0...HEAD
[1.1.0]: https://github.com/Gepetto/flex-joints/releases/tag/v1.1.0
