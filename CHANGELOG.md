# Changelog

All notable changes to this library are documented in this file. The format is
based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and this
project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.1.0] - 2026-06-25

### Changed

- Migrated to a per-instance I2C bus handle so multiple buses or devices can be used without global transport state.
- Reconciled file layout, naming, and tooling with the sibling multiplatform libraries.

### Added

- Arduino Library Manager metadata (`library.properties`).

## [1.0.0] - 2026-02-09

### Added

- Initial release: multiplatform driver for the TI DRV8214 brushed-DC motor driver over I2C (Arduino, ESP32, STM32, RP2040), with speed/current monitoring, stall detection, CI across all platforms, and example sketches.
