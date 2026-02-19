# microstrain inertial

[![crates](https://img.shields.io/crates/v/microstrain-inertial.svg)](https://crates.io/crates/microstrain-inertial)
[![docs](https://img.shields.io/docsrs/microstrain-inertial)](https://docs.rs/microstrain-inertial)

Rust implementation of the Microstrain Inertial Packet (MIP) protocol used for communicating with
Microstrain 3DM inertial measurement units.

## Background

This was created and tested with the 3DM-CV7, but should be generally applicable to other
microstrain devices -- though it may not have the complete set of messages implemented and require
extending. It supporst both no_std/no_alloc microcontroller environments, as well as on a PC.

See the [crate documentation](https://docs.rs/zencan-node) for usage info.

## Sponsors

Development was sponsored by the [Rising Tide Research Foundation](https://risingtideresearch.com/).

