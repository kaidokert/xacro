# xacro-rs

<!-- Badges to add after publication:
[![crate](https://img.shields.io/crates/v/xacro-rs.svg)](https://crates.io/crates/xacro-rs)
[![documentation](https://docs.rs/xacro-rs/badge.svg)](https://docs.rs/xacro-rs/)
-->
[![Build and test](https://github.com/kaidokert/xacro/actions/workflows/rust.yaml/badge.svg)](https://github.com/kaidokert/xacro/actions/workflows/rust.yaml)
[![Coverage Status](https://coveralls.io/repos/github/kaidokert/xacro/badge.svg?branch=main)](https://coveralls.io/github/kaidokert/xacro?branch=main)

An XML preprocessor for xacro files to generate URDF files

Reference: http://wiki.ros.org/xacro

## Installation

### As a library

```bash
cargo add xacro-rs
```

### As a command-line tool

```bash
cargo install xacro-rs
```

## Usage

### Command-line tool

Process a xacro file:

```bash
xacro-rs robot.xacro > robot.urdf
```

With arguments:

```bash
xacro-rs robot.xacro robot_name:=myrobot wheel_radius:=0.5
```

From stdin:

```bash
cat robot.xacro | xacro-rs - > robot.urdf
```

See `xacro-rs --help` for all options.

### Library usage

See the [examples/](examples/) directory for runnable examples including:
- Basic file processing
- Processing from stdin
- Using arguments and the builder API
- Custom extensions
- Generic XML macros (non-ROS usage)

API documentation: https://docs.rs/xacro-rs

## Status

Core xacro functionality implemented:
- [X] macro
- [X] include
- [X] insert_block
- [X] property
- [X] if
- [X] unless
- [X] load_yaml
- [X] Basic Python expression evaluation (see limitations)
- [X] Extensions: $(find), $(env), $(optenv), $(cwd)

Not implemented:

- [ ] element (dynamic XML generation)
- [ ] attribute (dynamic attributes)

See [KNOWN_ISSUES.md](KNOWN_ISSUES.md) for limitations and deviations from Python xacro.

No ROS dependencies required. Package resolution via `RUST_XACRO_PACKAGE_MAP` environment variable.
