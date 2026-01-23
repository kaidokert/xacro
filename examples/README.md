# Examples

This directory contains examples demonstrating various ways to use the xacro library.

## Basic Examples

### basic.rs
The simplest way to process a xacro file.

```bash
cargo run --example basic
```

Demonstrates:
- Using the `process_file()` convenience function
- Processing a xacro file with properties and macros

### stdin.rs
Reading xacro content from standard input.

```bash
echo '<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test"/>' | cargo run --example stdin
# or
cargo run --example stdin < examples/robot.xacro
```

Demonstrates:
- Using `process_string()` for in-memory processing
- Pipeline-friendly processing

## Intermediate Examples

### with_args.rs
Using the builder API to pass arguments to xacro files.

```bash
cargo run --example with_args
```

Demonstrates:
- Building a custom `XacroProcessor` with arguments
- Using `<xacro:arg>` in xacro files
- Conditional content based on arguments

## Advanced Examples

### advanced.rs
Full-featured processor configuration.

```bash
cargo run --example advanced --features yaml
```

Demonstrates:
- Registering ROS extensions ($(find), $(optenv))
- Enabling YAML support
- Using compatibility modes
- Getting dependency information with `run_with_deps()`

## Example Input Files

- **robot.xacro** - Simple robot with properties and macros
- **robot_with_args.xacro** - Robot using `<xacro:arg>` for parameterization

## Running Examples

All examples can be run with:

```bash
cargo run --example <example_name>
```

For examples requiring the YAML feature:

```bash
cargo run --example <example_name> --features yaml
```
