# Examples

This directory contains examples demonstrating various ways to use the xacro library.

**Note:** While xacro originated in the ROS ecosystem for URDF files, it's a **general-purpose XML macro processor** that works with any XML document. See `generic_xml.rs` for non-ROS usage.

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

### generic_xml.rs
Using xacro as a general-purpose XML macro processor (no ROS dependencies).

```bash
cargo run --example generic_xml
```

Demonstrates:
- XML macros for configuration files or any XML documents
- Properties and arithmetic expressions in generic XML
- Conditional sections with `<xacro:if>`
- **No ROS-specific features required**

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
Full-featured processor configuration with ROS extensions (optional).

```bash
cargo run --example advanced --features yaml
```

Demonstrates:
- Registering ROS-specific extensions: `$(find)`, `$(optenv)`
- Enabling YAML support
- Using compatibility modes for legacy URDFs
- Getting dependency information with `run_with_deps()`

**Note:** ROS extensions are optional - see `generic_xml.rs` for non-ROS usage.

## Example Input Files

- **robot.xacro** - Simple robot with properties and macros (ROS/URDF example)
- **robot_with_args.xacro** - Robot using `<xacro:arg>` for parameterization (ROS/URDF example)

**Note:** The `generic_xml.rs` example has XML inline, demonstrating non-ROS usage.

## Running Examples

All examples can be run with:

```bash
cargo run --example <example_name>
```

For examples requiring the YAML feature:

```bash
cargo run --example <example_name> --features yaml
```
