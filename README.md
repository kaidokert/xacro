# xacro
A xml preprocessor for xacro files to generate URDF files

Reference: http://wiki.ros.org/xacro

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
