# xacro
An XML preprocessor for xacro files to generate URDF files

## Status
Active development. Core expression evaluation and conditional processing implemented.
Reference: https://github.com/ros/xacro/wiki

## Implemented Features
- [X] include - Recursive file inclusion with relative path resolution
- [X] property - Properties with Python-like expression evaluation (arithmetic, comparisons, functions)
- [X] macro - Macro expansion with parameters and block parameters
- [X] insert_block - Block parameter insertion with nested support
- [X] if/unless - Conditional inclusion with type-preserving boolean evaluation

## Planned Features
- [ ] element - Dynamic element/attribute creation

## Expression Evaluation
Expressions in `${...}` are evaluated using pyisheval (Python-compatible expression evaluator):
- Arithmetic: `${width * 2 + offset}`
- Comparisons: `${x > 3}`, `${name == 'base'}`
- Built-in functions: `${radians(45)}`, `${len(items)}`
- List comprehensions: `${[x*2 for x in range(5)]}`
- Conditional expressions: `${width if width > 0.5 else 0.5}`

Type information is preserved for correct truthiness in conditionals.

## Optional Features
The following might be implemented:
- [ ] load_yaml - YAML property loading
- [ ] eval-comments - Comment-based evaluation

These are considered scope expansion beyond core xacro functionality.

## Non-Goals
The following shall not be implemented:
- Rospack-based path resolution
- Python subprocess evaluation

This package aims for minimal dependencies. No Python or ROS runtime dependencies required.
