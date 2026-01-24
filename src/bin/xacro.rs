use clap::Parser;
use env_logger::{Builder, Env};
use log::LevelFilter;
use std::collections::HashMap;
use std::fs;
use std::io::{self, Read, Write};
use std::path::PathBuf;

#[derive(Parser, Debug)]
#[command(name = "xacro")]
#[command(about = "XML preprocessor for xacro files to generate URDF", long_about = None)]
#[command(version)]
struct Args {
    /// Input xacro file (use '-' for stdin, or omit to read from stdin)
    input: Option<PathBuf>,

    /// Write output to FILE instead of stdout
    #[arg(short = 'o', long = "output", value_name = "FILE")]
    output: Option<PathBuf>,

    /// Print file dependencies
    #[arg(long = "deps")]
    deps: bool,

    /// Process in read order (deprecated, now default)
    #[arg(long = "inorder", short = 'i')]
    inorder: bool,

    /// Quiet operation, suppress warnings
    #[arg(short = 'q', conflicts_with = "verbose")]
    quiet: bool,

    /// Increase verbosity (-v, -vv, -vvv)
    #[arg(short = 'v', long = "verbose", action = clap::ArgAction::Count)]
    verbose: u8,

    /// Set verbosity level explicitly (0-4)
    #[arg(
        long = "verbosity",
        value_name = "LEVEL",
        conflicts_with = "verbose",
        conflicts_with = "quiet"
    )]
    verbosity_level: Option<u8>,

    /// Python xacro compatibility mode (disabled by default)
    ///
    /// Accepts comma-separated list of modes or no value for all:
    ///   --compat              Enable all compatibility modes
    ///   --compat=all          Explicit all modes
    ///   --compat=namespace    Only lenient namespace validation
    ///   --compat=duplicate_params  Only accept duplicate parameters
    ///   --compat=namespace,duplicate_params  Multiple modes
    #[arg(long = "compat", value_name = "MODES", num_args = 0..=1, default_missing_value = "all")]
    compat: Option<String>,

    /// Additional arguments in key:=value format
    #[arg(trailing_var_arg = true, allow_hyphen_values = true)]
    extra_args: Vec<String>,
}

impl Args {
    fn get_verbosity(&self) -> u8 {
        if self.quiet {
            0
        } else if let Some(level) = self.verbosity_level {
            level.min(4)
        } else {
            1 + self.verbose.min(3)
        }
    }

    fn parse_mappings(&self) -> HashMap<String, String> {
        self.extra_args
            .iter()
            .filter_map(|arg| {
                if let Some((key, value)) = arg.split_once(":=") {
                    if key.is_empty() {
                        log::warn!("Ignoring mapping with empty key: '{}'", arg);
                        None
                    } else {
                        Some((key.to_string(), value.to_string()))
                    }
                } else {
                    if arg.contains('=') {
                        log::warn!("Use ':=' instead of '=' for mappings: '{}'", arg);
                    } else {
                        log::warn!(
                            "Ignoring unrecognized argument (expected key:=value format): '{}'",
                            arg
                        );
                    }
                    None
                }
            })
            .collect()
    }
}

fn init_logging(verbosity: u8) {
    // Start from environment configuration (e.g., RUST_LOG)
    let mut builder = Builder::from_env(Env::default());

    // Override based on verbosity flag
    let level = match verbosity {
        0 => LevelFilter::Warn, // Default: warnings only
        1 => LevelFilter::Warn,
        2 => LevelFilter::Info,
        3 => LevelFilter::Debug,
        _ => LevelFilter::Trace,
    };

    builder.filter_level(level);

    // Always enable Info level for xacro crate to support print_location()
    // This ensures xacro.print_location() works even without -v flags
    builder.filter_module("xacro", LevelFilter::Info);

    // Avoid panicking if a logger was already initialized
    let _ = builder.try_init();
}

fn main() -> anyhow::Result<()> {
    let args = Args::parse();

    let verbosity = args.get_verbosity();
    init_logging(verbosity);

    // Handle deprecated --inorder flag
    if args.inorder {
        log::warn!("in-order processing became default in ROS Melodic. You can drop the option.");
    }

    // Parse mappings (key:=value arguments for xacro:arg)
    let mappings = args.parse_mappings();

    // Parse compat mode from optional string argument
    let compat_mode = args
        .compat
        .as_deref()
        .map(|s| s.parse())
        .transpose()?
        .unwrap_or_default();

    // Build processor with mappings, compat mode, ROS extensions, and ROS YAML units
    // ROS features are enabled by default in CLI for user convenience
    let mut builder = xacro_rs::XacroProcessor::builder()
        .with_args(mappings)
        .with_compat_mode(compat_mode)
        .with_extension(Box::new(xacro_rs::extensions::FindExtension::new()))
        .with_extension(Box::new(xacro_rs::extensions::OptEnvExtension::new()));

    #[cfg(feature = "yaml")]
    {
        builder = builder.with_ros_yaml_units();
    }

    let processor = builder.build();

    // Determine if reading from stdin
    let use_stdin = match &args.input {
        None => true,
        Some(path) => path.as_os_str() == "-",
    };

    if args.deps {
        if use_stdin {
            anyhow::bail!("--deps flag is not supported when reading from stdin");
        }

        // Process file and get dependencies (already deduplicated and sorted by library)
        let (_, includes) = processor
            .run_with_deps(
                args.input
                    .as_ref()
                    .expect("input must be Some when not using stdin"),
            )
            .map_err(|e| anyhow::anyhow!("Failed to process xacro file: {}", e))?;

        // Output space-separated list of included files (matches Python xacro behavior)
        // Note: Python xacro outputs WITHOUT trailing newline
        let deps_str = includes
            .iter()
            .map(|p| p.display().to_string())
            .collect::<Vec<_>>()
            .join(" ");
        print!("{}", deps_str);
        return Ok(());
    }

    let result = if use_stdin {
        // Read from stdin
        let mut content = String::new();
        io::stdin()
            .read_to_string(&mut content)
            .map_err(|e| anyhow::anyhow!("Failed to read from stdin: {}", e))?;

        processor
            .run_from_string(&content)
            .map_err(|e| anyhow::anyhow!("Failed to process xacro from stdin: {}", e))?
    } else {
        processor
            .run(
                args.input
                    .as_ref()
                    .expect("input must be Some when not using stdin"),
            )
            .map_err(|e| anyhow::anyhow!("Failed to process xacro file: {}", e))?
    };

    // Output result
    if let Some(output_path) = &args.output {
        fs::write(output_path, result)?;
    } else {
        io::stdout().write_all(result.as_bytes())?;
    }

    Ok(())
}
