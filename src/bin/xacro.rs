use clap::Parser;
use std::collections::HashMap;
use std::fs;
use std::io::{self, Write};
use std::path::PathBuf;

#[derive(Parser, Debug)]
#[command(name = "xacro")]
#[command(about = "XML preprocessor for xacro files to generate URDF", long_about = None)]
#[command(version)]
struct Args {
    /// Input xacro file
    input: PathBuf,

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
    use env_logger::{Builder, Env};
    use log::LevelFilter;

    // Start from environment configuration (e.g., RUST_LOG)
    let mut builder = Builder::from_env(Env::default());

    // Override based on verbosity flag
    let level = match verbosity {
        0 => LevelFilter::Off,
        1 => LevelFilter::Warn,
        2 => LevelFilter::Info,
        3 => LevelFilter::Debug,
        _ => LevelFilter::Trace,
    };

    builder.filter_level(level);

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

    if args.deps {
        // TODO: Implement dependency tracking
        anyhow::bail!("--deps flag not yet implemented");
    }

    // TODO: Wire mappings to xacro:arg substitution when implemented
    let _mappings = args.parse_mappings();

    // Process file
    let result = xacro::process_file(&args.input)
        .map_err(|e| anyhow::anyhow!("Failed to process xacro file: {}", e))?;

    // Output result
    if let Some(output_path) = &args.output {
        fs::write(output_path, result)?;
    } else {
        io::stdout().write_all(result.as_bytes())?;
    }

    Ok(())
}
