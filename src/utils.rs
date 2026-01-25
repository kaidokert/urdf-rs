use crate::deserialize::Robot;
use crate::errors::*;
use crate::funcs::*;

use regex::Regex;
use std::borrow::Cow;
use std::path::Path;
use std::process::Command;
use std::sync::LazyLock;

use std::env;
use std::fs;

#[cfg(feature = "rust-xacro")]
use std::{cell::RefCell, collections::HashMap, path::PathBuf};

// TODO: This is a hack, re-work this with either a better interface or simply
// read the file twice.
#[cfg(feature = "rust-xacro")]
thread_local! {
    static FOUND_PACKAGES: RefCell<HashMap<String, PathBuf>> = RefCell::new(HashMap::new());
}

#[cfg(feature = "rust-xacro")]
pub fn convert_xacro_to_urdf_with_args<P>(filename: P, args: &[(String, String)]) -> Result<String>
where
    P: AsRef<Path>,
{
    let filename = filename.as_ref();
    let mut params = std::collections::HashMap::new();
    for (k, v) in args {
        params.insert(k.clone(), v.clone());
    }

    let processor = xacro_rs::XacroProcessor::builder()
        .with_args(params)
        .with_extension(Box::new(xacro_rs::extensions::FindExtension::new()))
        .with_extension(Box::new(xacro_rs::extensions::OptEnvExtension::new()))
        .build();

    let result = processor.run(filename).map_err(|e| {
        ErrorKind::Other(format!(
            "failed to process xacro file {}: {}",
            filename.display(),
            e
        ))
    })?;

    // Extract found packages from FindExtension and cache them for later use
    let extensions = processor.extensions();
    for ext in extensions.iter() {
        if let Some(find_ext) = ext
            .as_any()
            .downcast_ref::<xacro_rs::extensions::FindExtension>()
        {
            let found_packages = find_ext.get_found_packages();
            FOUND_PACKAGES.with(|packages| {
                packages.borrow_mut().clone_from(&found_packages);
            });
            break;
        }
    }

    Ok(result)
}

#[cfg(not(feature = "rust-xacro"))]
pub fn convert_xacro_to_urdf_with_args<P>(filename: P, args: &[(String, String)]) -> Result<String>
where
    P: AsRef<Path>,
{
    let filename = filename.as_ref();
    let output = Command::new("rosrun")
        .args(["xacro", "xacro", "--inorder"])
        .arg(filename)
        .args(args.iter().map(|(k, v)| format!("{k}:={v}")))
        .output()
        .or_else(|_| {
            Command::new("xacro")
                .arg(filename)
                .args(args.iter().map(|(k, v)| format!("{k}:={v}")))
                .output()
        })
        .map_err(|e| {
            format!("failed to execute xacro; consider installing xacro by `apt-get install ros-*-xacro`: {e}")
        })?;
    if output.status.success() {
        Ok(String::from_utf8(output.stdout)?)
    } else {
        Err(ErrorKind::Command {
            msg: format!("failed to xacro for {}", filename.display()),
            stdout: String::from_utf8_lossy(&output.stdout).into_owned(),
            stderr: String::from_utf8_lossy(&output.stderr).into_owned(),
        }
        .into())
    }
}

pub fn convert_xacro_to_urdf<P>(filename: P) -> Result<String>
where
    P: AsRef<Path>,
{
    convert_xacro_to_urdf_with_args(filename, &[])
}

pub fn rospack_find(package: &str) -> Result<String> {
    // 1. Check thread-local cache from xacro processing (if rust-xacro feature enabled)
    #[cfg(feature = "rust-xacro")]
    {
        let cached_path = FOUND_PACKAGES.with(|packages| packages.borrow().get(package).cloned());
        if let Some(path) = cached_path {
            return Ok(path.to_string_lossy().to_string());
        }
    }

    // 2. Check ROS_PACKAGE_PATH
    if let Ok(ros_path) = env::var("ROS_PACKAGE_PATH") {
        let check_package_name = |dir: &std::path::Path| -> Option<String> {
            fs::read_to_string(dir.join("package.xml"))
                .ok()
                .filter(|content| content.contains(&format!("<name>{}</name>", package)))
                .map(|_| dir.to_string_lossy().to_string())
        };

        if let Some(found) = env::split_paths(&ros_path)
            .filter(|path| path.exists())
            .find_map(|path| {
                // Check if this directory itself is the package
                check_package_name(&path)
                    // Or check subdirectories
                    .or_else(|| {
                        fs::read_dir(&path)
                            .ok()?
                            .flatten()
                            .filter(|e| e.path().is_dir())
                            .find_map(|entry| check_package_name(&entry.path()))
                    })
            })
        {
            return Ok(found);
        }
    }

    // 3. Fall back to rospack/ros2 commands
    let output = Command::new("rospack")
        .arg("find")
        .arg(package)
        .output()
        // support ROS2
        .or_else(|_| {
            Command::new("ros2")
                .args(["pkg", "prefix", "--share"])
                .arg(package)
                .output()
        })
        .map_err(|e| {
            format!("failed to execute neither `rospack` nor `ros2 pkg`; consider installing ROS or setting ROS_PACKAGE_PATH: {e}")
        })?;
    if output.status.success() {
        Ok(String::from_utf8(output.stdout)?.trim().to_string())
    } else {
        Err(ErrorKind::Command {
            msg: format!("failed to find ros package {package}"),
            stdout: String::from_utf8_lossy(&output.stdout).into_owned(),
            stderr: String::from_utf8_lossy(&output.stderr).into_owned(),
        }
        .into())
    }
}

// Note: We return Result here, although there is currently no branch that returns an error.
// This is to avoid a breaking change when changing the error about package:// from panic to error.
pub fn expand_package_path<'a>(filename: &'a str, base_dir: Option<&Path>) -> Result<Cow<'a, str>> {
    static RE: LazyLock<Regex> = LazyLock::new(|| Regex::new("^package://(\\w+)/").unwrap());

    Ok(if filename.starts_with("package://") {
        RE.replace(filename, |ma: &regex::Captures<'_>| {
            // TODO: It's better to propagate the error to the caller,
            // but regex doesn't provide API like try_replace.
            let found_path = rospack_find(&ma[1]).unwrap();
            found_path + "/"
        })
    } else if filename.starts_with("https://") || filename.starts_with("http://") {
        filename.into()
    } else if let Some(abs_path) = filename.strip_prefix("file://") {
        abs_path.into()
    } else if let Some(base_dir) = base_dir {
        let mut relative_path_from_urdf = base_dir.to_owned();
        relative_path_from_urdf.push(filename);
        relative_path_from_urdf
            .into_os_string()
            .into_string()
            .unwrap()
            .into()
    } else {
        filename.into()
    })
}

pub fn read_urdf_or_xacro_with_args<P>(input_path: P, args: &[(String, String)]) -> Result<Robot>
where
    P: AsRef<Path>,
{
    let input_path = input_path.as_ref();
    if let Some(ext) = input_path.extension() {
        if ext == "xacro" {
            let urdf_utf = convert_xacro_to_urdf_with_args(input_path, args)?;
            read_from_string(&urdf_utf)
        } else {
            read_file(input_path)
        }
    } else {
        Err(ErrorKind::Other(format!(
            "failed to get extension from {}",
            input_path.display()
        ))
        .into())
    }
}

pub fn read_urdf_or_xacro<P>(input_path: P) -> Result<Robot>
where
    P: AsRef<Path>,
{
    read_urdf_or_xacro_with_args(input_path, &[])
}

#[test]
fn it_works() {
    // test only for not packages
    assert_eq!(expand_package_path("home/aaa", None).unwrap(), "home/aaa");
    assert_eq!(
        expand_package_path("home/aaa.obj", Some(Path::new(""))).unwrap(),
        "home/aaa.obj"
    );
    assert_eq!(
        expand_package_path("home/aaa.obj", Some(Path::new("/var"))).unwrap(),
        "/var/home/aaa.obj"
    );
    assert_eq!(
        expand_package_path("/home/aaa.obj", Some(Path::new(""))).unwrap(),
        "/home/aaa.obj"
    );
    assert_eq!(
        expand_package_path("file:///home/aaa.obj", Some(Path::new("/var"))).unwrap(),
        "/home/aaa.obj"
    );
    assert_eq!(
        expand_package_path("http://aaa.obj", Some(Path::new("/var"))).unwrap(),
        "http://aaa.obj"
    );
    assert_eq!(
        expand_package_path("https://aaa.obj", Some(Path::new("/var"))).unwrap(),
        "https://aaa.obj"
    );
    assert!(read_urdf_or_xacro("sample.urdf").is_ok());
    assert!(read_urdf_or_xacro("sample_urdf").is_err());
}
