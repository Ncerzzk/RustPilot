use core::panic;
use std::{env, fs::File, io::Write, path::Path, process::Command};


fn get_cargo_target_dir() -> Result<std::path::PathBuf, Box<dyn std::error::Error>> {
    let out_dir = std::path::PathBuf::from(std::env::var("OUT_DIR")?);
    let profile = std::env::var("PROFILE")?;
    let mut target_dir = None;
    let mut sub_path = out_dir.as_path();
    while let Some(parent) = sub_path.parent() {
        if parent.ends_with(&profile) {
            target_dir = Some(parent);
            break;
        }
        sub_path = parent;
    }
    let target_dir = target_dir.ok_or("not found")?;
    Ok(target_dir.to_path_buf())
}

fn main() {
    // println!("{:?}",get_cargo_target_dir().unwrap());
    // println!("{:?}",std::env::var("CARGO_MANIFEST_DIR").unwrap());
    // let result = Command::new("pwd")  
    // .output().unwrap(); 
    // println!("{:?}",String::from_utf8_lossy(&result.stdout));
    // panic!("x");
//     let alias_script_path = get_cargo_target_dir().unwrap().join("alias.sh");
//     println!("{}",alias_script_path.to_str().unwrap());

//     let mut file = File::create(&alias_script_path).unwrap();

//     let content = r"
// alias param='./rust_pilot param'
// alias gazebo_sim='./rust_pilot gazebo_sim'
// alias
//     ";

//     file.write(content.as_bytes()).unwrap();

}