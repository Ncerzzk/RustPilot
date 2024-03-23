#![feature(lazy_cell)]
#![feature(trait_upcasting)]

mod msg_define;
mod pid;
mod param;

#[cfg(feature = "gzsim")]
mod gazebo_sim;
#[cfg(feature = "gzsim")]
mod gazebo_actuator;

mod fake_linux_input;
mod att_control;
mod mixer;
mod imu_update;
mod elrs;
mod fpga_spi_pwm;

use std::ffi::CStr;
use std::io::{Read, Write, BufReader, BufRead};
use std::mem::MaybeUninit;
use std::os::unix::net::{UnixStream, UnixListener};
use std::env;
use std::fs::remove_file;
use std::ptr::{null_mut, null};
use std::sync::LazyLock;
use rpos::module::Module;
use rpos::libc;

use clap::Parser;
use rpos::server_client::{server_init, Client};

/* main for debug */
// fn main(){
//     let a = ["sim_gz","/home/ncer/RustPilot/sim/quadcopter.toml"];
//     init_gazebo_sim(2, a.as_ptr());

//     let a = ["gazebo_actuator"];
//     init_gz_actuator(1,a.as_ptr());

//     unsafe{init_mixer(0,null_mut());}
//     init_att_control(0,null_mut());

//     init_imu_update(0,null());

//     unsafe {assert_eq!(libc::mlockall(1 | 2),0)};

//     gz::transport::wait_for_shutdown();

// }
#[derive(Parser)]
#[command(author, version, about, long_about = None, arg_required_else_help(true))]
struct Cli{

    #[arg(long)]
    server:bool,

    #[arg(short,long)]
    start_script:Option<String>,

    /// commands send by clients.
    #[arg(value_name="client commands")]
    other:Option<Vec<String>>
}

fn main() {
    let cli = Cli::parse();

    let args:Vec<String> = env::args().collect();
    const SOCKET_PATH: &str = "./rpsocket";

    if cli.server{
        let hello_txt = r"
        ____                    __     ____     _     __          __
        / __ \  __  __   _____  / /_   / __ \   (_)   / /  ____   / /_
       / /_/ / / / / /  / ___/ / __/  / /_/ /  / /   / /  / __ \ / __/
      / _, _/ / /_/ /  (__  ) / /_   / ____/  / /   / /  / /_/ // /_
     /_/ |_|  \__,_/  /____/  \__/  /_/      /_/   /_/   \____/ \__/";  // slant
        println!("{}",hello_txt);
        server_init(SOCKET_PATH).unwrap();
    }else{
        let mut client = Client::new(SOCKET_PATH).unwrap();
        client.send_str(cli.other.unwrap().join(" ").as_str());
        client.block_read();
    }

}
