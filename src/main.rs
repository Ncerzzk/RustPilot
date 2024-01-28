#![feature(lazy_cell)]

mod log;
mod msg_echo;
mod message;
mod msg_define;
mod pid;
mod param;

#[cfg(feature = "gzsim")]
mod gazebo_sim;
#[cfg(feature = "gzsim")]
mod gazebo_actuator;

mod att_control;
mod mixer;
mod imu_update;
mod elrs;

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

#[repr(C)]
#[derive(Clone, Copy)]
struct ThreadSpecificData  {
    stream:*mut UnixStream
}

pub static PTHREAD_KEY:LazyLock<u32>= LazyLock::new(||{
    let mut key:MaybeUninit<u32> = MaybeUninit::zeroed();
    unsafe{
        rpos::libc::pthread_key_create(key.as_mut_ptr(), Some(drop_specifidata));
        key.assume_init()
    }
});

fn set_thread_specifidata(data:ThreadSpecificData){
    let data = Box::new(data);
    let data = Box::leak(data);
    unsafe{
        rpos::libc::pthread_setspecific(*PTHREAD_KEY, &*data as *const ThreadSpecificData as *const libc::c_void );
    } 
}

unsafe extern "C" fn drop_specifidata(ptr:*mut libc::c_void){
    unsafe{
        drop(Box::from_raw(ptr as *mut ThreadSpecificData));
    };
}

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
        
        _ = remove_file(SOCKET_PATH);
        let stream = UnixListener::bind(SOCKET_PATH).unwrap();
        
        for client in stream.incoming(){

            let x= std::thread::spawn(move ||{
                let mut client = client.unwrap();
                let mut buffer = [0; 100];
                client.read(&mut buffer).unwrap();
                
                let cmd_raw= CStr::from_bytes_until_nul(&buffer).unwrap().to_str().unwrap().to_string();

                let cmd_with_args:Vec<_> = cmd_raw.split_whitespace().collect();
                assert!(cmd_with_args.len()>=1);
                println!("Client said:{},argc:{}",cmd_raw,cmd_with_args.len());

                let data =ThreadSpecificData{
                    stream: &mut client as *mut UnixStream
                };

                set_thread_specifidata(data);
                Module::get_module(cmd_with_args[0]).execute((cmd_with_args.len()) as u32, cmd_with_args.as_ptr());
                client.shutdown(std::net::Shutdown::Both).expect("failed to shutdown the socket!");
            });

        } 
    }else{
        let mut stream = UnixStream::connect(SOCKET_PATH).expect("please start a rustpilot server first!\n");// panic if the server is not runing.
        let other_args = cli.other.unwrap().join(" ");
        stream.write_all(other_args.as_bytes()).unwrap();

        let mut bufreader = BufReader::new(stream);
        let mut str_out:String = String::new();
        while let Ok(n) = bufreader.read_line(&mut str_out){
            if n == 0{break;}
            println!("{}",str_out);
            str_out.clear();
        }
    }

}
