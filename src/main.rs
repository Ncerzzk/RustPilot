#![feature(lazy_cell)]
mod gazebo_sim;
mod log;
mod msg_echo;
mod message;
mod msg_define;

mod att_control;
mod mixer;

use std::ffi::CStr;
use std::io::{Read, Write};
use std::mem::MaybeUninit;
use std::os::unix::net::{UnixStream, UnixListener};
use std::env;
use std::fs::remove_file;
use std::sync::LazyLock;
use gazebo_sim::init_gazebo_sim;
use rpos::module::Module;
use rpos::libc;

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
        let _ = Box::from_raw(ptr as *mut ThreadSpecificData);
    };
}

/* main for debug */
fn main(){
    let a = ["sim_gz","/home/ncer/RustPilot/sim/quadcopter.toml"];
    init_gazebo_sim(2, a.as_ptr());
    gz::transport::wait_for_shutdown();

}

// fn main() {
//     let is_server;
//     let args:Vec<String> = env::args().collect();
//     const SOCKET_PATH: &str = "./rpsocket";
//     if args.len()> 1 && args[1] == "--server"{
//         is_server = true;
//     }else{
//         is_server = false;
//     }

    
//     if is_server{
//         _ = remove_file(SOCKET_PATH);
//         let stream = UnixListener::bind(SOCKET_PATH).unwrap();
        
//         for client in stream.incoming(){

//             let x= std::thread::spawn(move ||{
//                 let mut client = client.unwrap();
//                 let mut buffer = [0; 100];
//                 client.read(&mut buffer).unwrap();
                
//                 let cmd_raw= CStr::from_bytes_until_nul(&buffer).unwrap().to_str().unwrap().to_string();

//                 let cmd_with_args:Vec<_> = cmd_raw.split_whitespace().collect();
//                 assert!(cmd_with_args.len()>=1);
//                 println!("Client said:{},argc:{}",cmd_raw,cmd_with_args.len());

//                 let data =ThreadSpecificData{
//                     stream: &mut client as *mut UnixStream
//                 };

//                 set_thread_specifidata(data);
//                 Module::get_module(cmd_with_args[0]).execute((cmd_with_args.len()) as u32, cmd_with_args.as_ptr());
//                 client.shutdown(std::net::Shutdown::Both).expect("failed to shutdown the socket!");
//             });

//         } 
//     }else{
//         let mut stream = UnixStream::connect(SOCKET_PATH).unwrap(); // panic if the server is not runing.
//         let other_args = args[1..].join(" ");
//         stream.write_all(other_args.as_bytes()).unwrap();
//         stream.flush().unwrap();
//         let mut out = String::new();
//         stream.read_to_string(&mut out).unwrap(); // TODO: change to read output immediately
//         println!("{}",out);
//     }

//     println!("Hello, world!");

//     //gazebo_sim::init_gazebo_sim();
// }
