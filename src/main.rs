mod gazebo_sim;
use std::io::{Read, Write};
use std::os::unix::net::{UnixStream, UnixListener};
use std::env::{self};
use std::fs::{self, remove_file};
use std::str::SplitWhitespace;
use rpos::module::Module;

fn main() {
    let is_server;
    let args:Vec<String> = env::args().collect();
    const SOCKET_PATH: &str = "./rpsocket";
    if args.len()> 1 && args[1] == "--server"{
        is_server = true;
    }else{
        is_server = false;
    }

    
    if is_server{
        _ = remove_file(SOCKET_PATH);
        let stream = UnixListener::bind(SOCKET_PATH).unwrap();
        let mut cmd_raw = String::new();
        for client in stream.incoming(){
            client.unwrap().read_to_string(&mut cmd_raw).unwrap();
            let cmd_with_args:Vec<_> = cmd_raw.split_whitespace().collect();
            assert!(cmd_with_args.len()>=1);
            Module::get_module(cmd_with_args[0]).execute((cmd_with_args.len() - 1) as u32, cmd_with_args[1..].as_ptr());
            println!("Client said: {}",cmd_raw);
        }
    }else{
        let mut stream = UnixStream::connect(SOCKET_PATH).unwrap(); // panic if the server is not runing.
        let other_args = args[1..].join(" ");
        stream.write_all(other_args.as_bytes()).unwrap();
    }

    println!("Hello, world!");

    //gazebo_sim::init_gazebo_sim();
}
