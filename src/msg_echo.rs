use std::any::Any;

use clap::Parser;
use rpos::{msg::get_new_rx_of_message, channel::Receiver, thread_logln};

use crate::{elrs::client_process_args, msg_define::*, mixer::MixerOutputMsg};

#[derive(Parser, Debug)]
#[command(name = "msg_echo", version, about="a dirty implement to debug")]
struct Cli {
    /// Name of the person to greet
    #[arg(short, long, default_value_t = 1)]
    count: i32,

    #[arg(short, long)]
    topic: String,
}


fn msg_echo_main(argc:u32, argv:*const &str){
    if let Some(args) = client_process_args::<Cli>(argc, argv){
        let mut func:Box<dyn FnMut() -> ()>;
        if args.topic == "rc_input"{
            let mut rx = get_new_rx_of_message::<RcInputMsg>(&args.topic).unwrap();
            func = Box::new(move ||{thread_logln!("{:?}",rx.read());});
            
        }else if args.topic == "controller_output0"{
            let mut rx = Box::new(get_new_rx_of_message::<ControllerOutputGroupMsg>(&args.topic).unwrap()); 
            func = Box::new(move ||{thread_logln!("{:?}",rx.read());});
        }else if args.topic == "mixer_output"{
            let mut rx = Box::new(get_new_rx_of_message::<MixerOutputMsg>(&args.topic).unwrap()); 
            func = Box::new(move ||{thread_logln!("{:?}",rx.read());});
        }else{
            func = Box::new(||{})
        }
        let mut cnt = 0;
        while args.count <=0  || cnt < args.count{
            func();
            cnt +=1;
        }
    }

}

#[rpos::ctor::ctor]
fn register(){
    crate::Module::register("msg_echo", msg_echo_main);
}