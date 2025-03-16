use rpos::{
    thread_logln,
};
use clap::Parser;
use std::sync::Arc;

use mavlink::error::MessageReadError;
use crate::{
    elrs::client_process_args};

#[derive(Parser, Clone)]
struct Cli {
    #[arg(short, long, value_name = "addr")]
    addr: String,
}


pub unsafe fn init_mavlink_gs(argc: u32, argv: *const &str) {
    let args =  client_process_args::<Cli>(argc, argv).unwrap();

    let mavconn = mavlink::connect::<mavlink::common::MavMessage>(&("udpout:".to_string() + &args.addr)).unwrap();

    let mavconn = Arc::new(mavconn);

    thread_logln!("mavlink connnect ok!");

    let heart_beat_msg: mavlink::common::MavMessage = mavlink::common::MavMessage::HEARTBEAT(mavlink::common::HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: mavlink::common::MavType::MAV_TYPE_QUADROTOR,
        autopilot: mavlink::common::MavAutopilot::MAV_AUTOPILOT_GENERIC,
        base_mode: mavlink::common::MavModeFlag::all(),
        system_status: mavlink::common::MavState::MAV_STATE_STANDBY,
        mavlink_version: 0x3,
    });

    std::thread::spawn({
        let mavconn = mavconn.clone();
        move || loop{
            let _ = mavconn.send_default(&heart_beat_msg);
            std::thread::sleep(std::time::Duration::from_secs(1));
        }

    });

    
    loop{
        match mavconn.recv() {
            Ok((_header, msg)) => {
                thread_logln!("received: {msg:?}");
            }
            Err(MessageReadError::Io(e)) => {
                if e.kind() == std::io::ErrorKind::WouldBlock {
                    //no messages currently available to receive -- wait a while
                    std::thread::sleep(std::time::Duration::from_secs(1));
                    continue;
                } else {
                    thread_logln!("recv error: {e:?}");
                    break;
                }
            }
            // messages that didn't get through due to parser errors are ignored
            _ => {}
        }
    }
    
}

#[rpos::ctor::ctor]
fn register() {
    rpos::module::Module::register("mavlink_gs", |a, b| unsafe { init_mavlink_gs(a, b) });

}
