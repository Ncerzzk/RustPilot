use clap::Parser;
use rpos::{thread_logln, msg::get_new_tx_of_message};
use std::{sync::Arc};

use crate::{
    param::{self, ParameterData}, msg_define::RcInputMsg,
};
use mavlink::{
    common::{self, MavMessage},
    error::MessageReadError,
    Message,
};

#[derive(Parser, Clone)]
struct Cli {
    #[arg(short, long, value_name = "addr")]
    addr: String,

    #[arg(long, help = "use joystick provided by ground station")]
    joystick: bool,
}

fn get_mav_paramtype(p: &param::ParameterData) -> common::MavParamType {
    match p {
        ParameterData::Bool(_) => common::MavParamType::MAV_PARAM_TYPE_UINT8,
        ParameterData::Int(_) => common::MavParamType::MAV_PARAM_TYPE_INT32,
        ParameterData::Float(_) => common::MavParamType::MAV_PARAM_TYPE_REAL32,
    }
}

fn get_paramdata_in_mav(p: &common::MavParamType, value: f32) -> Option<param::ParameterData> {
    match p {
        common::MavParamType::MAV_PARAM_TYPE_UINT8 => unsafe {
            let b: u32 = std::mem::transmute(value);

            thread_logln!("set mav bool:{}", b != 0);
            Some(param::ParameterData::Bool(b != 0))
        },
        common::MavParamType::MAV_PARAM_TYPE_INT32 => unsafe {
            let b: i32 = std::mem::transmute(value);

            thread_logln!("set mav int:{}", b);
            Some(param::ParameterData::Int(b))
        },
        common::MavParamType::MAV_PARAM_TYPE_REAL32 => {
            thread_logln!("set mav float:{}", value);
            Some(param::ParameterData::Float(value))
        }
        _ => None,
    }
}

pub unsafe fn init_mavlink_gs(argc: u32, argv: *const &str) {
    let args = crate::basic::client_process_args::<Cli>(argc, argv).unwrap();

    let mavconn = mavlink::connect::<MavMessage>(&("udpout:".to_string() + &args.addr)).unwrap();

    let mavconn = Arc::new(mavconn);

    thread_logln!("mavlink connnect ok!");

    let heart_beat_msg: MavMessage = MavMessage::HEARTBEAT(common::HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: common::MavType::MAV_TYPE_QUADROTOR,
        autopilot: common::MavAutopilot::MAV_AUTOPILOT_GENERIC,
        base_mode: common::MavModeFlag::MAV_MODE_FLAG_MANUAL_INPUT_ENABLED,
        system_status: common::MavState::MAV_STATE_STANDBY,
        mavlink_version: 0x3,
    });

    let header = mavlink::MavHeader {
        system_id: 1,
        component_id: 1,
        sequence: 0,
    };

    param::add_param("bool_test", ParameterData::Bool(true));
    param::add_param("int_test", ParameterData::Int(32));
    param::add_param("float_test", ParameterData::Float(32.0));

    std::thread::spawn({
        let mavconn = mavconn.clone();
        move || loop {
            //let _ = mavconn.send_default(&heart_beat_msg);
            let _ = mavconn.send(&header, &heart_beat_msg);
            std::thread::sleep(std::time::Duration::from_secs(1));
        }
    });

    let autopilot_version = MavMessage::AUTOPILOT_VERSION(common::AUTOPILOT_VERSION_DATA {
        capabilities: common::MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
            common::MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE |
            //common::MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_C_CAST |
            common::MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_COMMAND_INT,
        uid: 0,
        flight_sw_version: 0,
        middleware_sw_version: 0,
        os_sw_version: 0,
        board_version: 0,
        vendor_id: 0,
        product_id: 0,
        flight_custom_version: [0; 8],
        middleware_custom_version: [0; 8],
        os_custom_version: [0; 8],
    });

    let protocol_version = MavMessage::PROTOCOL_VERSION(common::PROTOCOL_VERSION_DATA {
        version: 200,
        min_version: 200,
        max_version: 200,
        spec_version_hash: [0; 8],
        library_version_hash: [0; 8],
    });

    let rc_input_tx;

    if args.joystick{
        rc_input_tx = Some(get_new_tx_of_message::<RcInputMsg>("rc_input").unwrap());
    }else{
        rc_input_tx = None;
    }

    loop {
        match mavconn.recv() {
            Ok((_header, msg)) => {
                match msg {
                    MavMessage::COMMAND_LONG(ref data) => {
                        let req_message_id = data.param1 as u32;
                        if data.command != common::MavCmd::MAV_CMD_REQUEST_MESSAGE {
                            println!("unsupport cmd: {msg:?}");
                        }
                        let ack = MavMessage::COMMAND_ACK(common::COMMAND_ACK_DATA {
                            command: common::MavCmd::MAV_CMD_REQUEST_MESSAGE,
                            result: common::MavResult::MAV_RESULT_ACCEPTED,
                        });

                        if req_message_id == autopilot_version.message_id() {
                            let _ = mavconn.send(&header, &autopilot_version);
                            let _ = mavconn.send(&header, &ack);
                            println!("send autopilot version back!");
                        } else if req_message_id == protocol_version.message_id() {
                            let _ = mavconn.send(&header, &protocol_version);
                            let _ = mavconn.send(&header, &ack);
                            println!("send protocol version back!");
                        } else {
                            println!("unsupport request for messageid:{req_message_id}");
                        }
                    }
                    MavMessage::PARAM_REQUEST_LIST(_) => {
                        // TODO: check component id and system id
                        let params = param::get_params_map();
                        let param_count = params.len() as u16;
                        let mut param_index = 0;

                        println!("recv param req list!");
                        for i in params {
                            let mut param_id = [0 as u8; 16];
                            param_id[..i.key().len()].copy_from_slice(i.key().as_bytes());
                            let param_msg = MavMessage::PARAM_VALUE(common::PARAM_VALUE_DATA {
                                param_value: i.value().get_data().union_to_f32(),
                                param_count,
                                param_index,
                                param_id,
                                param_type: get_mav_paramtype(&i.value().get_data()),
                            });
                            let _ = mavconn.send(&header, &param_msg);

                            println!("send param!");
                            param_index += 1;
                        }
                    }

                    MavMessage::PARAM_SET(data) => {
                        if let Ok(name) = std::ffi::CStr::from_bytes_until_nul(&data.param_id)
                            .unwrap()
                            .to_str()
                        {
                            if let Some(pdata) =
                                get_paramdata_in_mav(&data.param_type, data.param_value)
                            {
                                if param::set_param(name, pdata).is_ok() {
                                    let param_msg =
                                        MavMessage::PARAM_VALUE(common::PARAM_VALUE_DATA {
                                            param_value: data.param_value,
                                            param_count: param::get_params_map().len() as u16,
                                            param_index: u16::MAX,
                                            param_id: data.param_id,
                                            param_type: data.param_type,
                                        });
                                    let _ = mavconn.send(&header, &param_msg);
                                }
                            }
                        }
                    }

                    MavMessage::MISSION_REQUEST_LIST(data) => {
                        let msg = MavMessage::MISSION_COUNT(common::MISSION_COUNT_DATA {
                            count: 0,
                            target_system: data.target_system,
                            target_component: data.target_component,
                        });
                        let _ = mavconn.send(&header, &msg);
                    }

                    MavMessage::HEARTBEAT(_) => {}
                    MavMessage::MANUAL_CONTROL(data) => {
                        if let Some(ref tx) = rc_input_tx{
                            let mut vals = [0;8];
                            vals[2] = (data.z - 500) * 2;   // map 0-1000 to -1000 to 1000
                            tx.send(RcInputMsg { channel_vals: vals })
                        }
                        //println!("received: {msg:?}");
                    }
                    _ => {
                        println!("received: {msg:?}");
                    }
                }
            }
            Err(MessageReadError::Io(e)) => {
                if e.kind() == std::io::ErrorKind::WouldBlock {
                    //no messages currently available to receive -- wait a while
                    std::thread::sleep(std::time::Duration::from_secs(1));
                    continue;
                } else {
                    println!("recv error: {e:?}");
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
