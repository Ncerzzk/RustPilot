use clap::Parser;
use rpos::msg::{get_new_rx_of_message, get_new_tx_of_message};

use crate::{
    msg_define::RcInputMsg,
    msg_define::{AttitudeSetPointMsg, EulerVector3, TorqueThrustMsg, Vector3, Vector4},
};

#[derive(Parser, Clone)]
#[command(
    name = "manual_ctrl",
    about = "recv rc input, do channel mapping(if need) and output to mixer or attitude_controller"
)]
struct ManualCtrl {
    #[arg(short, long, help = "directly send output to mixer")]
    directly_out: bool,
}

pub fn init_manual_ctrl(argc: u32, argv: *const &str) {
    if let Some(args) = crate::basic::client_process_args::<ManualCtrl>(argc, argv) {
        let rx = get_new_rx_of_message::<RcInputMsg>("rc_input").unwrap();
        if args.directly_out {
            let ctrl_msg_tx =
                get_new_tx_of_message::<TorqueThrustMsg>("toreque_thrust_setpoint").unwrap();
            rx.register_callback("manual_ctrl_rx", move |rc_msg| {
                let arr = rc_msg
                    .channel_vals
                    .map(|x| (x as f32).clamp(-1000.0, 1000.0));
                ctrl_msg_tx.send(TorqueThrustMsg {
                    torques: EulerVector3 {
                        pitch: arr[1],
                        roll: arr[0],
                        yaw: arr[3],
                    },
                    thrusts: Vector3 {
                        x: 0.0,
                        y: 0.0,
                        z: arr[2],
                    },
                });
            });
        } else {
            let att_target_tx = get_new_tx_of_message::<AttitudeSetPointMsg>("att_target").unwrap();

            rx.register_callback("manual_ctrl_rx", move |rc_msg| {
                att_target_tx.send(AttitudeSetPointMsg {
                    attitude: Vector4{
                        w: 1.0,
                        x: 0.0,
                        y: 0.0,
                        z: 0.0,
                    },
                    body_thrusts: Vector3 {
                        x: 0.0,
                        y: 0.0,
                        z: (rc_msg.channel_vals[2] + 1000) as f32 / 2000.0,
                    }, // maping -1000~1000 to 0~1 }
                });
            });
        }
    }
}

#[rpos::ctor::ctor]
fn register() {
    rpos::module::Module::register("manual_ctrl", init_manual_ctrl);
}
