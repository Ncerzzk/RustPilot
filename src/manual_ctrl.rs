use clap::Parser;
use rpos::{
    msg::{get_new_rx_of_message, get_new_tx_of_message},
};

use crate::{
    elrs::client_process_args,
    msg_define::{ControllerOutputGroupMsg, RcInputMsg},
};

#[derive(Parser, Clone)]
#[command(name = "manual_ctrl",about = "recv rc input, do channel mapping(if need) and output to mixer or attitude_controller")]
struct ManualCtrl {
    #[arg(short, long, help = "directly send output to mixer")]
    directly_out: bool,
}

pub fn init_manual_ctrl(argc: u32, argv: *const &str) {
    if let Some(args) = client_process_args::<ManualCtrl>(argc, argv) {
        let rx = get_new_rx_of_message::<RcInputMsg>("rc_input").unwrap();

        let ctrl_msg_tx =
            get_new_tx_of_message::<ControllerOutputGroupMsg>("controller_output0").unwrap();
        rx.register_callback("manual_ctrl_rx", move |rc_msg| {
            if args.directly_out {
                let arr = rc_msg
                    .channel_vals
                    .map(|x| (x as f32).clamp(-1000.0, 1000.0));
                ctrl_msg_tx.send(ControllerOutputGroupMsg { output: arr });
            }
        });
    }
}

#[rpos::ctor::ctor]
fn register() {
    rpos::module::Module::register("manual_ctrl", init_manual_ctrl);
}
