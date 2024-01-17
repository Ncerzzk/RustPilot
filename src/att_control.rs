use rpos::{channel::Sender, pthread_scheduler::SchedulePthread};
use std::{
    os::raw::c_void,
    ptr::{null, null_mut},
    sync::Arc,
};

use crate::{
    message::get_message_list,
    msg_define::{AttitudeMsg, AttitudeTargetEulerMsg, ControllerOutputGroupMsg},
    pid::PIDController,
};

struct AttitudeController {
    pitch_controller: PIDController,
    roll_controller: PIDController,
    tx: Sender<ControllerOutputGroupMsg>,
}

fn att_control_main(ptr: *mut c_void) -> *mut c_void {
    let sp = unsafe { Arc::from_raw(ptr as *const SchedulePthread) };
    let msg = get_message_list().read().unwrap();

    let att_target_msg = msg.get_message::<AttitudeTargetEulerMsg>("att_target_euler");
    let mut att_target_rx = att_target_msg.unwrap().rx.clone();
    let mut att_rx: rpos::channel::Receiver<AttitudeMsg> =
        msg.get_message("attitude").unwrap().rx.clone();

    let mut att_ctrler = AttitudeController {
        pitch_controller: PIDController::new(100.0, 0.0, 0.0),
        roll_controller: PIDController::new(100.0, 0.0, 0.0),
        tx: msg
            .get_message::<ControllerOutputGroupMsg>("controller_output0")
            .unwrap()
            .tx
            .clone(),
    };

    let mut att_target = AttitudeTargetEulerMsg {
        roll: 0.0,
        pitch: 0.0,
        yaw: 0.0,
    };
    let mut att_now = [0.0; 3];
    loop {
        let att_target_recv = att_target_rx.try_read();
        match att_target_recv {
            Some(x) => att_target = x,
            None => {}
        };

        let att_now_q = att_rx.try_read();
        match att_now_q {
            Some(attmsg) => {
                let q: quaternion_core::Quaternion<f32> =
                    (attmsg.w, [attmsg.x, attmsg.y, attmsg.z]);
                att_now = quaternion_core::to_euler_angles(
                    quaternion_core::RotationType::Extrinsic,
                    quaternion_core::RotationSequence::XYZ,
                    q,
                );
                println!("{:?}", att_now);
            }
            None => {}
        }

        let pitch = att_now[1];
        let roll = att_now[0];
        let yaw = att_now[2];

        let pitch_out = att_ctrler
            .pitch_controller
            .calcuate(att_target.pitch, pitch, 0.0025);
        
        let roll_out = att_ctrler.roll_controller.calcuate(att_target.roll, roll, 0.0025);
        let mut output_all = [0.0;8];
        output_all[0] = pitch_out;
        output_all[1] = roll_out;
        att_ctrler.tx.send(ControllerOutputGroupMsg{ output:output_all});

        sp.schedule_until(2500);
    }
    null_mut()
}

pub fn init_att_control(_argc: u32, _argv: *const &str) {
    SchedulePthread::new(2048, 98, att_control_main, null_mut(), false, None); // TODO edit pthread_key
}

#[rpos::ctor::ctor]
fn register() {
    rpos::module::Module::register("att_control", init_att_control);
}
