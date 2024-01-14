use std::{os::raw::c_void, ptr::{null, null_mut}, sync::Arc};
use rpos::pthread_scheduler::SchedulePthread;

use crate::{message::get_message_list, msg_define::{AttitudeTargetEulerMsg, AttitudeMsg}};


fn att_control_main(ptr:*mut c_void) -> *mut c_void{
    let sp = unsafe{Arc::from_raw(ptr as *const SchedulePthread)};
    let msg = get_message_list().read().unwrap();

    let att_target_msg = msg.get_message::<AttitudeTargetEulerMsg>("att_target_euler");
    let mut att_target_rx = att_target_msg.unwrap().rx.clone();
    let mut att_rx:rpos::channel::Receiver<AttitudeMsg> = msg.get_message("attitude").unwrap().rx.clone();

    let mut att_target = AttitudeTargetEulerMsg{roll:0.0,pitch:0.0,yaw:0.0};
    loop{
        let att_target_recv = att_target_rx.try_read();
        match att_target_recv {
            Some(x) => att_target = x,
            None => {}
        };

        let target_q = quaternion_core::from_euler_angles(
            quaternion_core::RotationType::Extrinsic, 
            quaternion_core::RotationSequence::XYZ, 
            [att_target.pitch,att_target.roll,att_target.yaw]
        );


        sp.schedule_until(2500);
    }
    null_mut()
}

pub fn init_att_control(_argc:u32, _argv:*const &str){
    SchedulePthread::new(2048,98,att_control_main,null_mut(),false,None);  // TODO edit pthread_key 

}

#[rpos::ctor::ctor]
fn register(){
    rpos::module::Module::register("att_control", init_att_control);
}
