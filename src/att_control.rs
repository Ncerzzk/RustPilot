use rpos::{
    channel::Sender,
    msg::{get_new_rx_of_message, get_new_tx_of_message},
    pthread_scheduler::SchedulePthread,
};
use std::{os::raw::c_void, ptr::null_mut, sync::Arc};

use crate::{
    basic::pid::PIDController,
    msg_define::{Vector4, TorqueThrustMsg, EulerVector3, Vector3, AttitudeSetPointMsg},
    param,
};

use quaternion_core::{frame_rotation, point_rotation, Quaternion as Q};

struct AttitudeController {
    pitch_controller: PIDController,
    roll_controller: PIDController,
    tx: Sender<TorqueThrustMsg>,
}

fn get_attitude_distance(target: Q<f32>, now: Q<f32>) -> [f32; 3] {
    let now_z = point_rotation(now, [0.0, 0.0, 1.0]);
    let target_z = point_rotation(target, [0.0, 0.0, 1.0]);

    // 获取机体坐标系的z轴在世界坐标系的坐标（向量）
    // 获取期望的集体坐标系z轴在世界坐标系的坐标（向量）
    let theta = quaternion_core::dot(now_z, target_z).acos();
    if theta.abs() < 0.00001 {
        return [0.0; 3];
    }

    let axis = quaternion_core::cross(now_z, target_z);
    // 通过叉乘，获取一个轴和角度，z轴可以绕这个 轴 旋转到达期望z轴

    let axis_new = frame_rotation(now, axis);
    // 获取这个轴在机体坐标系的坐标

    let axis_q = quaternion_core::from_axis_angle(axis_new, theta);
    // 通过这个轴和角度，构造一个机体坐标系的旋转四元数

    quaternion_core::normalize(axis_q).1
}

fn att_control_main(ptr: *mut c_void) -> *mut c_void {
    let sp: Arc<SchedulePthread> = unsafe { Arc::from_raw(ptr as *const SchedulePthread) };

    let mut att_target_rx = get_new_rx_of_message::<AttitudeSetPointMsg>("att_target").unwrap();
    let mut att_rx = get_new_rx_of_message::<Vector4>("attitude").unwrap();

    let mut att_ctrler = AttitudeController {
        pitch_controller: PIDController::new(100.0, 0.0, 0.0),
        roll_controller: PIDController::new(100.0, 0.0, 0.0),
        tx: get_new_tx_of_message("toreque_thrust_setpoint").unwrap(),
    };

    let mut att_target_q: Q<f32> = (1.0, [0.0, 0.0, 0.0]);
    let mut att_q: Q<f32> = (1.0, [0.0, 0.0, 0.0]);

    let mut thrust_z:f32 =0.0;

    loop {
        if let Some(set_point) = att_target_rx.try_read() {
            att_target_q = (set_point.attitude.w, [set_point.attitude.x, set_point.attitude.y, set_point.attitude.z]);

            thrust_z = set_point.body_thrusts.z;
            // let att_target = [x.pitch, x.roll, x.yaw];
            // att_target_q = quaternion_core::from_euler_angles(
            //     quaternion_core::RotationType::Extrinsic,
            //     quaternion_core::RotationSequence::XYZ,
            //     att_target,
            // );
        }

        if let Some(attmsg) = att_rx.try_read() {
            att_q = (attmsg.w, [attmsg.x, attmsg.y, attmsg.z]);
        }

        let q_err = get_attitude_distance(att_target_q, att_q);

        let pitch_out = att_ctrler.pitch_controller.calcuate(q_err[0], 0.0025);
        let roll_out = att_ctrler.roll_controller.calcuate(q_err[1], 0.0025);

        let mut output_all = [0.0; 8];
        output_all[0] = pitch_out;
        output_all[1] = roll_out;
        att_ctrler.tx.send(TorqueThrustMsg {
            torques: EulerVector3 {
                pitch: pitch_out,
                roll: roll_out,
                yaw: 0.0,
            },
            thrusts: Vector3{
                x: 0.0,
                y: 0.0,
                z: thrust_z,
            },
        });

        sp.schedule_until(2500);
    }
    #[allow(unreachable_code)]
    null_mut()
}

pub fn init_att_control(_argc: u32, _argv: *const &str) {
    param::add_param("att_Kp", param::ParameterData::Float(0.0));
    param::add_param("att_Ki", param::ParameterData::Float(0.0));
    param::add_param("att_Kd", param::ParameterData::Float(0.0));
    SchedulePthread::new(16384, 98, att_control_main, null_mut(), false); // TODO edit pthread_key
}

#[rpos::ctor::ctor]
fn register() {
    rpos::module::Module::register("att_control", init_att_control);
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_q_difference() {
        let target_q: quaternion_core::Quaternion<f32> = (1.0, [0.0, 0.0, 0.0]);

        let now_q: quaternion_core::Quaternion<f32> =
            quaternion_core::from_axis_angle([1.0, 0.0, 0.0], 1.57);
        let err = get_attitude_distance(target_q, now_q);
        println!("err:{:?}", err);
    }
}
