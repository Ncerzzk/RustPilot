use std::{
    ffi::c_void,
    ptr::null_mut,
    sync::Arc,
};

use rpos::{
    channel::{Receiver, Sender},
    pthread_scheduler::SchedulePthread,
};

use quaternion_core::Quaternion as Q;

use crate::{
    message::get_message_list,
    msg_define::{AccMsg, AttitudeMsg, GyroMsg},
};

struct IMUUpdate {
    gyro_rx: Receiver<GyroMsg>,
    acc_rx: Receiver<AccMsg>,
}

fn update_time(s: gz::msgs::imu::IMU) {}

fn imu_update_main(ptr: *mut c_void) -> *mut c_void {
    let sp = unsafe { Arc::from_raw(ptr as *const SchedulePthread) };
    let msg_list = get_message_list().read().unwrap();
    let mut wrapper = IMUUpdate {
        gyro_rx: msg_list.get_message("gyro").unwrap().rx.clone(),
        acc_rx: msg_list.get_message("acc").unwrap().rx.clone(),
    };

    let q_tx: Sender<AttitudeMsg> = msg_list.get_message("attitude").unwrap().tx.clone();
    let mut q_rx_debug: Receiver<AttitudeMsg> = msg_list.get_message("attitude").unwrap().rx.clone();

    const IMU_UPDATE_T: f32 = 0.002;
    const IMU_UPDATE_HALF_T: f32 = IMU_UPDATE_T / 2.0;
    const IMU_UPDATE_T_US: i64 = (IMU_UPDATE_T * 1000.0 * 1000.0) as i64;
    const IMU_P: f32 = 2.0;
    const IMU_I: f32 = 0.01;

    let mut acc_data: [f32; 3] = [0.0; 3];
    let mut gyro_data: [f32; 3] = [0.0; 3];
    let mut q: Q<f32> = (1.0, [0.0, 0.0, 0.0]);

    let mut err_i: [f32; 3] = [0.0; 3];
    loop {
        if let Some(acc_msg) = wrapper.acc_rx.try_read() {
            acc_data = [acc_msg.x, acc_msg.y, acc_msg.z];
        }
        if let Some(gyro_msg) = wrapper.gyro_rx.try_read() {
            gyro_data = [gyro_msg.x, gyro_msg.y, gyro_msg.z];
        }

        let acc_normed = quaternion_core::normalize(acc_data);

        let acc_rotate = quaternion_core::frame_rotation(q, [0.0, 0.0, 1.0]);

        let mut err = quaternion_core::cross(acc_normed, acc_rotate); // use the product of cross as the err
        for (index, err_item) in err.iter_mut().enumerate() {
            if err_item.is_normal() {
                err_i[index] += (*err_item) * IMU_UPDATE_T * IMU_I;
                *err_item = (*err_item) * IMU_P + err_i[index];

                gyro_data[index] += *err_item;
            }
        }

        q.0 += (-q.1[0] * gyro_data[0] - q.1[1] * gyro_data[1] - q.1[2] * gyro_data[2])
            * IMU_UPDATE_HALF_T;
        q.1[0] += (q.0 * gyro_data[0] + q.1[1] * gyro_data[2] - q.1[2] * gyro_data[1])
            * IMU_UPDATE_HALF_T;
        q.1[1] += (q.0 * gyro_data[1] - q.1[0] * gyro_data[2] + q.1[2] * gyro_data[0])
            * IMU_UPDATE_HALF_T;
        q.1[2] += (q.0 * gyro_data[2] + q.1[0] * gyro_data[1] - q.1[1] * gyro_data[0])
            * IMU_UPDATE_HALF_T;

        let mut x = AttitudeMsg{w:0.0,x:0.0,y:0.0,z:0.0};
        if let Some(msg) = q_rx_debug.try_read(){
            x = msg;
        }
        //let x_q = quaternion_core::mul(q,quaternion_core::from_axis_angle([0.0, 0.0, 1.0], -3.14159 / 2.0));
        println!("cal:{:?} , gazebo:{:?}",q,x);
        // q_tx.send(AttitudeMsg {
        //     w: q.0,
        //     x: q.1[0],
        //     y: q.1[1],
        //     z: q.1[2],
        // });
        sp.schedule_until(IMU_UPDATE_T_US);
    }
    null_mut()
}

pub fn init_imu_update(_argc: u32, _argv: *const &str) {
    SchedulePthread::new(4096, 97, imu_update_main, null_mut(), false, None); // TODO edit pthread_key
}

#[rpos::ctor::ctor]
fn register() {
    rpos::module::Module::register("imu_update", init_imu_update);
}
