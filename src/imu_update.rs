use std::{ffi::c_void, ptr::null_mut, sync::Arc};

use rpos::{
    channel::Receiver, msg::get_new_rx_of_message, pthread_scheduler::SchedulePthread
};

use crate::basic::rotation::get_euler_degree;
use crate::utils::udp_scope::UdpScope;

use quaternion_core::{normalize, Quaternion as Q};

use crate::msg_define::{Vector4, Vector3};

use rpos::libc::c_long;

struct IMUUpdate {
    q: Q<f32>,
    imu_update_ki: f32,
    imu_update_kp: f32,
    err_i:[f32;3]
    // scope:UdpScope // for debug
}

impl IMUUpdate {
    fn update(&mut self, acc: [f32; 3], mut gyro_data: [f32; 3], dt: f32) {
        let imu_update_half_t = dt / 2.0;

        let acc_normed = quaternion_core::normalize(acc);

        let acc_rotate = quaternion_core::frame_rotation(self.q, [0.0, 0.0, 1.0]);
        let err = quaternion_core::cross(acc_normed, acc_rotate); // use the product of cross as the err

        //self.scope.send_wave(&[gyro_data[0],gyro_data[1],gyro_data[2],gyro_data[2]]);
        for (index, err_item) in err.iter().enumerate() {
            if err_item.is_normal() {
                self.err_i[index] += (*err_item) * dt * self.imu_update_ki;

                gyro_data[index] +=  (*err_item) * self.imu_update_kp + self.err_i[index];
            }
        }

        let q = &mut self.q;

        q.0 += (-q.1[0] * gyro_data[0] - q.1[1] * gyro_data[1] - q.1[2] * gyro_data[2])
            * imu_update_half_t;
        q.1[0] += (q.0 * gyro_data[0] + q.1[1] * gyro_data[2] - q.1[2] * gyro_data[1])
            * imu_update_half_t;
        q.1[1] += (q.0 * gyro_data[1] - q.1[0] * gyro_data[2] + q.1[2] * gyro_data[0])
            * imu_update_half_t;
        q.1[2] += (q.0 * gyro_data[2] + q.1[0] * gyro_data[1] - q.1[1] * gyro_data[0])
            * imu_update_half_t;

        *q = normalize(*q);
    }
}

fn imu_update_main(ptr: *mut c_void) -> *mut c_void {
    let sp = unsafe { Arc::from_raw(ptr as *const SchedulePthread) };
    let mut gyro_rx = get_new_rx_of_message::<Vector3>("gyro").unwrap();
    let mut acc_rx = get_new_rx_of_message::<Vector3>("acc").unwrap();
    let mut imu_update = IMUUpdate {
        q: (1.0, [0.0; 3]),
        imu_update_ki: 0.2,
        imu_update_kp: 2.0,
        err_i:[0.0;3]
    };

    // let q_tx: Sender<Vector4> = get_new_tx_of_message("attitude").unwrap();
    let mut q_rx_debug: Receiver<Vector4> = get_new_rx_of_message("attitude").unwrap();

    const IMU_UPDATE_T: f32 = 0.010;
    const IMU_UPDATE_T_US: c_long = (IMU_UPDATE_T * 1000.0 * 1000.0) as c_long;

    let mut acc_data: [f32; 3] = [0.0; 3];
    let mut gyro_data: [f32; 3] = [0.0; 3];

    loop {
        if let Some(acc_msg) = acc_rx.try_read() {
            acc_data = [acc_msg.x, acc_msg.y, acc_msg.z];
        }
        if let Some(gyro_msg) = gyro_rx.try_read() {
            gyro_data = [gyro_msg.x, gyro_msg.y, gyro_msg.z];
        }
        imu_update.update(acc_data, gyro_data, IMU_UPDATE_T);
        let mut x = Vector4 {
            w: 0.0,
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        if let Some(msg) = q_rx_debug.try_read() {
            x = msg;
        }

        let gazebo_q = (x.w,[x.x,x.y,x.z]);
        // let euler_cal = get_euler_degree(imu_update.q);
        // let euler_gz = get_euler_degree(gazebo_q);

        //imu_update.scope.send_wave(&[euler_cal[0],euler_cal[1],euler_gz[0],euler_gz[1]]);

        sp.schedule_until(IMU_UPDATE_T_US);
    }
    #[allow(unreachable_code)]
    null_mut()
}

pub fn init_imu_update(_argc: u32, _argv: *const &str) {
    SchedulePthread::new(1024 * 1024, 97, imu_update_main, null_mut(), false);
}

#[rpos::ctor::ctor]
fn register() {
    rpos::module::Module::register("imu_update", init_imu_update);
}

#[cfg(test)]
mod tests {
    use std::f32::consts::PI;

    use super::IMUUpdate;
    use quaternion_core::RotationSequence::*;

    #[test]
    fn test_imu_update() {
        let mut imu_update = IMUUpdate {
            q: (1.0, [0.0, 0.0, 0.0]),
            imu_update_ki: 0.0,
            imu_update_kp: 0.0,
            err_i: [0.0;3],
        };

        let target_rad = 90.0 / 180.0 * PI;
        let angle_speed = target_rad / 1.0;
        for _ in 0..500 {
            imu_update.update([0.0, 0.0, 1.0], [angle_speed, 0.0, 0.0], 1.0 / 500.0);
            println!("{:?}", imu_update.q);
            println!(
                "angle:{:?}",
                quaternion_core::to_euler_angles(
                    quaternion_core::RotationType::Intrinsic,
                    YXZ,
                    imu_update.q
                )
            );
        }
    }
}
