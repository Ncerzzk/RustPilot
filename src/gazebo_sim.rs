use crate::msg_define::*;
use crate::basic::rotation::Rotation;
use core::slice;
use gz::msgs::imu::IMU;
use rpos::channel::Sender;
use rpos::ctor::ctor;
use rpos::hrt::Timespec;
use rpos::lock_step::lock_step_update_time;
use rpos::module::Module;
use rpos::msg::get_new_tx_of_message;
use std::{cell::RefCell, sync::Arc, time::Duration};

struct GazeboSim {
    gz_node: RefCell<gz::transport::Node>,
    #[allow(unused)]
    pose_index: RefCell<i32>,
    gz_sub_info: GzSubInfo,
    gyro_tx: Sender<Vector3>,
    acc_tx: Sender<Vector3>,
    attitude_tx: Sender<Vector4>,
}

#[derive(serde::Deserialize)]
struct GzSubInfo {
    world_name: String,

    #[allow(unused)]
    pose_obj_name: String,
}

impl GazeboSim {
    fn update_time(self: &Arc<Self>, s: gz::msgs::clock::Clock) {
        let t = Timespec {
            nsec: s.sim.nsec as i64,
            sec: s.sim.sec,
        };
        lock_step_update_time(t);
    }

    fn update_imu(self: &Arc<Self>, s: IMU) {
        let gyro_data = s.angular_velocity;
        let acc_data = s.linear_acceleration;
        let attitude_data = s.orientation;
        let rotation = Rotation::Yaw270;
        self.gyro_tx.send(rotation.rotate_v(Vector3 {
            x: gyro_data.x as f32,
            y: gyro_data.y as f32,
            z: gyro_data.z as f32,
        }));
        self.acc_tx.send(rotation.rotate_v(Vector3 {
            x: acc_data.x as f32,
            y: acc_data.y as f32,
            z: acc_data.z as f32,
        }));
        let imu_q: quaternion_core::Quaternion<f32> = (
            attitude_data.w as f32,
            [
                attitude_data.x as f32,
                attitude_data.y as f32,
                attitude_data.z as f32,
            ],
        );

        let imu_q = rotation.rotate_q(imu_q);
        /*
        imu_q is the rotate quaternion from gazebo axis to body axis 
        rotate_q is the rotate quaternion from gazebo axis to world axis(our defination): x -> -y_old ,  y -> x_old. 
        */

        self.attitude_tx.send(Vector4 {
            w: imu_q.0,
            x: imu_q.1[0],
            y: imu_q.1[1],
            z: imu_q.1[2],
        });
    }

    fn new(toml_filename: &str) -> Arc<Self> {
        let sub_info: GzSubInfo =
            toml::from_str(&std::fs::read_to_string(toml_filename).unwrap()).unwrap();

        let sim = Arc::new_cyclic(|_| {
            let a = GazeboSim {
                gz_node: RefCell::new(gz::transport::Node::new().unwrap()),
                pose_index: RefCell::new(-1),
                gz_sub_info: sub_info,
                gyro_tx: get_new_tx_of_message("gyro").unwrap(),
                acc_tx: get_new_tx_of_message("acc").unwrap(),
                attitude_tx: get_new_tx_of_message("attitude").unwrap(),
            };
            a
        });
        assert!(sim.subscribe("/clock", Self::update_time));
        assert!(sim.subscribe("/imu", Self::update_imu));
        sim
    }

    fn subscribe<T, F>(self: &Arc<Self>, topic: &str, callback: F) -> bool
    where
        T: gz_msgs_common::GzMessage,
        F: Fn(&Arc<Self>, T) + 'static,
    {
        println!("tp:{}", topic);
        let clone = self.clone();
        self.gz_node.borrow_mut().subscribe(topic, move |x| {
            callback(&clone, x);
        })
    }

    #[allow(dead_code)]
    fn run_steps(self: &Arc<Self>, steps: u64) {
        let mut req = gz::msgs::world_control::WorldControl::new();
        req.pause = false;
        req.multi_step = steps as u32;

        let topic = format!("/world/{}/control", self.gz_sub_info.world_name);
        self.gz_node
            .borrow_mut()
            .request::<gz::msgs::world_control::WorldControl, gz::msgs::boolean::Boolean>(
                &topic,
                &req,
                Duration::from_secs(1),
            );
    }
}

pub fn init_gazebo_sim(_argc: u32, _argv: *const &str) {
    assert!(_argc == 2);
    let argv = unsafe { slice::from_raw_parts(_argv, _argc as usize) };
    let _ = GazeboSim::new(argv[1]);
    println!("gz inited!");
}

#[ctor]
fn register() {
    Module::register("gazebo_sim", init_gazebo_sim);
}
