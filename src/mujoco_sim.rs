use crate::msg_define::*;
use crate::basic::rotation::Rotation;
use core::slice;
use rpos::channel::Sender;
use rpos::ctor::ctor;
use rpos::module::Module;
use rpos::msg::{get_new_tx_of_message, get_new_rx_of_message};
use std::sync::Arc;
use std::io::Write;
use std::thread;
use std::ptr;

use mujoco_rust::model::ObjType;
use crate::mujoco_display;
use crate::mujoco_lidar;

unsafe impl Send for MujocoSim {}
unsafe impl Sync for MujocoSim {}

enum SimEvent {
    MixerUpdate(Vec<f64>),
    TimerTick,
    Quit,
}

struct MujocoSim {
    model: mujoco_rust::Model,
    simulation: mujoco_rust::Simulation,
    gyro_tx: Sender<Vector3>,
    acc_tx: Sender<Vector3>,
    attitude_tx: Sender<Vector4>,
    rf_ids: Vec<u16>,
    angles: Vec<u16>,
}

impl MujocoSim{
    pub fn update_mj_sensor(&self) {
        // Update the Mujoco sensor data
        let mj_model = unsafe { *self.model.ptr() };
        // 0: gyro
        let gyro_start = unsafe { *mj_model.sensor_adr.add(0) } as usize;
        let gyro_dim = unsafe { *mj_model.sensor_dim.add(0) } as usize;
        let gyro_data = &self.simulation.sensordata()[gyro_start..gyro_start + gyro_dim];

        // 1: accelerometer
        let acc_start = unsafe { *mj_model.sensor_adr.add(1) } as usize;
        let acc_dim = unsafe { *mj_model.sensor_dim.add(1) } as usize;
        let acc_data = &self.simulation.sensordata()[acc_start..acc_start + acc_dim];

        // 2: framequat (attitude quaternion)
        let att_start = unsafe { *mj_model.sensor_adr.add(2) } as usize;
        let att_dim = unsafe { *mj_model.sensor_dim.add(2) } as usize;
        let att_data = &self.simulation.sensordata()[att_start..att_start + att_dim];


        let rotation = Rotation::Yaw270;
        self.gyro_tx.send(rotation.rotate_v(Vector3 {
            x: gyro_data[0] as f32,
            y: gyro_data[1] as f32,
            z: gyro_data[2] as f32,
        }));
        self.acc_tx.send(rotation.rotate_v(Vector3 {
            x: acc_data[0] as f32,
            y: acc_data[1] as f32,
            z: acc_data[2] as f32,
        }));
        let imu_q: quaternion_core::Quaternion<f32> = (
            att_data[0] as f32,
            [
                att_data[1] as f32,
                att_data[2] as f32,
                att_data[3] as f32,
            ],
        );
        let imu_q = rotation.rotate_q(imu_q);

        self.attitude_tx.send(Vector4 {
            w: imu_q.0,
            x: imu_q.1[0],
            y: imu_q.1[1],
            z: imu_q.1[2],
        });

    }

    fn mujoco_sim_event_loop(sim: Arc<MujocoSim>, actuator_num: usize) -> Result<(), Box<dyn std::error::Error>> {
        let mut mixer_rx = get_new_rx_of_message::<MixerOutputMsg>("mixer_output").unwrap();
        
        let mut ctrl: Vec<f64> = vec![0.0; actuator_num as usize];

        // init lidar
        let lidar_width = 800;
        let lidar_height = 800;
        let mut buffer: Vec<u32> = vec![0; lidar_width * lidar_height];
        let mut lidar_window = minifb::Window::new(
            "LiDAR Scan",
            lidar_width,
            lidar_height,
            minifb::WindowOptions {
                borderless: true,
                ..minifb::WindowOptions::default()
            },
        )?;

        let front_cam = sim.simulation.model.name_to_id(ObjType::CAMERA, "front_cam").unwrap() as i32;
        let down_cam = sim.simulation.model.name_to_id(ObjType::CAMERA, "down_cam").unwrap() as i32;
        let rear_cam = sim.simulation.model.name_to_id(ObjType::CAMERA, "rear_cam").unwrap() as i32;
        let right_cam = sim.simulation.model.name_to_id(ObjType::CAMERA, "right_cam").unwrap() as i32;

        let body_id = sim.simulation.model.name_to_id(ObjType::BODY, "x2").unwrap() as i32;

        // UI with 3rd-person view
        let mut ui_state_3rd = mujoco_display::glfw_init(&sim.simulation, &[0x7FFFFFFF]);

        // vedio streaming with 1st-person view
        let mut ffmpeg = mujoco_display::init_ffmpeg();
        let mut stdin = ffmpeg.stdin.take().unwrap();
        let mut ui_state_1st = mujoco_display::glfw_init(&sim.simulation, &[down_cam, front_cam, right_cam, rear_cam]);

        sim.simulation.control(&ctrl);
        loop {
            if let Some(mixer) = mixer_rx.try_read() {
                for (i, val) in mixer.output.iter().enumerate() {
                    if i < ctrl.len() {
                        ctrl[i] = *val as f64;
                    }
                }
            }
            let ctrl_f64: Vec<f64> = ctrl.iter().map(|&x| x as f64).collect();
    
            // update Lidar window
            mujoco_lidar::update_lidar_buffer(lidar_width, lidar_height, &mut buffer, &sim.rf_ids, &sim.angles, &sim.simulation);
            lidar_window.update_with_buffer(&buffer, lidar_width, lidar_height)?;

            sim.simulation.control(&ctrl);

            sim.simulation.step();

            let pos = sim.simulation.qpos(); 
            sim.update_mj_sensor();

            let frame = mujoco_display::update_mjscene(&sim.simulation, &mut ui_state_1st);
            let _ = stdin.write_all(&frame);

            // ui render update
            mujoco_display::glfw_update_scene(&sim.simulation, &mut ui_state_3rd);

            // time step setting
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
        mujoco_display::free_resource(&mut ui_state_1st);
        mujoco_display::free_resource(&mut ui_state_3rd);

        // close stdin
        drop(stdin);

        // wait for FFmpeg end
        match ffmpeg.wait_with_output() {
            Ok(output) => {
                if !output.status.success() {
                    eprintln!("FFmpeg error: {}", String::from_utf8_lossy(&output.stderr));
                    return Ok(());
                }
                Ok(())
            }
            Err(e) => {
                eprintln!("Error: wait for FFmpeg failed: {}", e);
                return Ok(());
            }
        }
    }


    fn new(xml_filename: &str) -> Arc<Self> {
        let model = mujoco_rust::Model::from_xml(xml_filename).unwrap();
        let simulation = mujoco_rust::Simulation::new(model.clone());
        
        // Lidar
        let angles = vec![0, 15, 30, 45, 60, 75, 90, 105, 120, 135, 150, 165, 180, 195, 210, 225, 240, 255, 270, 285, 300, 315, 330, 345];
        let mut rf_ids = Vec::new();
        for angle in angles.iter() {
            let sensor_name = format!("rf_{}", angle);
            let id = model.name_to_id(ObjType::SITE, &sensor_name).unwrap();
            rf_ids.push(id as u16);
        }

        let sim = Arc::new_cyclic(|_| {
            let tmp = MujocoSim {
                model: model.clone(),
                simulation: simulation,
                gyro_tx: get_new_tx_of_message("gyro").unwrap(),
                acc_tx: get_new_tx_of_message("acc").unwrap(),
                attitude_tx: get_new_tx_of_message("attitude").unwrap(),
                rf_ids: rf_ids,
                angles: angles
            };
            tmp
        });
        sim
    }
    
}

pub fn init_mujoco_sim(_argc: u32, _argv: *const &str){
    assert!(_argc == 2);
    let argv = unsafe { slice::from_raw_parts(_argv, _argc as usize) };
    let sim = MujocoSim::new(argv[1]);
    let actuator_num = unsafe { (*sim.simulation.model.ptr()).nu as usize};
    
    println!("MujocoSim inited!");

    std::thread::spawn(move || {
        if let Err(e) = MujocoSim::mujoco_sim_event_loop(sim, actuator_num) {
            eprintln!("Simulated thread error: {:?}", e);
        }
    });
}


#[ctor]
fn register() {
    Module::register("mujoco_sim", init_mujoco_sim);
}