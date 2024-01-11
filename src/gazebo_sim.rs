use core::slice;
use std::{ os::raw::c_void, sync::Arc, cell::RefCell, time::Duration};
use gz::msgs::{any::Any};
use rpos::hrt::Timespec;
use rpos::workqueue::{WorkQueue, WorkItem};
use rpos::lock_step::lock_step_update_time;
use rpos::module::Module;
use rpos::ctor::ctor;
use rpos::channel::Sender;
use gz::{msgs::world_stats::WorldStatistics,msgs::pose_v::Pose_V,msgs::imu::IMU};
use quaternion_core::{Quaternion,RotationType::Intrinsic,RotationSequence,to_euler_angles};
use crate::message::get_message_list;
use crate::msg_define::*;



struct GazeboSim{
    workitem:Arc<WorkItem>,
    gz_node:RefCell<gz::transport::Node>,
    pose_index:RefCell<i32>,
    gz_sub_info:GzSubInfo,
    gyro_tx:Sender<GyroMsg>,
    acc_tx:Sender<AccMsg>,
    attitude_tx:Sender<AttitudeMsg>
}

#[derive(serde::Deserialize)]
struct GzSubInfo{
    world_name:String,
    pose_obj_name:String
}

impl GazeboSim{
    fn update_time(self:&Arc<Self>,s:WorldStatistics){
        let t = Timespec{
            nsec:s.sim_time.nsec as i64,
            sec:s.sim_time.sec
        };
        lock_step_update_time(t);
    }

    #[inline(always)]
    fn update_pose(self:&Arc<Self>,s:Pose_V){
        let mut pos_idx = self.pose_index.borrow_mut();
        if *pos_idx == -1{
            let index = s.pose.iter().position(|x| x.name == self.gz_sub_info.pose_obj_name).unwrap() as i32;
            *pos_idx = index;
        }
        let pose = &s.pose[*pos_idx as usize];
        let _position = (pose.position.x,pose.position.y,pose.position.z);
        let q:Quaternion<f32> =  (pose.orientation.w as f32, 
            [pose.orientation.x as f32,
            pose.orientation.y as f32, 
            pose.orientation.z as f32 
            ]);
        let angle = to_euler_angles(Intrinsic, RotationSequence::ZYX, q);
    }

    fn update_imu(self:&Arc<Self>,s:IMU){
        let gyro_data = s.angular_velocity;
        let acc_data = s.linear_acceleration;
        let attitude_data = s.orientation;
        self.gyro_tx.send(GyroMsg{x:gyro_data.x as f32,y:gyro_data.y as f32,z:gyro_data.z as f32});
        self.acc_tx.send(AccMsg{x:acc_data.x as f32, y:acc_data.y as f32, z:acc_data.z as f32});
        self.attitude_tx.send(AttitudeMsg { 
            w:attitude_data.w as f32, 
            x:attitude_data.x as f32, 
            y:attitude_data.y as f32, 
            z:attitude_data.z as f32 
        });
    }

    fn new(wq: &Arc<WorkQueue>,toml_filename:&str)->Arc<Self>{
        
        let sub_info:GzSubInfo= toml::from_str(&std::fs::read_to_string(toml_filename).unwrap()).unwrap();
        let msg_list =  get_message_list();

        let sim = Arc::new_cyclic(|weak|{
            let a = GazeboSim{
                workitem: WorkItem::new(wq,"gazebo",weak.as_ptr() as *mut Any,run),
                gz_node: RefCell::new(gz::transport::Node::new().unwrap()),
                pose_index: RefCell::new(-1),
                gz_sub_info:sub_info,
                gyro_tx:msg_list.read().unwrap().get_message("gyro").unwrap().tx.clone(),
                acc_tx:msg_list.read().unwrap().get_message("acc").unwrap().tx.clone(),
                attitude_tx:msg_list.read().unwrap().get_message("attitude").unwrap().tx.clone(),
            };
            a
        });
        assert!(sim.subscribe(&format!("/world/{}/stats",sim.gz_sub_info.world_name) ,Self::update_time));
        assert!(sim.subscribe(&format!("/world/{}/dynamic_pose/info",sim.gz_sub_info.world_name),Self::update_pose));
        assert!(sim.subscribe("/imu",Self::update_imu));
        sim
    }

    fn subscribe<T,F>(self:&Arc<Self>,topic:&str,callback: F) ->bool
    where 
        T:gz_msgs_common::GzMessage, 
        F:Fn(&Arc<Self>,T)+'static
    {
        println!("tp:{}",topic);
        let clone = self.clone();
        self.gz_node.borrow_mut().subscribe(topic, move |x|{
            callback(&clone,x);
        })
    }

    fn run_steps(self:&Arc<Self>,steps:u64){
        let mut req = gz::msgs::world_control::WorldControl::new();
        req.pause = false;
        req.multi_step = steps as u32;

        let topic = format!("/world/{}/control",self.gz_sub_info.world_name);
        self.gz_node.borrow_mut().request::<gz::msgs::world_control::WorldControl,gz::msgs::boolean::Boolean>(&topic,&req,Duration::from_secs(1));
    }
}

fn run(ptr:*mut c_void){

}

pub fn init_gazebo_sim(_argc:u32, _argv:*const &str){
    assert!(_argc == 2);
    let argv = unsafe { slice::from_raw_parts(_argv, _argc as usize) };

    let wq = WorkQueue::new("sim",8192, 98, false);

    let sim = GazeboSim::new(&wq,argv[1]);
    println!("gz inited!");
}

#[ctor]
fn register(){
    Module::register("sim_gz", init_gazebo_sim);
}