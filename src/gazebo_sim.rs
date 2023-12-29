use core::slice;
use std::fmt::format;
use std::mem::MaybeUninit;
use std::{ os::raw::c_void, sync::Arc, cell::RefCell, time::Duration};

use gz::msgs::gz_msgs10;
use gz::msgs::{any::Any};
use rpos::hrt::Timespec;
use rpos::workqueue::{WorkQueue, WorkItem};
use rpos::lock_step::lock_step_update_time;
use rpos::module::Module;
use rpos::ctor::ctor;
use gz::{msgs::world_stats::WorldStatistics,msgs::pose_v::Pose_V};
use quaternion_core::{Quaternion,RotationType::Intrinsic,RotationSequence,to_euler_angles};



struct GazeboSim{
    workitem:Arc<WorkItem>,
    gz_node:RefCell<gz::transport::Node>,
    pose_index:RefCell<i32>,
    gz_sub_info:GzSubInfo
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

    fn new(wq: &Arc<WorkQueue>,toml_filename:&str)->Arc<Self>{
        
        let sub_info:GzSubInfo= toml::from_str(&std::fs::read_to_string(toml_filename).unwrap()).unwrap();
        let sim = Arc::new_cyclic(|weak|{
            let a = GazeboSim{
                workitem: WorkItem::new(wq,"gazebo",weak.as_ptr() as *mut Any,run),
                gz_node: RefCell::new(gz::transport::Node::new().unwrap()),
                pose_index: RefCell::new(-1),
                gz_sub_info:sub_info
            };
            a
        });
        assert!(sim.subscribe(&format!("/world/{}/stats",sim.gz_sub_info.world_name) ,Self::update_time));
        assert!(sim.subscribe(&format!("/world/{}/dynamic_pose/info",sim.gz_sub_info.world_name),Self::update_pose));
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
    sim.workitem.schedule();
}

#[ctor]
fn register(){
    Module::register("sim_gz", init_gazebo_sim);
}