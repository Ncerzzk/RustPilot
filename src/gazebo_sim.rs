use std::{ os::raw::c_void, sync::Arc, cell::RefCell, time::Duration};

use gz::msgs::{any::Any};
use rpos::hrt::Timespec;
use rpos::workqueue::{WorkQueue, WorkItem};
use rpos::lock_step::lock_step_update_time;
use rpos::module::Module;
use rpos::ctor::ctor;
use gz::{msgs::world_stats::WorldStatistics};


struct GazeboSim{
    workitem:Arc<WorkItem>,
    gz_node:RefCell<gz::transport::Node>
}

impl GazeboSim{
    fn update_time(s:WorldStatistics){
        let t = Timespec{
            nsec:s.sim_time.nsec as i64,
            sec:s.sim_time.sec
        };
        lock_step_update_time(t);
    }

    fn new(wq: &Arc<WorkQueue>)->Arc<Self>{
        let sim = Arc::new_cyclic(|weak|{
            let a = GazeboSim{
                workitem: WorkItem::new(wq,"gazebo",weak.as_ptr() as *mut Any,run),
                gz_node: RefCell::new(gz::transport::Node::new().unwrap())
            };
            a
        });

        assert!(sim.gz_node.borrow_mut().subscribe("/world/default/stats",Self::update_time) );
        sim
    }

    fn run_steps(self:&Arc<Self>,steps:u64){
        let mut req = gz::msgs::world_control::WorldControl::new();
        req.pause = false;
        req.multi_step = steps as u32;

        self.gz_node.borrow_mut().request::<gz::msgs::world_control::WorldControl,gz::msgs::boolean::Boolean>("/world/default/control",&req,Duration::from_secs(1));
    }
}

fn run(ptr:*mut c_void){

}

pub fn init_gazebo_sim(_argc:u32, _argv:*const &str){
    let wq = WorkQueue::new("sim",8192, 98, false);

    let sim = GazeboSim::new(&wq);
    println!("gz inited!");
    gz::transport::wait_for_shutdown();
}

#[ctor]
fn register(){
    Module::register("sim_gz", init_gazebo_sim);
}