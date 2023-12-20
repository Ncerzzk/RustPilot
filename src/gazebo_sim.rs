use std::{ptr::null, os::raw::c_void, sync::Arc, borrow::BorrowMut, cell::RefCell, time::Duration};

use gz::msgs::{any::Any, request::Request, world_control::WorldControl};
use rpos::workqueue::{WorkQueue, WorkItem};
use gz::{msgs::world_stats::WorldStatistics};


struct GazeboSim{
    workitem:Arc<WorkItem>,
    gz_node:RefCell<gz::transport::Node>
}

fn test(s:WorldStatistics){
    println!("{}",s.iterations);
}

impl GazeboSim{
    fn new(wq: &Arc<WorkQueue>)->Arc<Self>{
        let sim = Arc::new_cyclic(|weak|{
            let a = GazeboSim{
                workitem: WorkItem::new(wq,"gazebo",weak.as_ptr() as *mut Any,run),
                gz_node: RefCell::new(gz::transport::Node::new().unwrap())
            };
            a
        });

        assert!(sim.gz_node.borrow_mut().subscribe("/world/default/stats",test) );
        sim
    }

    fn run_steps(self:&Arc<Self>,steps:u64){
        let mut req = gz::msgs::world_control::WorldControl::new();
        req.pause = true;
        req.multi_step = steps as u32;

        self.gz_node.borrow_mut().request::<gz::msgs::world_control::WorldControl,gz::msgs::boolean::Boolean>("/world/default/control",&req,Duration::from_secs(1));
    }
}

fn run(ptr:*mut c_void){

}

pub fn init_gazebo_sim(){
    let wq = WorkQueue::new("sim",8192, 98, false);

    let sim = GazeboSim::new(&wq);
    println!("sub!");
    sim.run_steps(100);
    gz::transport::wait_for_shutdown();
}
