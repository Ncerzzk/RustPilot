use std::sync::OnceLock;

use gz::msgs::actuators::Actuators;
use gz::transport::Publisher;
use rpos::channel::Receiver;

use crate::{message::get_message_list, msg_define::MixerOutputMsg};

struct GazeboActuator {
    gz_node: gz::transport::Node,
    publisher: Publisher<Actuators>,
}

impl GazeboActuator {
    #[inline(always)]
    fn mixer_output_callback(&mut self, msg: &MixerOutputMsg) {
        let _ = self.publisher.publish(&Actuators {
            velocity: [1.0].to_vec(),
            ..Default::default()
        });
    }
}

unsafe impl Sync for GazeboActuator {}
unsafe impl Send for GazeboActuator {}

static mut GAZEBO_ACTUATOR: OnceLock<GazeboActuator> = OnceLock::new();

pub fn init_gz_actuator(argc: u32, argv: *const &str) {
    let mut node = gz::transport::Node::new().unwrap();

    let publisher: Publisher<Actuators> = node.advertise("/X3/gazebo/command/motor_speed").unwrap();

    let gz_ac = GazeboActuator {
        gz_node: node,
        publisher,
    };

    let _ = unsafe { GAZEBO_ACTUATOR.set(gz_ac) };

    let msg_list = get_message_list().read().unwrap();
    let rx: Receiver<MixerOutputMsg> = msg_list.get_message("mixer_output").unwrap().rx.clone();

    rx.register_callback("gz_actuator", |s| {
        GazeboActuator::mixer_output_callback(unsafe { GAZEBO_ACTUATOR.get_mut().unwrap() }, s)
    });

    std::thread::sleep(std::time::Duration::from_secs(1)); // wait sometime for gz node connection
    println!("hello, gz actuator!");
}

#[rpos::ctor::ctor]
fn register() {
    rpos::module::Module::register("gazebo_actuator", init_gz_actuator);
}

#[cfg(test)]
mod tests {
    use rpos::channel::Sender;
    use std::ptr::null_mut;

    use super::*;

    #[test]
    fn test_init_gz_actuator() {
        let msg_list = get_message_list().read().unwrap();
        let tx: Sender<MixerOutputMsg> = msg_list.get_message("mixer_output").unwrap().tx.clone();

        init_gz_actuator(0, null_mut());

        let subscriber = unsafe {
            GAZEBO_ACTUATOR.get_mut().unwrap().gz_node.subscribe(
                "/X3/gazebo/command/motor_speed",
                |x: Actuators| {
                    println!("{:?}", x);
                },
            )
        };

        tx.send(MixerOutputMsg{ output:Box::new(Vec::new())});
    }
}