use gz::msgs::actuators::Actuators;
use gz::transport::Publisher;
use rpos::{channel::Receiver, msg::get_new_rx_of_message};
use crate::basic::scaler::Scaler;
use crate::msg_define::MixerOutputMsg;

struct GazeboActuator {
    #[allow(unused)]
    gz_node: gz::transport::Node,
    publisher: Publisher<Actuators>,
    scaler: Scaler,
}

impl GazeboActuator {
    #[inline(always)]
    fn mixer_output_callback(&mut self, msg: &MixerOutputMsg) {
        if msg.control_group_id != 0 {
            return ;
        }
        let mut v: Vec<f64> = Vec::new();

        for i in msg.output {
            v.push(self.scaler.scale(i + 0.0) as f64);
        }
        let _ = self.publisher.publish(&Actuators {
            velocity: v,
            ..Default::default()
        });
    }
}

unsafe impl Sync for GazeboActuator {}
unsafe impl Send for GazeboActuator {}

pub fn init_gz_actuator(_argc: u32, _argv: *const &str) {
    let mut node = gz::transport::Node::new().unwrap();

    let publisher: Publisher<Actuators> = node.advertise("/X3/gazebo/command/motor_speed").unwrap();

    let gz_ac = Box::new(GazeboActuator {
        gz_node: node,
        publisher,
        scaler: Scaler {
            scale_p: 1.00,
            scale_n: 1.0,
            offset: 0.0,
            min: 0.0,
            max: 1000.0,
        },
    });

    let gz_ac = Box::leak(gz_ac);

    let rx: Receiver<MixerOutputMsg> = get_new_rx_of_message("mixer_output").unwrap();
    rx.register_callback("gz_actuator", |s| {
        GazeboActuator::mixer_output_callback(gz_ac, s)
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
    use rpos::{channel::Sender, msg::get_new_tx_of_message};
    use std::ptr::null_mut;

    use super::*;

    #[test]
    fn test_init_gz_actuator() {
        // let tx: Sender<MixerOutputMsg> = get_new_tx_of_message("mixer_output").unwrap();

        // init_gz_actuator(0, null_mut());

        // let subscriber = unsafe {
        //     GAZEBO_ACTUATOR.get_mut().unwrap().gz_node.subscribe(
        //         "/X3/gazebo/command/motor_speed",
        //         |x: Actuators| {
        //             println!("{:?}", x);
        //         },
        //     )
        // };

        // tx.send(MixerOutputMsg {
        //     output:[0.0;8],
        //     control_group_id: 0,
        // });
    }
}
