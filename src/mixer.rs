#![allow(dead_code)]
use crate::basic::scaler::Scaler;
use rpos::{
    channel::Sender,
    msg::{get_new_rx_of_message, get_new_tx_of_message},
};
use serde::{Deserialize, Serialize};
use std::{io::Read, path::Path };

use crate::msg_define::{TorqueThrustMsg, MixerOutputMsg};

// Mixer Output

/*
    for implement now:
    2 controller output group
    each controller output group has 8 channel.
    for basic use, we can only use the gropu0 channel 0~3(pitch,roll,yaw)

    flow:
    controller -->   mixer --> actuator

    the output of controller should be -1000 ~ 1000( should  multiply a coefficient)
    [pitch_out, roll_out, thrust_out, direction(yaw)_out, undefined...]

*/
#[derive(Serialize, Deserialize)]
struct Mixer {
    #[serde(skip)]
    controller_outputs: Vec<TorqueThrustMsg>,
    mixers: Vec<SumMixer>,
    #[serde(skip)]
    tx: Sender<MixerOutputMsg>,
}

impl Mixer {
    #[inline(always)]
    fn update_ctrl_outputs(&self, msg: &TorqueThrustMsg) {
        let mut publish: [f32; 8] = [0.0; 8];
        for i in &self.mixers {
            if i.bind_ctrl_group_id == 0 {
                // TODO: remove this
                publish[i.output_channel_idx as usize] = i.calcuate(&msg)
            }
        }
        self.tx.send(MixerOutputMsg {
            output: publish,
            control_group_id: 0,
        });
    }

    fn read_mixers_info_from_file<P>(&mut self, filepath: P) -> Result<(), ()>
    where
        P: AsRef<Path>,
    {
        let mut s = String::new();
        if let Ok(mut file) = std::fs::File::open(filepath) {
            file.read_to_string(&mut s).unwrap();
            if let Ok(temp) = serde_json::from_str::<Mixer>(&s) {
                self.mixers = temp.mixers;
            } else {
                return Err(());
            }
        } else {
            println!("no mixer file found!");
            return Err(());
        }

        Ok(())
    }

    fn init_x_quadcopter_mixers(&mut self) {
        /*
            gazebo x3 quadcopter motor index
               2     0
                  x
               1     3
        */
        let motor_0 = SumMixer {
            list: vec![
                MixerChannel {
                    scaler: Scaler::default(),
                    ctrl_group_id: 0,
                    ctrl_channel: ControlChannel::Pitch,
                },
                MixerChannel {
                    scaler: -Scaler::default(),
                    ctrl_group_id: 0,
                    ctrl_channel: ControlChannel::Roll,
                },
            ],
            bind_ctrl_group_id: 0,
            output_channel_idx: 0,
            mode: OutputMode::Speed,
        };

        let motor_1 = SumMixer {
            list: vec![
                MixerChannel {
                    scaler: -Scaler::default(),
                    ctrl_group_id: 0,
                    ctrl_channel: ControlChannel::Pitch,
                },
                MixerChannel {
                    scaler: Scaler::default(),
                    ctrl_group_id: 0,
                    ctrl_channel: ControlChannel::Roll,
                },
            ],
            bind_ctrl_group_id: 0,
            output_channel_idx: 1,
            mode: OutputMode::Speed,
        };

        let motor_2 = SumMixer {
            list: vec![
                MixerChannel {
                    scaler: Scaler::default(),
                    ctrl_group_id: 0,
                    ctrl_channel: ControlChannel::Pitch,
                },
                MixerChannel {
                    scaler: Scaler::default(),
                    ctrl_group_id: 0,
                    ctrl_channel: ControlChannel::Roll,
                },
            ],
            bind_ctrl_group_id: 0,
            output_channel_idx: 2,
            mode: OutputMode::Speed,
        };

        let motor_3 = SumMixer {
            list: vec![
                MixerChannel {
                    scaler: -Scaler::default(),
                    ctrl_group_id: 0,
                    ctrl_channel: ControlChannel::Pitch,
                },
                MixerChannel {
                    scaler: -Scaler::default(),
                    ctrl_group_id: 0,
                    ctrl_channel: ControlChannel::Roll,
                },
            ],
            bind_ctrl_group_id: 0,
            output_channel_idx: 3,
            mode: OutputMode::Speed,
        };

        self.mixers.push(motor_0);
        self.mixers.push(motor_1);
        self.mixers.push(motor_2);
        self.mixers.push(motor_3);
    }
}

#[derive(Debug, serde::Serialize, serde::Deserialize)]
struct MixerChannel {
    scaler: Scaler,
    ctrl_group_id: u8,
    ctrl_channel: ControlChannel,
}

#[derive(Debug, serde::Serialize, serde::Deserialize)]
struct SumMixer {
    list: Vec<MixerChannel>,
    bind_ctrl_group_id: u8,
    output_channel_idx: u8,
    #[serde(default)]
    mode: OutputMode,
}

#[derive(Debug, serde::Serialize, serde::Deserialize, Clone,Copy)]
enum ControlChannel{
    Pitch=0,
    Roll,
    Yaw,
    ThrustX,
    ThrustY,
    ThrustZ, 
}

#[derive(Debug, Default, serde::Serialize, serde::Deserialize, Clone)]
pub enum OutputMode {
    #[default]
    Duty, // 0 ~ 1.0
    PluseWidth, // 0 ~ 2000 us (actually we can get a larger value, but no significance)
    DShot,      // 0 ~ 1.0
    Speed,      // only used in simulation now
}

impl SumMixer {
    fn calcuate(&self, ctrl_out: &TorqueThrustMsg) -> f32 {
        let mixer: f32 = self.list.iter().fold(0.0, |acc, i| {
            let input;
            if (i.ctrl_channel as u8) < 3 {
                input = ctrl_out.torques[i.ctrl_channel as usize];
            } else if (i.ctrl_channel as u8) < 6 {
                input = ctrl_out.thrusts[i.ctrl_channel as usize - 3];
            } else {
                panic!("error index!");
            }
            acc + i.scaler.scale(input)
        });
        mixer
    }
}

pub unsafe fn init_mixer(argc: u32, argv: *const &str) {
    let mut mixer = Mixer {
        controller_outputs: Vec::new(),
        mixers: Vec::new(),
        tx: get_new_tx_of_message("mixer_output").unwrap(),
    };

    if argc == 2 {
        let path = std::slice::from_raw_parts(argv, argc as usize);
        println!("read mixer from {}.", path[1]);
        mixer.read_mixers_info_from_file(path[1]).unwrap();
    } else if argc == 1 {
        println!("use default x quadcopter mixer!");
        mixer.init_x_quadcopter_mixers();
    } else {
        panic!("error arg num of mixer!");
    }

    let rx = get_new_rx_of_message::<TorqueThrustMsg>("toreque_thrust_setpoint").unwrap();
    rx.register_callback("mixer_listner", move |x: &TorqueThrustMsg| {
        mixer.update_ctrl_outputs(x);
    });
}

#[rpos::ctor::ctor]
fn register() {
    rpos::module::Module::register("mixer", |a, b| unsafe { init_mixer(a, b) });
}

#[cfg(test)]
mod tests {
    use std::{default, ptr::null_mut};

    use crate::{mixer, msg_define::{EulerVector3, Vector3}};

    use super::*;

    #[test]
    fn test_init_mixer() {
        let tx = get_new_tx_of_message::<TorqueThrustMsg>("toreque_thrust_setpoint").unwrap();
        let mut rx =get_new_rx_of_message::<MixerOutputMsg>("mixer_output").unwrap();
        unsafe {
            init_mixer(1, null_mut());
            assert!(rx.try_read().is_none());
            tx.send(TorqueThrustMsg {
                torques: EulerVector3{
                    pitch: 1.0,
                    roll: 0.0,
                    yaw: 0.0,
                },
                thrusts: Vector3{
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
            });
            rx.try_read().unwrap();
        }
    }

    // #[test]
    // fn test_mixer2toml() {
    //     unsafe {
    //         init_mixer(1, null_mut());
    //     }
    //     println!(
    //         "{}",
    //         toml::to_string_pretty(unsafe { &(MIXER.get().unwrap()) }).unwrap()
    //     );
    // }

    // #[test]
    // fn test_toml2mixer() {
    //     unsafe {
    //         init_mixer(1, null_mut());
    //         let origin = format!("{:?}", MIXER.get().unwrap().mixers);
    //         MIXER
    //             .get_mut()
    //             .unwrap()
    //             .read_mixers_info_from_file("mixers/gz_mixer.toml")
    //             .unwrap();
    //         let new = format!("{:?}", MIXER.get().unwrap().mixers);
    //         assert_eq!(origin, new);
    //     }
    // }
}
