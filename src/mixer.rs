use std::{ops::Neg, sync::OnceLock};

use rpos::channel::Sender;

use crate::{
    message::get_message_list,
    msg_define::{ControllerOutputGroupMsg, MixerOutputMsg},
};

/*
    for implement now:
    2 controller output group
    each controller output group has 8 channel.
    for basic use, we can only use the gropu0 channel 0~3(pitch,roll,yaw)

    flow:
    controller -->   mixer --> actuator
*/

struct Mixer {
    controller_outputs: Vec<ControllerOutputGroupMsg>,
    mixers: Vec<SumMixer>,
    tx: Sender<MixerOutputMsg>,
}

impl Mixer {
    #[inline(always)]
    fn update_ctrl_outputs(&mut self, ctrl_group_id: u8, msg: &ControllerOutputGroupMsg) {
        self.controller_outputs[ctrl_group_id as usize] = *msg;
        let mut publish = Vec::<(u8, f32)>::new();

        for i in &self.mixers {
            if i.bind_ctrl_group_id == ctrl_group_id {
                publish.push((i.output_channel_idx, i.calcuate(&self.controller_outputs)));
            }
        }

        self.tx.send(MixerOutputMsg {
            output: Box::new(publish),
        });
    }

    fn init_x_quadcopter_mixers(&mut self) {
        /*
               4     1
                  x
               2     3
        */
        let motor_1 = SumMixer {
            list: vec![
                MixerChannel {
                    scaler: Scaler::default(),
                    ctrl_group_id: 0,
                    ctrl_channel_idx: ControllerOutputChannel::Pitch as u8,
                },
                MixerChannel {
                    scaler: -Scaler::default(),
                    ctrl_group_id: 0,
                    ctrl_channel_idx: ControllerOutputChannel::Roll as u8,
                },
            ],
            scaler: Scaler {
                scale_p: 0.5,
                scale_n: 0.5,
                offset: 50.0,
                min: 0.0,
                max: 100.0,
            },
            bind_ctrl_group_id: 0,
            output_channel_idx: 0,
        };

        let motor_2 = SumMixer {
            list: vec![
                MixerChannel {
                    scaler: -Scaler::default(),
                    ctrl_group_id: 0,
                    ctrl_channel_idx: ControllerOutputChannel::Pitch as u8,
                },
                MixerChannel {
                    scaler: Scaler::default(),
                    ctrl_group_id: 0,
                    ctrl_channel_idx: ControllerOutputChannel::Roll as u8,
                },
            ],
            scaler: Scaler {
                scale_p: 0.5,
                scale_n: 0.5,
                offset: 50.0,
                min: 0.0,
                max: 100.0,
            },
            bind_ctrl_group_id: 0,
            output_channel_idx: 1,
        };

        let motor_3 = SumMixer {
            list: vec![
                MixerChannel {
                    scaler: -Scaler::default(),
                    ctrl_group_id: 0,
                    ctrl_channel_idx: ControllerOutputChannel::Pitch as u8,
                },
                MixerChannel {
                    scaler: -Scaler::default(),
                    ctrl_group_id: 0,
                    ctrl_channel_idx: ControllerOutputChannel::Roll as u8,
                },
            ],
            scaler: Scaler {
                scale_p: 0.5,
                scale_n: 0.5,
                offset: 50.0,
                min: 0.0,
                max: 100.0,
            },
            bind_ctrl_group_id: 0,
            output_channel_idx: 2,
        };

        let motor_4 = SumMixer {
            list: vec![
                MixerChannel {
                    scaler: Scaler::default(),
                    ctrl_group_id: 0,
                    ctrl_channel_idx: ControllerOutputChannel::Pitch as u8,
                },
                MixerChannel {
                    scaler: Scaler::default(),
                    ctrl_group_id: 0,
                    ctrl_channel_idx: ControllerOutputChannel::Roll as u8,
                },
            ],
            scaler: Scaler {
                scale_p: 0.5,
                scale_n: 0.5,
                offset: 50.0,
                min: 0.0,
                max: 100.0,
            },
            bind_ctrl_group_id: 0,
            output_channel_idx: 3,
        };

        self.mixers.push(motor_1);
        self.mixers.push(motor_2);
        self.mixers.push(motor_3);
        self.mixers.push(motor_4);
    }
}

enum ControllerOutputChannel {
    Pitch = 0x0,
    Roll,
    Yaw,
}

struct Scaler {
    scale_p: f32,
    scale_n: f32,
    offset: f32,
    min: f32,
    max: f32,
}

impl Default for Scaler {
    fn default() -> Self {
        Self {
            scale_p: 1.0,
            scale_n: 1.0,
            offset: 0.0,
            min: -100.0,
            max: 100.0,
        }
    }
}

impl Neg for Scaler {
    type Output = Self;

    fn neg(self) -> Self::Output {
        Self {
            scale_p: -self.scale_p,
            scale_n: -self.scale_n,
            offset: self.offset,
            min: self.min,
            max: self.max,
        }
    }
}

impl Scaler {
    fn scale(&self, input: f32) -> f32 {
        let scale_k;
        if input > 0.0 {
            scale_k = self.scale_p;
        } else {
            scale_k = self.scale_n;
        }
        let ret = scale_k * input + self.offset;
        ret.clamp(self.min, self.max)
    }
}

struct MixerChannel {
    scaler: Scaler,
    ctrl_group_id: u8,
    ctrl_channel_idx: u8,
}

struct SumMixer {
    list: Vec<MixerChannel>,
    scaler: Scaler,
    bind_ctrl_group_id: u8,
    output_channel_idx: u8,
}

impl SumMixer {
    fn calcuate(&self, ctrl_output_groups: &Vec<ControllerOutputGroupMsg>) -> f32 {
        let ret: f32 = self.list.iter().fold(0.0, |acc, i| {
            let group = ctrl_output_groups.get(i.ctrl_group_id as usize).unwrap();
            let input = group.output.get(i.ctrl_channel_idx as usize).unwrap();
            acc + i.scaler.scale(*input)
        });
        self.scaler.scale(ret)
    }
}

static mut MIXER: OnceLock<Mixer> = OnceLock::new();
const CONTROLLER_OUTPUT_COUNT: u8 = 2;

pub unsafe fn init_mixer(_argc: u32, _argv: *const &str) {
    let msg_list = get_message_list().read().unwrap();
    let msg = msg_list.get_message("mixer_output").unwrap();
    let _ = MIXER.get_or_init(|| {
        let mut ret = Mixer {
            controller_outputs: Vec::new(),
            mixers: Vec::new(),
            tx: msg.tx.clone(),
        };

        let msg_list = get_message_list().read().unwrap();
        for i in 0..CONTROLLER_OUTPUT_COUNT {
            let msg = msg_list
                .get_message::<ControllerOutputGroupMsg>(format!("controller_output{}", i).as_str())
                .unwrap();
            ret.controller_outputs
                .push(ControllerOutputGroupMsg { output: [0.0; 8] });
            msg.rx.register_callback(
                format!("mixer_listener{}", i).as_str(),
                move |x: &ControllerOutputGroupMsg| {
                    MIXER.get_mut().unwrap().update_ctrl_outputs(i, x);
                },
            );
        }
        ret
    });
}

#[rpos::ctor::ctor]
fn register() {
    rpos::module::Module::register("mixer", |a, b| unsafe { init_mixer(a, b) });
}

#[cfg(test)]
mod tests {
    use std::ptr::null_mut;

    use crate::mixer;

    use super::*;

    fn get_fake_controller_output_tx(group_id: u8) -> Sender<ControllerOutputGroupMsg> {
        let msg = get_message_list().read().unwrap();
        let tx = msg
            .get_message::<ControllerOutputGroupMsg>(
                format!("controller_output{}", group_id).as_str(),
            )
            .unwrap()
            .tx
            .clone();
        tx
    }
    #[test]
    fn test_init_mixer() {
        let tx = get_fake_controller_output_tx(0);
        unsafe {
            init_mixer(0, null_mut());
            assert_eq!(MIXER.get().unwrap().controller_outputs.len(), 2);

            tx.send(ControllerOutputGroupMsg { output: [1.0; 8] });
            for i in 0..8 {
                assert!(MIXER.get().unwrap().controller_outputs[0].output[i as usize] > 0.0);
                assert!(MIXER.get().unwrap().controller_outputs[0].output[i as usize] < 2.0);
            }
        }
    }

    #[test]
    fn test_x_quadcoptermixer_calcute() {
        let ctrl_tx = get_fake_controller_output_tx(0);
        let msg_list = get_message_list().read().unwrap();
        let mut mixer_rx = msg_list
            .get_message::<MixerOutputMsg>("mixer_output")
            .unwrap()
            .rx
            .clone();
        unsafe {
            init_mixer(0, null_mut());
            MIXER.get_mut().unwrap().init_x_quadcopter_mixers();
        }

        ctrl_tx.send(ControllerOutputGroupMsg { output: [50.0; 8] });
        println!("{:?}", mixer_rx.read())
    }
}
