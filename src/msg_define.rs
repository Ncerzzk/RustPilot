#![allow(dead_code)]
use std::ops::Index;

use rpos::msg::add_message;


// Gyro/Acc message data, unit:rad/s
#[derive(Debug,Clone,Copy,Default)]
pub struct Vector3{
    pub x:f32,
    pub y:f32,
    pub z:f32
}

#[derive(Debug,Clone,Copy)]
pub struct Vector4{
    pub w:f32,
    pub x:f32,
    pub y:f32,
    pub z:f32
}

#[derive(Debug,Clone,Copy)]
pub struct AttitudeSetPointMsg{
    pub attitude:Vector4, // quaternion 
    pub body_thrusts:Vector3 // [-1,1]
}

// Attitude target euler
#[derive(Debug,Clone,Copy)]
pub struct EulerVector3{
    pub pitch:f32,
    pub roll:f32,
    pub yaw:f32
}

impl Index<usize> for EulerVector3{
    type Output = f32;
    fn index(&self, index:usize) -> &Self::Output {
        match index {
            0=>&self.pitch,
            1=>&self.roll,
            2=>&self.yaw,
            _=>panic!("over index!")
        }
    }
}

impl Index<usize> for Vector3{
    type Output = f32;
    fn index(&self, index:usize) -> &Self::Output {
        match index {
            0=>&self.x,
            1=>&self.y,
            2=>&self.z,
            _=>panic!("over index!")
        }
    }
}

#[derive(Debug,Clone,Copy)]
pub struct TorqueThrustMsg{
    pub torques:EulerVector3,
    pub thrusts:Vector3
}

// will used in rate controller
pub struct RateSetPointMsg{
    pub angle_rate:EulerVector3,
    pub thrusts:Vector3
}

// Manual Control Input
#[allow(dead_code)]
#[derive(Debug,Clone)]
pub struct ManualControlMsg{
    pub pitch:u32,
    pub roll:u32,
    pub thrust:u32,
    pub direction:u32
}

#[derive(Debug,Clone)]
pub struct RcInputMsg{
    pub channel_vals:[i16;8]   // -1000~1000
}

#[allow(dead_code)]
impl ManualControlMsg{
    pub const MAX:u32 = 10000;
}

#[derive(Debug,Clone)]
pub struct MixerOutputMsg{
    pub control_group_id:u8,
    pub output:[f32;8],
}


#[rpos::ctor::ctor]
fn register_msgs(){
    add_message::<Vector3>("gyro");
    add_message::<Vector3>("acc");
    add_message::<Vector4>("attitude");
    //add_message::<EulerVector3>("att_target_euler");
    add_message::<AttitudeSetPointMsg>("att_target");
    add_message::<TorqueThrustMsg>("toreque_thrust_setpoint");
    //add_message::<ControllerOutputGroupMsg>("controller_output0");
    //add_message::<ControllerOutputGroupMsg>("controller_output1");
    add_message:: <MixerOutputMsg>("mixer_output");
    add_message::<ManualControlMsg>("manual_control");
    add_message::<RcInputMsg>("rc_input");
}

