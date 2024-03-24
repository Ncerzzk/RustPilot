use quaternion_core::Quaternion;
use rpos::msg::add_message;


// Gyro message data, unit:rad/s
#[derive(Debug,Clone,Copy,Default)]
pub struct GyroMsg{
    pub x:f32,
    pub y:f32,
    pub z:f32
}

// Acc message data, unit:m/(s^2)
#[derive(Debug,Clone,Copy,Default)]
pub struct AccMsg{
    pub x:f32,
    pub y:f32,
    pub z:f32
}

// Attitude message data
#[derive(Debug,Clone,Copy)]
pub struct AttitudeMsg{
    pub w:f32,
    pub x:f32,
    pub y:f32,
    pub z:f32
}

// Attitude target euler
#[derive(Debug,Clone,Copy)]
pub struct AttitudeTargetEulerMsg{
    pub pitch:f32,
    pub roll:f32,
    pub yaw:f32
}

// Controller Output
#[derive(Debug,Clone,Copy)]
pub struct ControllerOutputGroupMsg{
    pub output:[f32;8],
}

// Manual Control Input
#[derive(Debug,Clone)]
pub struct ManualControlMsg{
    pub pitch:u32,
    pub roll:u32,
    pub thrust:u32,
    pub direction:u32
}

#[derive(Debug,Clone)]
pub struct RcInputMsg{
    pub channel_vals:[i16;8]
}

impl ManualControlMsg{
    pub const MAX:u32 = 10000;
}


#[rpos::ctor::ctor]
fn register_msgs(){
    add_message::<GyroMsg>("gyro");
    add_message::<AccMsg>("acc");
    add_message::<AttitudeMsg>("attitude");
    add_message::<AttitudeTargetEulerMsg>("att_target_euler");
    add_message::<ControllerOutputGroupMsg>("controller_output0");
    add_message::<ControllerOutputGroupMsg>("controller_output1");

    add_message::<ManualControlMsg>("manual_control");
    add_message::<RcInputMsg>("rc_input");
}

