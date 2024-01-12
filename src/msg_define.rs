use quaternion_core::Quaternion;


// Gyro message data, unit:rad/s
#[derive(Debug,Clone,Copy)]
pub struct GyroMsg{
    pub x:f32,
    pub y:f32,
    pub z:f32
}

// Acc message data, unit:m/(s^2)
#[derive(Debug,Clone,Copy)]
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

#[rpos::ctor::ctor]
fn register_msgs(){
    let msg_list = crate::message::get_message_list();
    let mut msg_list = msg_list.write().unwrap();
    msg_list.add_message::<GyroMsg>("gyro");
    msg_list.add_message::<AccMsg>("acc");
    msg_list.add_message::<AttitudeMsg>("attitude");
    msg_list.add_message::<AttitudeTargetEulerMsg>("att_target_euler");
}