
// Gyro message data, unit:rad/s
pub struct GyroMsg{
    pub x:f32,
    pub y:f32,
    pub z:f32
}

// Acc message data, unit:m/(s^2)
pub struct AccMsg{
    pub x:f32,
    pub y:f32,
    pub z:f32
}

#[rpos::ctor::ctor]
fn register_msgs(){
    let msg_list = crate::message::get_message_list();
    let mut msg_list = msg_list.write().unwrap();
    msg_list.add_message::<GyroMsg>("gyro");
    msg_list.add_message::<AccMsg>("acc");  
}