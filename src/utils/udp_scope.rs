use std::net::UdpSocket;

pub struct UdpScope{
    socket:UdpSocket,
    addr:String
}

impl UdpScope{
    pub fn new()->UdpScope{
        let socket = UdpSocket::bind("0.0.0.0:0").unwrap();

        UdpScope{
            socket,
            addr:"127.0.0.1:12345".to_string()
        }
    }

    pub fn send_wave(&self,floats:&[f32]){
        let s =     unsafe {
            std::slice::from_raw_parts(
                floats.as_ptr() as *const u8,
                floats.len() * std::mem::size_of::<f32>()
            )
        };
        self.socket.send_to(s, &self.addr).unwrap();
    }
}



