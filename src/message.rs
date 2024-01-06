use std::{sync::{LazyLock, RwLock}, collections::HashMap, any::Any};
use crossbeam::channel::{Receiver, unbounded, Sender};

pub struct Message<T>{
    pub rx:Receiver<T>,
    pub tx:Sender<T>
}

struct MessageList{
    data:HashMap<&'static str,Box<dyn Any>>
}

impl MessageList{
    pub fn add_message<T:'static>(&mut self, name:&'static str){
        let (tx,rx) = unbounded::<T>();
        let msg= Message{
            rx,
            tx
        };
        self.data.insert(name, Box::new(msg));
    }

    pub fn get_message<T:'static>(&self,name:&str)->Option<&Message<T>>{
        let a = self.data.get(name).unwrap();
        let a = a.downcast_ref::<Message<T>>();
        a
    }
}

unsafe impl Send for MessageList{}
unsafe impl Sync for MessageList{}

static MESSAGE_LIST:LazyLock<RwLock<MessageList>> = LazyLock::new(||{
    RwLock::new(MessageList { data:HashMap::new() })
});

#[cfg(test)]
mod tests{
    use super::*;

    struct GyroData{
        data:[i32;3]
    }

    #[rpos::ctor::ctor]
    fn ttt(){
        let mut msg_list = MESSAGE_LIST.write().unwrap();
        msg_list.add_message::<GyroData>("gyro");        
    }

    #[test]
    fn test_basic(){
        let mut msg_list = MESSAGE_LIST.write().unwrap();
        //msg_list.add_message::<GyroData>("gyro");
        let msg = msg_list.get_message::<GyroData>("gyro").unwrap();
        let rx = msg.rx.clone();
        let tx = msg.tx.clone();

        std::thread::spawn(move ||{
            tx.send(GyroData{ data: [1,2,3] });
        });

        let recv_data = rx.recv().unwrap().data;

        assert_eq!(recv_data[0],1);
        assert_eq!(recv_data[1],2);

        println!("data:{:?}",recv_data);

    }
}

