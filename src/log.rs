use std::{os::unix::thread, ptr::null, io::Write};

use rpos::libc;

use crate::ThreadSpecificData;

#[macro_export]
macro_rules! thread_logln {
    ($($arg:tt)*) => {
        write!(crate::log::get_output(),"{}\n", format!($($arg)*)).unwrap()
    }
}

#[macro_export]
macro_rules! thread_log {
    ($($arg:tt)*) => {
        write!(get_output(),"{}", format!($($arg)*)).unwrap()
    }
}

pub fn get_output()->Box<dyn Write>{
    let thread_data = unsafe { libc::pthread_getspecific(*crate::PTHREAD_KEY) };
    if thread_data == std::ptr::null_mut(){
        Box::new(std::io::stdout()) as Box<dyn Write>
    }else{
        let stream:ThreadSpecificData = unsafe{*(thread_data as *mut ThreadSpecificData)};
        unsafe{
            Box::new((*stream.stream).try_clone().unwrap()) as Box<dyn Write>
        }
    }
}

#[cfg(test)]
mod tests{
    use super::*;

    #[test]
    fn test_thread_log(){
        thread_logln!("hello,wolrd!{}",5555);
    }
}
