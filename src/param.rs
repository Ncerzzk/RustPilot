use std::{any::Any, collections::HashMap, str::FromStr, sync::OnceLock};

pub trait ParameterDataType {}

impl ParameterDataType for u32 {}
impl ParameterDataType for i32 {}
impl ParameterDataType for f32 {}

pub struct Parameter {
    name: String,
    data: Option<i32>,
    default: i32,
}

impl Parameter {
    pub fn set<T>(&mut self, val: T)
    where
        T: ParameterDataType,
    {
        let ptr = &mut self.data as *mut Option<i32> as *mut Option<T>;

        unsafe {
            *ptr = Some(val);
        }
    }

    /*
       return the val of this parameter.
       if the parameter val is None, then return the default value.
    */
    pub fn get<T>(&self) -> T
    where
        T: ParameterDataType + Copy,
    {
        if self.data.is_none() {
            let ptr = &self.default as *const i32 as *const T;
            unsafe { *ptr }
        } else {
            let ptr = &self.data as *const Option<i32> as *const Option<T>;
            unsafe { (*ptr).unwrap() }
        }
    }

    pub fn reset(&mut self) {
        self.data = None;
    }
}

pub struct ParameterList {
    data: HashMap<String, Parameter>,
}

impl ParameterList {
    fn new() -> Self {
        ParameterList {
            data: HashMap::new(),
        }
    }

    pub fn add<T>(&mut self, name:&str, val: Option<T>, default: T)
    where
        T: ParameterDataType,
    {
        let mut param = Parameter {
            name: name.to_string(),
            data: None,
            default: 0,
        };
        if val.is_some() {
            param.set(val.unwrap());
        }
        let default_ptr = &mut param.default as *mut i32 as *mut T;
        unsafe {
            *default_ptr = default;
        }
        self.data.insert(name.to_string(), param);
    }

    pub fn get_val<T>(&self, name: &str) -> Option<T>
    where
        T: ParameterDataType + Copy,
    {
        let a = self.data.get(name);

        match a {
            Some(param) => Some(param.get::<T>()),
            None => None,
        }
    }

    pub fn get(&mut self,name:&str) ->Option<&mut Parameter>{
        self.data.get_mut(name)
    }
}

pub static mut PARAMS: OnceLock<ParameterList> = OnceLock::new();

fn param_main(argc:u32,argv: *const &str){

}

#[rpos::ctor::ctor]
fn register() {
    unsafe{let _ = PARAMS.set(ParameterList::new());};
    rpos::module::Module::register("param", param_main);
}


#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_option_ptr() {
        let mut x: Option<i32> = Some(0);
        let ptr = &mut x as *mut Option<i32> as *mut Option<f32>;
        unsafe {
            *ptr = Some(0.1);
            println!("{:?}", *ptr);
        }
    }

    #[test]
    fn test_parameters_add(){
        let mut params = ParameterList::new();
        params.add("gyro_set", None, 0 as i32);

        assert_eq!(params.get_val::<i32>("gyro_set").unwrap(),0);
        assert!(params.get_val::<i32>("undefined").is_none());

        let x = params.get("gyro_set").unwrap();
        x.set(50);

        assert_eq!(x.get::<i32>(),50);
        assert_eq!(params.get_val::<i32>("gyro_set").unwrap(),50);
    }

    #[test]
    fn test_parameters_f32(){
        let mut params = ParameterList::new();
        params.add("f32_test",Some(1.0),0.5);
        let val = params.get_val::<f32>("f32_test").unwrap();
        assert!(val > 0.9999);
        assert!(val < 1.0001);

        params.get("f32_test").unwrap().reset();
        let val = params.get_val::<f32>("f32_test").unwrap();
        assert!(val > 0.49999);
        assert!(val < 0.50001);
    }

    #[test]
    fn test_static_params(){
        let list = unsafe { PARAMS.get_mut().unwrap() };
        list.add("test", None, 0);

        let val = unsafe { PARAMS.get().unwrap().get_val::<i32>("test").unwrap() };
        assert_eq!(val,0);

    }

}
