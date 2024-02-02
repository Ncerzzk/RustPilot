use core::panic;
use std::{collections::HashMap, str::FromStr, sync::OnceLock};

use clap::{Args, Command};

use crate::thread_logln;

#[derive(Debug)]
enum ParameterData {
    Int(i32),
    Float(f32),
}

impl ParameterData {
    fn get_i32(&self) -> i32 {
        if let Self::Int(x) = self {
            x.clone()
        } else {
            panic!("this param is not i32!")
        }
    }

    fn get_f32(&self) -> f32 {
        if let Self::Float(x) = self {
            x.clone()
        } else {
            panic!("this param is not f32!")
        }
    }
}

pub struct Parameter {
    name: String,
    data: Option<ParameterData>,
    default: ParameterData,
}

#[derive(Args, Debug)]
struct Cli {
    #[arg(short, long, value_name = "param_name")]
    get: Option<String>,

    #[arg(short, long, value_name = "param_name")]
    set: Option<String>,

    #[arg(short, long, requires = "set")]
    value: Option<String>,
}

impl Parameter {
    pub fn set(&mut self, val: ParameterData) {
        self.data = Some(val);
    }

    /*
       return the val of this parameter.
       if the parameter val is None, then return the default value.
    */

    pub fn get(&self) -> &ParameterData {
        if self.data.is_none() {
            &self.default
        } else {
            &self.data.as_ref().unwrap()
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

    pub fn add(&mut self, name: &str, default: ParameterData) {
        let mut param = Parameter {
            name: name.to_string(),
            data: None,
            default,
        };
        self.data.insert(name.to_string(), param);
    }

    pub fn get_val(&self, name: &str) -> Option<&ParameterData> {
        let a = self.data.get(name);

        match a {
            Some(param) => Some(param.get()),
            None => None,
        }
    }

    pub fn get(&mut self, name: &str) -> Option<&mut Parameter> {
        self.data.get_mut(name)
    }
}

static mut PARAMS: OnceLock<ParameterList> = OnceLock::new();

pub fn params_list() -> &'static ParameterList {
    unsafe { PARAMS.get().unwrap() }
}

pub fn param_list_mut() -> &'static mut ParameterList {
    unsafe { PARAMS.get_mut().unwrap() }
}

fn param_main(argc: u32, argv: *const &str) {
    let cli = Command::new("param");
    let mut cli = Cli::augment_args(cli).disable_help_flag(true);
    let argv = unsafe { std::slice::from_raw_parts(argv, argc as usize) };

    let matches = cli.clone().try_get_matches_from(argv);

    if matches.is_err(){
        let help_str = cli.render_help();
        thread_logln!("{}",help_str);
        return ;
    }

    let matches = matches.unwrap();
    let get = matches.get_one::<String>("get");

    if let Some(name) = get {
        thread_logln!("param:{}  val:{:?}", name, params_list().get_val(name));
    }

    if let Some(name) = matches.get_one::<String>("set") {
        let value = matches.get_one::<String>("value");
        let param = param_list_mut().get(name);
        if param.is_some() {
            match value {
                Some(v) => {
                    if let Ok(x) = i32::from_str(v) {
                        param.unwrap().set(ParameterData::Int(x));
                    } else if let Ok(x) = f32::from_str(v) {
                        param.unwrap().set(ParameterData::Float(x));
                    }
                }
                None => thread_logln!("You should provide a value to write into parameter!"),
            }
        } else {
            thread_logln!("Could not find this parameter:{}!", name);
        }
    }
}

#[rpos::ctor::ctor]
fn register() {
    unsafe {
        let _ = PARAMS.set(ParameterList::new());
    };
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
    fn test_parameters_add() {
        let mut params = ParameterList::new();
        params.add("gyro_set", ParameterData::Int(0));

        let val = params.get_val("gyro_set").unwrap();
        assert_eq!(val.get_i32(), 0);
        assert!(params.get_val("undefined").is_none());

        let x = params.get("gyro_set").unwrap();
        x.set(ParameterData::Int(50));

        assert_eq!(x.data.as_ref().unwrap().get_i32(), 50);
        assert_eq!(params.get_val("gyro_set").unwrap().get_i32(), 50);
    }

    #[test]
    fn test_parameters_f32() {
        let mut params = ParameterList::new();
        params.add("f32_test", ParameterData::Float(0.5));

        params
            .get("f32_test")
            .unwrap()
            .set(ParameterData::Float((1.0)));
        let val = params.get_val("f32_test").unwrap();
        assert!(val.get_f32() > 0.9999);
        assert!(val.get_f32() < 1.0001);

        params.get("f32_test").unwrap().reset();
        let val = params.get_val("f32_test").unwrap();
        assert!(val.get_f32() > 0.49999);
        assert!(val.get_f32() < 0.50001);
    }

    #[test]
    fn test_static_params() {
        let list = unsafe { PARAMS.get_mut().unwrap() };
        list.add("test", ParameterData::Int(0));

        let val = unsafe { PARAMS.get().unwrap().get_val("test").unwrap() };
        assert_eq!(val.get_i32(), 0);
    }
}
