use core::panic;
use std::sync::LazyLock;

use clap::Args;
use dashmap::DashMap;

#[derive(Debug, Clone, Copy)]
pub enum ParameterData {
    Bool(bool),
    Int(i32),
    Float(f32),
}

impl ParameterData {
    pub fn union_to_f32(&self) -> f32 {
        match self {
            ParameterData::Bool(x) => unsafe { std::mem::transmute(*x as u32) },
            ParameterData::Int(x) => unsafe { std::mem::transmute(*x) },
            ParameterData::Float(x) => *x,
        }
    }

    pub fn as_i32(&self) -> i32 {
        if let Self::Int(x) = self {
            x.clone()
        } else {
            panic!("this param is not i32!")
        }
    }

    pub fn as_f32(&self) -> f32 {
        if let Self::Float(x) = self {
            x.clone()
        } else {
            panic!("this param is not f32!")
        }
    }

    pub fn as_bool(&self) -> bool {
        if let Self::Bool(x) = self {
            x.clone()
        } else {
            panic!("this param is not bool!")
        }
    }
}

pub struct Parameter {
    //name: String,
    data: Option<ParameterData>,
    default: ParameterData,
}

impl Parameter {
    pub fn get_data(&self) -> ParameterData {
        if self.data.is_none() {
            self.default
        } else {
            self.data.unwrap()
        }
    }
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

static PARAMS: LazyLock<DashMap<String, Parameter>> = LazyLock::new(|| DashMap::new());

pub fn get_params_map() -> &'static DashMap<String, Parameter> {
    &PARAMS
}

pub fn get_param(name: &str) -> Option<ParameterData> {
    if let Some(parameter) = PARAMS.try_get(name).try_unwrap() {
        Some(parameter.get_data())
    } else {
        None
    }
}

pub fn reset_param(name: &str) -> Result<(), ()> {
    if let Some(mut x) = PARAMS.try_get_mut(name).try_unwrap() {
        x.data = None;
        Ok(())
    } else {
        Err(())
    }
}

pub fn set_param(name: &str, val: ParameterData) -> Result<(), ()> {
    if let Some(mut x) = PARAMS.try_get_mut(name).try_unwrap() {
        x.data = Some(val);
        Ok(())
    } else {
        Err(())
    }
}

pub fn add_param(name: &str, default: ParameterData) {
    assert!(name.len() < 16); // mavlink parameter name should < 16, let's follow them.
    PARAMS.insert(
        name.to_string(),
        Parameter {
            data: None,
            default,
        },
    );
}

fn param_main(_argc: u32, _argv: *const &str) {

    // let cli = Command::new("param");
    // let mut cli = Cli::augment_args(cli).disable_help_flag(true);
    // let argv = unsafe { std::slice::from_raw_parts(argv, argc as usize) };

    // let matches = cli.clone().try_get_matches_from(argv);

    // if matches.is_err(){
    //     let help_str = cli.render_help();
    //     thread_logln!("{}",help_str);
    //     return ;
    // }

    // let matches = matches.unwrap();
    // let get = matches.get_one::<String>("get");

    // if let Some(name) = get {
    //     thread_logln!("param:{}  val:{:?}", name, params_list().get_val(name));
    // }

    // if let Some(name) = matches.get_one::<String>("set") {
    //     let value = matches.get_one::<String>("value");
    //     let param = param_list_mut().get(name);
    //     if param.is_some() {
    //         match value {
    //             Some(v) => {
    //                 if let Ok(x) = i32::from_str(v) {
    //                     param.unwrap().set(ParameterData::Int(x));
    //                 } else if let Ok(x) = f32::from_str(v) {
    //                     param.unwrap().set(ParameterData::Float(x));
    //                 }
    //             }
    //             None => thread_logln!("You should provide a value to write into parameter!"),
    //         }
    //     } else {
    //         thread_logln!("Could not find this parameter:{}!", name);
    //     }
    // }
}

#[rpos::ctor::ctor]
fn register() {
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
        add_param("gyro_set", ParameterData::Int(0));

        let val = get_param("gyro_set").unwrap();
        assert_eq!(val.as_i32(), 0);
        assert!(get_param("undefined").is_none());

        let x = set_param("gyro_set", ParameterData::Int(50)).unwrap();

        assert_eq!(get_param("gyro_set").unwrap().as_i32(), 50);
    }

    #[test]
    fn test_parameters_f32() {
        add_param("f32_test", ParameterData::Float(0.5));

        set_param("f32_test", ParameterData::Float(1.0));

        let val = get_param("f32_test").unwrap();
        assert!(val.as_f32() > 0.9999);
        assert!(val.as_f32() < 1.0001);
    }

    #[test]
    fn test_key_from_c() {
        add_param("ctest", ParameterData::Bool(true));
        let c_str = ['c' as u8, 't' as u8, 'e' as u8, 's' as u8, 't' as u8, 0, 0];

        assert!(PARAMS.contains_key(
            std::ffi::CStr::from_bytes_until_nul(&c_str)
                .unwrap()
                .to_str()
                .unwrap()
        ));
    }
}
