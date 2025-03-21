pub mod scaler;
pub mod rotation;
pub mod pid;

pub fn client_process_args<T:clap::Parser>(
    argc: u32,
    argv: *const &str
) -> Option<T> {

    let argv = unsafe { std::slice::from_raw_parts(argv, argc as usize) };

    let ret = T::try_parse_from(argv);

    if ret.is_err() {
        let help_str = T::command().render_help();
        rpos::thread_logln!("{}", help_str);
        return None
    }
    ret.ok()
}
