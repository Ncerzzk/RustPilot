use clap::Parser;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Name of the person to greet
    #[arg(short, long, default_value_t = 1)]
    count: i32,

    #[arg(short, long)]
    topic: String,
}


fn msg_echo_main(_argc:u32, _argv:*const &str){
    let argv = unsafe { std::slice::from_raw_parts(_argv, _argc as usize) };
    let argv= Args::parse_from(argv);

    let topic = argv.topic;

    let sub = rpos::msg::MSGSubscriber::new(topic.as_str());


    println!("count:{}",argv.count);
}

#[rpos::ctor::ctor]
fn register(){
    crate::Module::register("msg_echo", msg_echo_main);
}