
use rpos::server_client::setup_client_stdin_out;
use termion::{self, color, event::Key, input::TermRead, raw::IntoRawMode};

struct Channel {
    name: String,
    add_key: char,
    sub_key: char,
    value: i16,
}

impl Channel {
    fn new(name: &str, add_key: char, sub_key: char, init_val: i16) -> Self {
        Self {
            name: name.to_string(),
            add_key,
            sub_key,
            value: init_val,
        }
    }
}

pub fn init_fake_linux_input(_argc: u32, _argv: *const &str) {
    let thust_chn = Channel::new("Thrust", 'w', 's', 0);
    let direction_chn = Channel::new("Direction", 'd', 'a', 5000);
    let ele_chn = Channel::new("Elevator", 'j', 'k', 5000);
    let aile_chn = Channel::new("Aileron", 'h', 'l', 5000);
    let mut channels = [thust_chn, direction_chn, ele_chn, aile_chn];

    setup_client_stdin_out().unwrap();

    let stdin = std::io::stdin();
    let _stdout = std::io::stdout().into_raw_mode().unwrap();

    print!(
        "{}{}q to exit.\n\r",
        termion::clear::All,
        termion::cursor::Goto(1, 1)
    );

    for c in stdin.keys() {
        let event = c.unwrap();
        if event == Key::Char('q') {
            break;
        }
        for channel in &mut channels {
            if event == Key::Char(channel.add_key) {
                channel.value += 100;
            } else if event == Key::Char(channel.sub_key) {
                channel.value -= 100;
            }

            channel.value = channel.value.clamp(0, 10000);
        }

        print!(
            "{}{}",
            termion::cursor::Goto(1, 2),
            termion::clear::AfterCursor
        );
        for channel in &channels {
            println!("{}:{}\r", channel.name, channel.value);
        }
    }
    println!("finished!\r");
}

#[rpos::ctor::ctor]
fn register() {
    rpos::module::Module::register("fake_linux_input", init_fake_linux_input);
}
