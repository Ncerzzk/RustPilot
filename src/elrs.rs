use std::{fs::OpenOptions, io::Read};

use clap::{ArgMatches, Args, Command};
use rpos::{channel::Sender, msg::get_new_tx_of_message, thread_logln};

use crate::msg_define::RcInputMsg;

#[derive(Args, Debug)]
struct Cli {
    #[arg(short, long, default_value_t = 420000)]
    baudrate: u32,

    dev_name: String,
}

struct Elrs {
    tx: Sender<RcInputMsg>,
    dev: Box<dyn Read>,
    parser: crsf::CrsfPacketParser,
}

impl Elrs {
    fn new(dev_name: &String) -> Self {
        let file = OpenOptions::new().read(true).open(dev_name).unwrap();
        Elrs {
            tx: get_new_tx_of_message("rc_input").unwrap(),
            dev: Box::new(file),
            parser: crsf::CrsfPacketParser::default(),
        }
    }

    fn process(&mut self) {
        let mut buf = [0; 1024];
        let len = self.dev.read(&mut buf).unwrap();
        self.parser.push_bytes(&buf[..len]);

        while let Some(packet) = self.parser.next_packet() {
            match packet {
                crsf::Packet::RcChannelsPacked(channels) => {
                    let mut v: [u16; 8] = [0; 8];
                    v.copy_from_slice(&channels[0..8]);
                    self.tx.send(RcInputMsg { channel_vals: v })
                }
                _ => {}
            }
        }
    }
}

pub fn client_process_args(
    argc: u32,
    argv: *const &str,
    cmd_name: &'static str,
) -> Option<ArgMatches> {
    let cli = Command::new(cmd_name);
    let mut cli = Cli::augment_args(cli).disable_help_flag(false);
    let argv = unsafe { std::slice::from_raw_parts(argv, argc as usize) };

    let matches = cli.clone().try_get_matches_from(argv);

    if matches.is_err() {
        let help_str = cli.render_help();
        thread_logln!("{}", help_str);
    }
    matches.ok()
}

pub fn elrs_main(argc: u32, argv: *const &str) {
    if let Some(args) = client_process_args(argc, argv, "elrs") {
        let dev_name = args.get_one::<String>("dev_name").unwrap();
        thread_logln!("dev:{}", dev_name);
    }
}

#[rpos::ctor::ctor]
fn register() {
    rpos::module::Module::register("elrs", elrs_main);
}

#[cfg(test)]
mod tests {
    use std::fs::OpenOptions;
    use std::io::Read;
    use std::io::Seek;
    use std::io::Write;

    use bitfield::bitfield;
    use bitfield::BitRangeMut;

    bitfield! {
        struct RcChannelsRaw([u8]);
        u16;
        ch0, _: 10, 0;
        ch1, _: 21, 11;
        ch2, _: 32, 22;
        ch3, _: 43, 33;
        ch4, _: 54, 44;
        ch5, _: 65, 55;
        ch6, _: 76, 66;
        ch7, _: 87, 77;
        ch8, _: 98, 88;
        ch9, _: 109, 99;
        ch10, _: 120, 110;
        ch11, _: 131, 121;
        ch12, _: 142, 132;
        ch13, _: 153, 143;
        ch14, _: 164, 154;
        ch15, _: 175, 165;
    }
    use super::*;
    use crc::Crc;
    use crc::CRC_8_DVB_S2;
    use crsf::CrsfPacketParser;
    use crsf::Destination;
    use crsf::Packet;
    use crsf::PacketType;
    use rand::thread_rng;
    use rand::Rng;
    use rpos::msg::get_new_rx_of_message;

    fn new_rc_channel_packet(dest: Destination, channel_vals: &[u16]) -> [u8; 26] {
        let mut buf: [u8; 26] = [0; 26];
        buf[0] = dest as u8;
        buf[1] = 0x18;
        buf[2] = PacketType::RcChannelsPacked as u8;
        let mut a = RcChannelsRaw(&mut buf[3..=24]);
        for (index, val) in channel_vals.iter().enumerate() {
            a.set_bit_range(11 * (index + 1) - 1, 11 * index, *val);
        }
        let crc8_alg = Crc::<u8>::new(&CRC_8_DVB_S2);
        buf[25] = crc8_alg.checksum(&buf[2..buf.len() - 1]);
        buf
    }

    #[test]
    fn test_new_rc_channel_packet() {
        let buf = new_rc_channel_packet(Destination::Transmitter, &[1000; 16]);
        let parse_result = Packet::parse(&buf);
        assert!(parse_result.is_some());
        let packet = parse_result.unwrap();
        match packet {
            Packet::RcChannelsPacked(x) => {
                x.iter().for_each(|y| {
                    assert_eq!(*y, 1000);
                });
            }
            _ => panic!("failed"),
        }
    }
    #[test]
    fn elrs_basic_test() {
        let mut file = OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .open("elrs_test")
            .unwrap();
        let mut rng = thread_rng();
        for i in 0..5 {
            let buf = new_rc_channel_packet(Destination::Controller, &[1000 + (i * 100); 16]);
            file.write(&buf).unwrap();

            let mut rand_vals: Vec<u8> = Vec::new();
            let rand_num = rng.gen_range(0..5);
            for _ in 0..rand_num {
                let temp = rng.gen_range(0..255) as u8;
                rand_vals.push(temp);
            }
            file.write(&rand_vals).unwrap();
        }
        let mut read_buf: Vec<u8> = Vec::new();
        file.rewind().unwrap();
        file.read_to_end(&mut read_buf).unwrap();

        let mut parser = CrsfPacketParser::default();
        parser.push_bytes(&read_buf);

        let mut cnt = 0;
        while let Some(packet) = parser.next_packet() {
            match packet {
                Packet::RcChannelsPacked(channels) => {
                    assert_eq!(channels[0], 1000 + cnt * 100);
                    cnt += 1;
                    println!("{:?}", channels);
                }
                _ => panic!("failed"),
            }
        }
        drop(file);

        let mut elrs = Elrs::new(&"elrs_test".to_string());
        let mut rx = get_new_rx_of_message::<RcInputMsg>("rc_input").unwrap();
        elrs.process();
        let msg = rx.read();
        assert_eq!(msg.channel_vals[0], 1000 + (cnt - 1) * 100);
        println!("{:?}", msg.channel_vals);
    }
}
