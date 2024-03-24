use std::{
    fs::OpenOptions,
    io::{Read, Write},
};

use clap::{Args, Parser};
use rpos::{msg::get_new_rx_of_message, pthread_scheduler::SchedulePthread, thread_logln};
use spidev::{SpiModeFlags, Spidev, SpidevOptions, SpidevTransfer};

use crate::{
    elrs::client_process_args,
    mixer::{MixerOutputMsg, OutputMode},
};

#[derive(Args, Clone)]
struct SpideviceArgs {
    #[arg(short, long)]
    dev_name: String,

    #[arg(long, default_value_t = 3)]
    spi_mode: u8,

    #[arg(long, default_value_t = 20_000_000)]
    spi_max_freq: u32,
}

#[derive(Parser, Clone)]
#[command(name = "fpga_spi_pwm")]
struct Cli {
    #[arg(short, long, default_value_t = 420000)]
    baudrate: u32,

    #[arg(long, default_value_t = 240_000_000)]
    pll_freq: u32,

    #[arg(long, default_value_t = 4)]
    predivider: u8,

    #[arg(long, default_value_t = 8)]
    channel_num: u8,

    #[arg(long, value_delimiter = ',', help = "allocate channel to aux channel")]
    aux_channels: Vec<u8>,

    #[arg(long, default_value_t = 500)]
    main_freq: u32,
    #[arg(long, default_value_t = 500)]
    aux_freq: u32,

    #[arg(short, long)]
    firmware: Option<String>,

    #[command(flatten)]
    spi_args: SpideviceArgs,
}

mod regs {
    pub const SUBPWM0_PERIOD: u8 = 0x00;
    pub const CCR0: u8 = 0x01;
    pub const CCR1: u8 = 0x02;
    pub const CCR2: u8 = 0x03;
    pub const CCR3: u8 = 0x04;
    pub const CCR4: u8 = 0x05;
    pub const CCR5: u8 = 0x06;
    pub const CCR6: u8 = 0x07;
    pub const CCR7: u8 = 0x08;
    pub const PWM_CHANNEL_MAP0: u8 = 0x20;
    pub const SUBPWM1_PERIOD: u8 = 0x40;
    pub const CONFIG: u8 = 0x50;
    pub const TIMEOUT_MAX_LOW: u8 = 0x7D;
    pub const TIMEOUT_MAX_HIGH: u8 = 0x7E;
    pub const WATCHDOG: u8 = 0x7F;
}

trait Actator {
    fn set_duty(&mut self, chn: u8, duty: f32) -> Result<(), ()>;

    // pulse_width : us
    fn set_pluse_width(&mut self, chn: u8, freq: f32, pulse_width: f32) -> Result<(), ()> {
        self.set_duty(chn, freq * pulse_width / 1000_0000.0)
    }
}

struct FPGASPIPWM {
    dev: Spidev,
    periods: Vec<u16>,
    freqs: Vec<f32>,
    cli_args: Cli,
}

impl FPGASPIPWM {
    fn new(dev: Spidev, cli: Cli) -> Self {
        let mut fpga_spi_pwm = FPGASPIPWM {
            dev,
            periods: Vec::new(),
            freqs: Vec::new(),
            cli_args: cli.clone(),
        };

        let cal_period = |target_freq: u32| {
            let ret = cli.pll_freq / cli.predivider as u32 / target_freq;
            if ret > u16::MAX as u32 {
                panic!(
                    "Could not generate pwm at freq:{} by pll:{} and predivider:{}",
                    target_freq, cli.pll_freq, cli.predivider
                );
            }
            ret as u16
        };

        let main_period = cal_period(cli.main_freq);
        let aux_period = cal_period(cli.aux_freq);

        fpga_spi_pwm.write_reg(regs::SUBPWM0_PERIOD, main_period);
        fpga_spi_pwm.write_reg(regs::SUBPWM1_PERIOD, aux_period);

        fpga_spi_pwm
            .periods
            .append(&mut vec![main_period; cli.channel_num as usize]);

        fpga_spi_pwm
            .freqs
            .append(&mut vec![cli.main_freq as f32; cli.channel_num as usize]);

        fpga_spi_pwm.write_reg(regs::CONFIG, 1 << 14 | (cli.predivider - 1) as u16);

        let mut chn_map: u16 = 0;
        for i in cli.aux_channels {
            chn_map |= 1 << (i * 2);
            fpga_spi_pwm.periods[i as usize] = aux_period;
            fpga_spi_pwm.freqs[i as usize] = cli.aux_freq as f32;
        }

        fpga_spi_pwm.write_reg(regs::PWM_CHANNEL_MAP0, chn_map);

        println!("chn num:{}", cli.channel_num);
        println!("chn num:{}", fpga_spi_pwm.cli_args.channel_num);

        fpga_spi_pwm
    }

    fn write_reg(&mut self, addr: u8, value: u16) {
        let buf = [addr << 1 | 1, (value >> 8) as u8, (value & 0xff) as u8];
        self.dev.write(&buf).unwrap();
    }

    fn read_reg(&mut self, addr: u8) -> u16 {
        let tx_buf = [addr << 1 | 0, 0x0, 0x0];
        let mut rx_buf = [0; 3];
        let mut trans = SpidevTransfer::read_write(&tx_buf, &mut rx_buf);
        self.dev.transfer(&mut trans).unwrap();

        (rx_buf[1] as u16) << 8 | rx_buf[2] as u16
    }

    fn stop_all(&mut self) {
        for i in 0..self.cli_args.channel_num {
            self.write_reg(regs::CCR0 + i, 0);
        }
    }
}

impl Actator for FPGASPIPWM {
    fn set_duty(&mut self, chn: u8, duty: f32) -> Result<(), ()> {
        let period = self.periods[chn as usize];
        let duty = duty.clamp(0.0, 1.0);
        self.write_reg(regs::CCR0 + chn as u8, (duty * period as f32) as u16);
        Ok(())
    }
}
fn process_spi_args(args: &SpideviceArgs) -> Spidev {
    let dev_name = &args.dev_name;
    let mut spidev = Spidev::open(dev_name).unwrap();

    let spi_mode = match args.spi_mode {
        0 => SpiModeFlags::SPI_MODE_0,
        1 => SpiModeFlags::SPI_MODE_1,
        2 => SpiModeFlags::SPI_MODE_2,
        3 => SpiModeFlags::SPI_MODE_3,
        _ => panic!("unsupport spi mode!"),
    };

    let option = SpidevOptions::new()
        .bits_per_word(8)
        .max_speed_hz(args.spi_max_freq)
        .mode(spi_mode)
        .build();
    spidev.configure(&option).unwrap();

    spidev
}

pub fn fpga_spi_pwm_init(argc: u32, argv: *const &str) {
    let ret = client_process_args::<Cli>(argc, argv);
    if ret.is_none() {
        return;
    }
    let args = ret.unwrap();
    if args.firmware.is_some() {
        flash_firmware(&args.firmware.clone().unwrap(), &args.spi_args.dev_name);
    }
    let spidev = process_spi_args(&args.spi_args);
    let mut rx = get_new_rx_of_message::<MixerOutputMsg>("mixer_output").unwrap();

    let mut fpga_spi_pwm = Box::new(FPGASPIPWM::new(spidev, args));
    // move the data to heap. (if the data place in stack, the addr may change when being moved)

    let ptr = &*fpga_spi_pwm as *const FPGASPIPWM as rpos::libc::c_ulong;

    // add a panic hook, so that we could stop the motors when panic.
    let default_hook = std::panic::take_hook();
    std::panic::set_hook(Box::new(move |a| {
        let c = ptr as *mut FPGASPIPWM;
        unsafe {
            (*c).stop_all();
        }
        println!("shutdown!");
        default_hook(a);
    }));

    SchedulePthread::new_fifo(
        8192,
        96,
        Box::new(move |s| loop {
            if let Some(mixer_out) = rx.try_read() {
                for i in *mixer_out.output {
                    match i.mode {
                        OutputMode::Duty => fpga_spi_pwm.set_duty(i.chn, i.data).unwrap(),
                        OutputMode::PluseWidth => fpga_spi_pwm
                            .set_pluse_width(i.chn, fpga_spi_pwm.freqs[i.chn as usize], i.data)
                            .unwrap(),
                        _ => {}
                    };
                }
            }
            s.schedule_until(2000);
        }),
    );
}

fn flash_firmware(firmware_path: &String, dev_name: &String) {
    let mut firmware_buf: Vec<u8> = vec![0x3b];
    // 0x3b is the write command for fpga

    let mut spidev = Spidev::open(dev_name).unwrap();
    spidev
        .configure(&SpidevOptions::new().mode(SpiModeFlags::SPI_MODE_0).build())
        .unwrap();
    // flash firmware use a fixed spi option
    // so we do it here

    let mut f = OpenOptions::new().read(true).open(firmware_path).unwrap();
    f.read_to_end(&mut firmware_buf).unwrap();

    const READ_ID_CODE: u8 = 0x11;
    const USER_CODE: u8 = 0x13;
    const STATUS_CODE: u8 = 0x41;

    let get_code = |spidev: &mut Spidev, code| {
        let mut tx_buf: [u8; 8] = [0; 8];
        tx_buf[0] = code;
        let mut rx_buf = [0; 8];
        let mut trans = SpidevTransfer::read_write(&tx_buf, &mut rx_buf);
        spidev.transfer(&mut trans).unwrap();
        let result: u32 = (rx_buf[4] as u32) << 24
            | (rx_buf[5] as u32) << 16
            | (rx_buf[6] as u32) << 8
            | rx_buf[7] as u32;
        result
    };

    let get_all_code = |spidev: &mut Spidev| {
        [READ_ID_CODE, USER_CODE, STATUS_CODE].map(|code| {
            thread_logln!("get code:{:#x} val:{:#x}", code, get_code(spidev, code));
        })
    };

    // erase
    spidev.write(&[0x05, 0x0]).unwrap();

    // rewrite
    spidev.write(&[0x3C, 0x0]).unwrap();

    get_all_code(&mut spidev);

    // write enable
    spidev.write(&[0x15, 0x0]).unwrap();

    spidev.write(&firmware_buf).unwrap();

    // write disable
    spidev.write(&[0x3A, 0x0]).unwrap();

    get_all_code(&mut spidev);
}

#[rpos::ctor::ctor]
fn register() {
    rpos::module::Module::register("fpga_spi_pwm", fpga_spi_pwm_init);
}
