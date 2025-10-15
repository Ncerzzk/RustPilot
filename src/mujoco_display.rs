// ui.rs
use mujoco_rs_sys::render;
use mujoco_rs_sys::no_render;
use glfw;
use glfw::Context;
use gl;
use mujoco_rust::Simulation;
use std::ptr;

use mujoco_rust::model::ObjType;
use image::{RgbImage, ImageBuffer};
use std::process::{Command, Stdio};

pub struct UIState {
    pub cameras: Vec<render::mjvCamera_>,
    pub opt: render::mjvOption_,
    pub scenes: Vec<render::mjvScene_>,
    pub contexts: Vec<render::mjrContext_>,
    pub window: glfw::Window,
}

pub fn glfw_init(simulation: &Simulation, cam_ids: &[i32]) -> UIState {
    let mut glfw = glfw::init(glfw::FAIL_ON_ERRORS).unwrap();
    glfw.default_window_hints();

    let (mut window, events) = if cam_ids[0] == 0x7FFFFFFF {
        glfw.create_window(1200, 900, "MuJoCo UI", glfw::WindowMode::Windowed)
            .expect("Unable to create visible GLFW window.")
    } else {
        glfw.window_hint(glfw::WindowHint::Visible(false));
        glfw.create_window(640, 480, "hidden", glfw::WindowMode::Windowed)
            .expect("Unable to create hidden GLFW window.")
    };

    // window settings init
    window.make_current();
    window.set_key_polling(true);
    window.set_cursor_pos_polling(true);
    window.set_mouse_button_polling(true);
    window.set_scroll_polling(true);

    gl::load_with(|symbol| window.get_proc_address(symbol) as *const _);

    // init UIstate
    let mut cameras = Vec::new();
    let mut scenes = Vec::new();
    let mut contexts = Vec::new();
    let mut opt = render::mjvOption_::default();

    for &cam_id in cam_ids {
        let mut cam = render::mjvCamera_::default();
        let mut scn = render::mjvScene_::default();
        let mut con = render::mjrContext_::default();

        unsafe {
            // init camera & scene
            no_render::mjv_defaultCamera(&mut cam);
            render::mjv_defaultScene(&mut scn);
            render::mjr_defaultContext(&mut con);

            no_render::mjv_makeScene(simulation.model.ptr(), &mut scn, 2000);
            render::mjr_makeContext(simulation.model.ptr(), &mut con, 200);
        }
        if cam_id == 0x7FFFFFFF {
            cam.type_ = 1; // free viewport
            cam.trackbodyid = 1; // Set tracked object ID
            cam.distance = 5.0;
        } else {
            cam.fixedcamid = cam_id;
            cam.type_ = 2; // fix camera
        }

        cameras.push(cam);
        scenes.push(scn);
        contexts.push(con);
    }

    unsafe {
        no_render::mjv_defaultOption(&mut opt);
    }

    UIState {
        cameras,
        opt,
        scenes,
        contexts,
        window,
    }
}

pub fn glfw_update_scene(simulation: &Simulation, ui_state: &mut UIState) {
    ui_state.window.make_current();
    unsafe {
        // get window size
        let (width, height) = ui_state.window.get_framebuffer_size();
        let num_cameras = ui_state.cameras.len().min(4);
        let cols = if num_cameras < 2 { 1 } else { 2 };
        let rows = if num_cameras <= 2 { 1 } else { 2 };
        let sub_window_width = width / cols as i32;
        let sub_window_height = height / rows as i32;

        // clear buffer
        gl::Clear(gl::COLOR_BUFFER_BIT | gl::DEPTH_BUFFER_BIT);

        // update & render
        for i in 0..num_cameras {
            no_render::mjv_updateScene(
                simulation.model.ptr(),
                simulation.state.ptr(),
                &ui_state.opt,
                ptr::null(),
                &mut ui_state.cameras[i],
                0xFFFFFF,
                &mut ui_state.scenes[i],
            );

            // calc sub window's pos in the main window
            let row = i / cols;
            let col = i % cols;

            // define sub window viewport
            let viewport = render::mjrRect_ {
                left: col as i32 * sub_window_width,
                bottom: (rows - 1 - row) as i32 * sub_window_height, // 从底部开始，翻转行顺序
                width: sub_window_width,
                height: sub_window_height,
            };

            // render scene
            render::mjr_render(viewport, &mut ui_state.scenes[i], &mut ui_state.contexts[i]);
        }

        // swap buffer to display render scene
        ui_state.window.swap_buffers();
    }
}

pub fn init_ffmpeg() -> std::process::Child {
    let rtsp_url = "rtsp://localhost:8554/mystream".to_string();
    let mut ffmpeg = Command::new("ffmpeg")
        .args([
            "-f", "rawvideo",
            "-pixel_format", "rgb24",
            "-video_size", "640x480",
            "-framerate", "30",
            "-i", "pipe:",
            "-c:v", "libx264",
            "-pix_fmt", "yuv420p",
            "-preset", "ultrafast",
            "-tune", "zerolatency",
            "-f", "rtsp",
            "-rtsp_transport", "tcp",
            &rtsp_url
        ])
        .stdin(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn() // Spawn the process
        .expect("Failed to start FFmpeg");
    
    ffmpeg
}

pub fn update_mjscene(simulation: &Simulation, ui_state: &mut UIState) -> Vec<u8> {
    ui_state.window.make_current();
    unsafe {
        // get window size
        let (width, height) = ui_state.window.get_framebuffer_size();
        let num_cameras = ui_state.cameras.len().min(4);
        let cols = if num_cameras < 2 { 1 } else { 2 };
        let rows = if num_cameras <= 2 { 1 } else { 2 };
        let sub_window_width = width / cols as i32;
        let sub_window_height = height / rows as i32;

        // clear buffer
        gl::Clear(gl::COLOR_BUFFER_BIT | gl::DEPTH_BUFFER_BIT);

        // update & render
        for i in 0..num_cameras {
            no_render::mjv_updateScene(
                simulation.model.ptr(),
                simulation.state.ptr(),
                &ui_state.opt,
                ptr::null(),
                &mut ui_state.cameras[i],
                0xFFFFFF,
                &mut ui_state.scenes[i],
            );
            // calc sub window's pos in the main window
            let row = i / cols; 
            let col = i % cols;

            // sub window viewport
            let viewport = render::mjrRect_ {
                left: col as i32 * sub_window_width,
                bottom: (rows - 1 - row) as i32 * sub_window_height, // 从底部开始，翻转行顺序
                width: sub_window_width,
                height: sub_window_height,
            };

            // render the scene
            render::mjr_render(viewport, &mut ui_state.scenes[i], &mut ui_state.contexts[i]);
        }

        // 
        let mut rgb = vec![0u8; (width * height * 3) as usize];

        // define full viewport rather than a partial viewport
        let full_viewport = render::mjrRect_ {
            left: 0,
            bottom: 0,
            width: width,
            height: height,
        };

        gl::PixelStorei(gl::PACK_ALIGNMENT, 1);

        // read the main window's Pixels
        render::mjr_readPixels(rgb.as_mut_ptr(), ptr::null_mut(), full_viewport, &mut ui_state.contexts[0]);

        // flips the image
        let mut flipped_rgb = vec![0u8; (width * height * 3) as usize];
        for y in 0..height {
            for x in 0..width {
                let src_idx = ((y * width + x) * 3) as usize;
                let dst_idx = (((height - 1 - y) * width + x) * 3) as usize;
                flipped_rgb[dst_idx] = rgb[src_idx];
                flipped_rgb[dst_idx + 1] = rgb[src_idx + 1];
                flipped_rgb[dst_idx + 2] = rgb[src_idx + 2];
            }
        }
        flipped_rgb
    }
}

pub fn free_resource(ui_state: &mut UIState){
    unsafe {
        for i in 0..ui_state.scenes.len() {
            render::mjv_freeScene(&mut ui_state.scenes[i]);
            render::mjr_freeContext(&mut ui_state.contexts[i]);
        }
    }
}