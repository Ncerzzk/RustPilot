pub struct PIDController {
    kp: f32,
    ki: f32,
    kd: f32,

    last_err: f32,
    i_err: f32, //integration of err
}

impl PIDController {
    pub fn calcuate(&mut self, err:f32, dt: f32) -> f32 {
        self.i_err += err * dt;
        let out = err * self.kp + self.i_err * self.ki + (err - self.last_err) / dt * self.kd;
        self.last_err = err;
        out
    }

    pub fn new(kp: f32, ki: f32, kd: f32) -> Self {
        PIDController {
            kp,
            ki,
            kd,
            last_err: 0.0,
            i_err: 0.0,
        }
    }
}
