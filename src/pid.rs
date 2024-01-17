use std::default;

pub struct PIDController {
    KP: f32,
    KI: f32,
    KD: f32,

    last_err: f32,
    i_err: f32, //integration of err
}

impl PIDController {
    pub fn calcuate(&mut self, target: f32, now: f32, dt: f32) -> f32 {
        let err = target - now;
        self.i_err += err * dt;
        let out = err * self.KP + self.i_err * self.KI + (err - self.last_err) / dt * self.KD;
        self.last_err = err;
        out
    }

    pub fn new(KP: f32, KI: f32, KD: f32) -> Self {
        PIDController {
            KP,
            KI,
            KD,
            last_err: 0.0,
            i_err: 0.0,
        }
    }
}
