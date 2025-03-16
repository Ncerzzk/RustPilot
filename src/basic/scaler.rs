use std::ops::Neg;

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub struct Scaler {
    pub scale_p: f32,
    pub scale_n: f32,
    pub offset: f32,
    pub min: f32,
    pub max: f32,
}

impl Default for Scaler {
    fn default() -> Self {
        Self {
            scale_p: 1.0,
            scale_n: 1.0,
            offset: 0.0,
            min: -100.0,
            max: 100.0,
        }
    }
}

impl Neg for Scaler {
    type Output = Self;

    fn neg(self) -> Self::Output {
        Self {
            scale_p: -self.scale_p,
            scale_n: -self.scale_n,
            offset: self.offset,
            min: self.min,
            max: self.max,
        }
    }
}

impl Scaler {
    pub fn scale(&self, input: f32) -> f32 {
        let scale_k;
        if input > 0.0 {
            scale_k = self.scale_p;
        } else {
            scale_k = self.scale_n;
        }
        let ret = scale_k * input + self.offset;
        ret.clamp(self.min, self.max)
    }
}
