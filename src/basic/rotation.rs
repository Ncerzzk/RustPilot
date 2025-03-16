

use quaternion_core::Quaternion;
use std::f32::consts::PI;
use quaternion_core::RotationSequence::*;
use quaternion_core::RotationType::*;

use crate::msg_define::Vector3Msg;

#[allow(dead_code)]
pub fn get_euler_degree(q:quaternion_core::Quaternion<f32>)->[f32;3]{
    quaternion_core::to_euler_angles(Intrinsic, XYZ, q).map(|x|{
        x/PI * 180.0
    })
}

#[allow(dead_code)]
pub enum Rotation{
    Yaw90,
    Yaw180,
    Yaw270,
}

impl Rotation{
    pub fn rotate(&self,q:[f32;3])->[f32;3]{
        match self{
            Rotation::Yaw90 =>q,
            Rotation::Yaw180 => q,
            Rotation::Yaw270 => {
                [-q[1], q[0],q[2]]
            },
        }
    }

    #[inline]
    pub fn rotate_q(&self,q:Quaternion<f32>)->Quaternion<f32>{
        (q.0,self.rotate(q.1))
    }

    #[inline]
    pub fn rotate_v(&self,q:Vector3Msg)->Vector3Msg{
        let ret = self.rotate([q.x,q.y,q.z]);
        Vector3Msg{
            x:ret[0],
            y:ret[1],
            z:ret[2]
        }
    }
}


#[cfg(test)]
mod tests{
    use super::*;

    fn check_q_eq(q1:Quaternion<f32>,q2:Quaternion<f32>){
        assert!((q1.0 - q2.0).abs() < 1e-12);
        assert!((q1.1[0] - q2.1[0]).abs() < 1e-12);
        assert!((q1.1[1] - q2.1[1]).abs() < 1e-12);
        assert!((q1.1[2] - q2.1[2]).abs() < 1e-12);

    }
    #[test]
    fn test_rotate(){
        const ANGLE:f32 = PI/6.0;
        let q = quaternion_core::from_axis_angle([1.0,0.0,0.0], ANGLE);
        let r = Rotation::Yaw270;
        let rq = r.rotate_q(q);
        check_q_eq(rq, quaternion_core::from_axis_angle([0.0,1.0,0.0], ANGLE));
    }

}