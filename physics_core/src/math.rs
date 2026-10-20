// Helper functions
pub fn transpose_3x3(m: &[[f64;3];3]) -> [[f64;3];3] {
    // Transposes a 3x3 matrix
    [
        [m[0][0], m[1][0], m[2][0]],
        [m[0][1], m[1][1], m[2][1]],
        [m[0][2], m[1][2], m[2][2]],
    ]
}

pub fn mult_3x3_3x1(m1: &[[f64;3];3], m2: &[f64;3]) -> [f64;3] {
    // Multiplies a 3x3 matrix and a 3x1 matrix
    [
        m1[0][0] * m2[0] + m1[0][1] * m2[1] + m1[0][2] * m2[2],
        m1[1][0] * m2[0] + m1[1][1] * m2[1] + m1[1][2] * m2[2],
        m1[2][0] * m2[0] + m1[2][1] * m2[1] + m1[2][2] * m2[2],
    ]
}

pub fn quat_to_rot (q: [f64;4]) -> [[f64;3];3] {
    // Converts Quaternions to a 3x3 rotation matrix
     let [qw, qx, qy, qz] = &q;

    [
        [1.0 - 2.0 * ( qy * qy + qz * qz), 2.0 * (qx * qy - qz * qw), 2.0 * (qx * qz + qy * qw)],
        [2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qx * qw)],
        [2.0 * (qx * qz - qy * qw), 2.0 * (qy * qz + qx * qw), 1.0 - 2.0 * (qx * qx + qy * qy)]
    ]
}

pub fn quat_evolve(q: &mut [f64;4], omega: [f64;3], dt:f64) -> [f64;4] {
     // Angular velocity quaternion
    let omega_quat = [
        0.0,
        omega[0],
        omega[1],
        omega[2],
    ];

    // Quaternion derivative: q_dot = 0.5 * q ⊗ omega_quat
    let q_dot = [
        0.5 * (-q[1]*omega_quat[1] - q[2]*omega_quat[2] - q[3]*omega_quat[3]),
        0.5 * ( q[0]*omega_quat[1] + q[2]*omega_quat[3] - q[3]*omega_quat[2]),
        0.5 * ( q[0]*omega_quat[2] + q[3]*omega_quat[1] - q[1]*omega_quat[3]),
        0.5 * ( q[0]*omega_quat[3] + q[1]*omega_quat[2] - q[2]*omega_quat[1]),
    ];

    let q_new = 
    [
        q[0] + dt * q_dot[0],
        q[1] + dt * q_dot[1],
        q[2] + dt * q_dot[2],
        q[3] + dt * q_dot[3]
    ];

    let mag = (q_new[0] * q_new[0] + q_new[1] * q_new[1] + q_new[2] * q_new[2] +  q_new[3] * q_new[3]).sqrt();

        [
            q_new[0] / mag,
            q_new[1] / mag,
            q_new[2] / mag,
            q_new[3] / mag
        ]
    
}