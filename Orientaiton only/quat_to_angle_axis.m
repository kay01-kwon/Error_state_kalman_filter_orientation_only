function [angle, axis] = quat_to_angle_axis(q)
%QUAT_TO_ANGLE_AXIS 이 함수의 요약 설명 위치
%   자세한 설명 위치

qw = q(1);
q_vec = sign(qw)*q(2:4);
q_vec_norm = sqrt(q_vec'*q_vec);
q_vec_normalized = q_vec/q_vec_norm;

sin_half_angle = q_vec_norm;


angle = 2*atan2(sin_half_angle, qw);

if angle > 1e-30
    axis = q_vec_normalized/sin_half_angle;
else
    axis = zeros(3,1);
end

end