function rotm = angle_axis_vec_to_rotm(theta_vec)
%ANGLE_AXIS_VEC_TO_ROTM 이 함수의 요약 설명 위치
%   자세한 설명 위치
angle = norm(theta_vec,2);

if angle > 1e-30
    axis = theta_vec/angle;
else
    axis = zeros(3,1);
end

Omega = vec2skew(axis);

rotm = eye(3) + sin(angle)*Omega + (1-cos(angle))*Omega^2;

end

