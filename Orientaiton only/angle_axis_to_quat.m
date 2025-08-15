function q = angle_axis_to_quat(angle, axis)
%ANGLE_AXIS_TO_QUATERNION 이 함수의 요약 설명 위치
%   자세한 설명 위치

% Normalize axis vector
axis = axis/sqrt(axis'*axis);
q = [cos(angle/2);
    axis*sin(angle/2)];

end

