function [s_eskf, P_eskf] = measurement_update(s_pred, P_pred, z_meas)
%MEASUREMENT_UPDATE 이 함수의 요약 설명 위치
%   자세한 설명 위치
Hs = [eye(4), zeros(4,3)];

q_pred = s_pred(1:4);
bias_gyro_pred = s_pred(5:7);

qw = q_pred(1);
qx = q_pred(2);
qy = q_pred(3);
qz = q_pred(4);

q_vec = [qx;qy;qz];

Q_del_theta = 0.5*[-qx, -qy, -qz;
                    qw*eye(3) + vec2skew(q_vec)];
S_dels = [Q_del_theta, zeros(4,3);
          zeros(3,3), eye(3,3)];

H = Hs*S_dels;

% Observation of the error state via filter correction
V = 0.001^2*eye(4);

sensor_model = q_pred;

K_kalman = P_pred*H'/(H*P_pred*H' + V);
del_s = K_kalman*(z_meas - sensor_model);
P_eskf = (eye(6) - K_kalman*H)*P_pred;


% Injection of the observed error into the nominal state

del_theta = del_s(1:3);
del_bias_gyro = del_s(4:6);

half_del_theta_quat = [1;0.5*del_theta];

q_eskf = otimes(q_pred, half_del_theta_quat);
bias_gyro_eskf = bias_gyro_pred + del_bias_gyro;

s_eskf = [q_eskf;bias_gyro_eskf];

dtheta_dtheta = eye(3) - 0.5*vec2skew(del_theta);
G = [dtheta_dtheta, zeros(3,3);
    zeros(3,3), eye(3)];

P_eskf = G*P_eskf*G';


end

