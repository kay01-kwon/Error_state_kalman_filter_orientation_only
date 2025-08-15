function [s_pred, P_k_pred] = prediction_step(s_k,P_k,w_imu, time_prev, time_curr)
%PREDICTION_STEP 이 함수의 요약 설명 위치
%   자세한 설명 위치
dt = time_curr - time_prev;
q_k = s_k(1:4);
b_gyro_k = s_k(5:7);
w_k = w_imu - b_gyro_k;
w_k_quat = [0;w_k];

q_pred = q_k + 0.5*otimes(q_k, w_k_quat)*dt;
q_pred = q_pred/norm(q_pred,2);
b_gyro_pred = b_gyro_k;

s_pred = [q_pred;b_gyro_pred];

R = angle_axis_vec_to_rotm(w_k*dt);

Fs = [R', -eye(3)*dt;
    zeros(3,3), eye(3)];

Fi = eye(6);

Q = [0.00001^2*eye(3), zeros(3,3);
    zeros(3,3), 0.00001^2*eye(3)];

% Fs
% 
% P_k
% 
% Fi
% 
% Q
P_k_pred = Fs*P_k*Fs' + Fi*Q*Fi';


end

