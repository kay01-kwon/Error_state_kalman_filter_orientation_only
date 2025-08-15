close all
clear all


dt = 0.01;
Tf = 60*5;
time = 0:dt:Tf;
N = length(time);



s_init = [1, 0, 0, 0, ...
          0, 0, 0]';

q = s_init(1:4);
w = s_init(5:7);

u_ref = [1;1;0];
u_ref = u_ref/sqrt(u_ref'*u_ref);
des_angle = deg2rad(10);
q_ref = [cos(des_angle/2);u_ref*sin(des_angle/2)];

s_true_vec = zeros(7,N);

Param.J = diag([0.052, 0.052, 0.080]);
Param.rx = 0.00;
Param.ry = 0.00;



u = [20, 0., 0, 0]';
d = [0, 0, 0]';
d_true_vec = zeros(3,N);
d_true_vec(:,1) = d;


s_prior = [1, 0, 0, 0, ...
           0, 0, 0, ...
           0, 0, 0]';


K_p = 3*eye(3);
K_d = 0.52*eye(3);

rx = Param.rx;
ry = Param.ry;

s = s_init;



sigma_angle = deg2rad(1);
sigma_axis = [0.01;
              0.01;
              0.01];
sigma_w = [0.001;
           0.001;
           0.001];
bias = zeros(3,1);

RW = 0.00037331694*eye(3);

des_theta_vec = zeros(3,N);
des_theta_vec(:,1) = des_angle*u_ref;
bias_vec = zeros(3,N);
bias_est_vec = zeros(3,N);
theta_vec = zeros(3,N);

s_eskf = zeros(7,1);
s_eskf(1) = 1;
q_eskf_vec = zeros(4,N);
q_eskf_vec(:,1) = s_eskf(1:4);

P_eskf = 0.001^2*eye(6);

w_obs_prev = zeros(3,1);

w_biased_vec = zeros(3,N);
w_unbiased_vec = zeros(3,N);

for i = 1:N-1

    dt = time(i+1) - time(i);

    [angle, axis] = quat_to_angle_axis(q);

    angle_obs = angle + normrnd(0, sigma_angle);
    axis_obs = axis + normrnd(0,sigma_axis);
    
    q_obs = angle_axis_to_quat(angle_obs, axis_obs);

    bias = bias + RW*normrnd(0, ones(3,1))*sqrt(dt);
    bias_vec(:,i+1) = bias;
    w_obs = w + normrnd(0, sigma_w) + bias;
    w_biased_vec(:,i+1) = w_obs;

    obs = [q_obs;w_obs];

    [s_pred, P_pred] = prediction_step(s_eskf, P_eskf, w_obs_prev, time(i), time(i+1));
    [s_eskf, P_eskf] = measurement_update(s_pred, P_pred, q_obs);
    
    q_eskf_vec(:,i+1) = s_eskf(1:4);
    bias_est_vec(:,i+1) = s_eskf(5:7);

    des_theta_vec(:,i+1) = des_angle*u_ref;

    bias_gyro = s_eskf(5:7);

    w_unbiased = w_obs - bias_gyro;
    w_unbiased_vec(:,i+1) = w_unbiased;

    s_feed = [q_obs;w_obs-bias_gyro];


    M = pd_control(s_feed, q_ref, K_p, K_d);
    u(2:4) = M;



    s = rk4_ode(@(t,s) rotational_dynamics(s, u, d, Param), s_init, time(i+1), time(i));
    
    s_true_vec(:,i+1) = s;
    s_init = s;

    q = s(1:4);
    w = s(5:7);
    theta_vec(:,i+1) = quat_to_angle_axis_vector(q);

    w_obs_prev = w_obs;

end


figure(1)
subplot(3,1,1)
plot(time, theta_vec(1,:))
hold on;
plot(time, des_theta_vec(1,:))
title('$\theta_{x}$','Interpreter','latex')

subplot(3,1,2)
plot(time, theta_vec(2,:))
hold on;
plot(time, des_theta_vec(2,:))
title('$\theta_{y}$','Interpreter','latex')

subplot(3,1,3)
plot(time, theta_vec(3,:))
hold on;
plot(time, des_theta_vec(3,:))
title('$\theta_{z}$','Interpreter','latex')


figure(2)
subplot(3,1,1)
plot(time,bias_vec(1,:))
hold on
plot(time,bias_est_vec(1,:))
title('$b_{\omega, x}$ - t','Interpreter','latex')
legend('true','est')
xlabel('time (s)')
ylabel('$b_{\omega, x}$','Interpreter','latex')

subplot(3,1,2)
plot(time,bias_vec(2,:))
hold on
plot(time,bias_est_vec(2,:))
title('$b_{\omega, y}$ - t','Interpreter','latex')
legend('true','est')
xlabel('time (s)')
ylabel('$b_{\omega, y}$','Interpreter','latex')

subplot(3,1,3)
plot(time,bias_vec(3,:))
hold on
plot(time,bias_est_vec(3,:))
title('$b_{\omega, z}$ - t','Interpreter','latex')
legend('true','est')
xlabel('time (s)')
ylabel('$b_{\omega, z}$','Interpreter','latex')

% exportgraphics(gcf,'gyro bias.png','resolution',600)

figure(3)
subplot(4,1,1)
plot(time,s_true_vec(1,:))
hold on
plot(time,q_eskf_vec(1,:))

subplot(4,1,2)
plot(time,s_true_vec(2,:))
hold on
plot(time,q_eskf_vec(2,:))

subplot(4,1,3)
plot(time,s_true_vec(3,:))
hold on
plot(time,q_eskf_vec(3,:))

subplot(4,1,4)
plot(time,s_true_vec(4,:))
hold on
plot(time,q_eskf_vec(4,:))

figure(4)
subplot(3,1,1)
plot(time,w_unbiased_vec(1,:))
hold on
plot(time,w_biased_vec(1,:))
plot(time, s_true_vec(5,:))
title('$\omega_{x}-t$','Interpreter','latex')
xlabel('time (s)')
legend('unbiased', 'biased', 'true', ...
    'location','bestoutside')

subplot(3,1,2)
plot(time,w_unbiased_vec(2,:))
hold on
plot(time,w_biased_vec(2,:))
plot(time, s_true_vec(6,:))
title('$\omega_{y}-t$','Interpreter','latex')
xlabel('time (s)')
legend('unbiased', 'biased', 'true', ...
    'location','bestoutside')

subplot(3,1,3)
plot(time,w_unbiased_vec(3,:))
hold on
plot(time,w_biased_vec(3,:))
plot(time, s_true_vec(7,:))
title('$\omega_{z}-t$','Interpreter','latex')
xlabel('time (s)')
legend('unbiased', 'biased', 'true', ...
    'location','bestoutside')