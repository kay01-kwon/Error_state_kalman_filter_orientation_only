function M = pd_control(state, q_ref, P_gain, D_gain)
%PD_CONTROL 이 함수의 요약 설명 위치
%   자세한 설명 위치

q_state = state(1:4);
q_state_conj = [q_state(1); 
               -q_state(2:4)];

q_tilde = otimes(q_state_conj, q_ref);

qw = q_tilde(1);
q_vec = -q_tilde(2:4);

% qw = state(1);
% q_vec = state(2:4);

w = state(5:7);

M = - P_gain*sign(qw)*q_vec - D_gain*w;

end

