function q_res = otimes(q1,q2)
%OTIMES 이 함수의 요약 설명 위치
%   자세한 설명 위치

qw = q1(1);
qx = q1(2);
qy = q1(3);
qz = q1(4);

q1_L = [qw, -qx, -qy, -qz;
        qx, qw, -qz, qy;
        qy, qz, qw, -qx;
        qz, -qy, qx, qw];

q_res = q1_L*q2;

end

