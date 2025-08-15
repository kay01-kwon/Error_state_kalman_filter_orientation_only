function res = vec2skew(vec)
%VEC2SKEW 이 함수의 요약 설명 위치
%   자세한 설명 위치

vx = vec(1);
vy = vec(2);
vz = vec(3);

res = [0, -vz, vy;
      vz, 0, -vx;
      -vy, vx, 0];

end

