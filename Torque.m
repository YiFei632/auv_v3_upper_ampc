function torque = Torque(tau)
%TORQUE 此处显示有关此函数的摘要
%   此处显示详细说明
% ix = [0.566 0;0 0.588]; %给Fx用的惯量矩阵，包含Izz和Iyy
% iy = [0.444 0;0 23.85]; %给Fy用的惯量矩阵，包含Ixx和m
dx = [0.65 0;0 0.65]; %给Fx用的力臂矩阵，包含dl
dy = [0.55 0;0 1]; %给Fy用的力臂矩阵，包含dw
% 
tau_x=zeros(2,1);
tau_x(1)=tau(4);
tau_x(2)=tau(3);

tau_y=zeros(2,1);
tau_y(1)=tau(2);
tau_y(2)=tau(1);
% 
Fx = inv(dx)*tau_x;
Fy = inv(dy)*tau_y; %由矩阵乘法计算分力的分量
% 
torque=zeros(4,1);
% torque(1) = realsqrt((Fx(1).^2) + (Fx(2).^2));
% torque(2) = atan((Fx(1)./Fx(2)));
% torque(3) = (Fy(2) - Fy(1)) / 2;
% torque(4) = (Fy(2) + Fy(1)) / 2;
torque(3) = (Fy(2)) / 4;
torque(4) = (Fy(2)) / 4;
torque(1) = - (Fy(2)) / 4;
torque(2) = 0;

if isnan(torque(2))
    torque(2)=0;
end
% 突然想到，现在程序计算的就是tau，所以我也没必要重新重复计算了，直接将推力加和就好
% torque(3) = 3.5;
% torque(4) = 4;
% torque(1) = -4;

end

