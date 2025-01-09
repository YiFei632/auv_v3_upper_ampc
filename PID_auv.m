function output = PID_auv(p,i,d)
%PID 此处显示有关此函数的摘要
%   此处显示详细说明
global Kp;
global Ki;
global Kd;

output=Kp*p+Ki*i+Kd*d;
for i=1:4
    if output(i)>3.48
        output(i)=3.48;
    elseif output(i)<-1.74
        output(i)=1.74;
    end

end

