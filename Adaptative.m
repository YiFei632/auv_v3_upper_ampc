function hatTheta = Adaptative(x,hatx,du,hatTheta_last)
%ADAPTATIVE 此处显示有关此函数的摘要
%   此处显示详细说明
global sig;
global lambda;
global steps;

X=[x;du];
if sig<steps
    hatTheta=hatTheta_last+lambda*(x-hatx)*X';
end

end
