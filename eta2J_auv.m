function J = eta2J_auv(eta)
%ETA2J_AUV 此处显示有关此函数的摘要
%   Calculate Jacobian matrix
[J00,J11,J22]=eulerang(eta(2),eta(3),eta(4));
J=[1,zeros(1,3);zeros(3,1),J22];

end

