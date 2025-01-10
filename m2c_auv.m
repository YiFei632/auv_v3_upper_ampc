function C = m2c_auv(m,nu)
%M2C_AUV 此处显示有关此函数的摘要
% Calculate C matrix
C=zeros(4,4);
C(2,3)=m(4)*nu(4);
C(3,2)=-m(4)*nu(4);

C(2,4)=-m(3)*nu(3);
C(4,2)=m(3)*nu(3);

C(3,4)=m(2)*nu(2);
C(4,2)=-m(2)*nu(2);

end

