function du = MPC(x,Rs,rsk)
%AMPC 此处显示有关此函数的摘要
%   此处显示详细说明
global A1;
global B1;
global C1;
global Nc;
global Np;
[F,Phi,Phi_Phi,BarR]=Gain(A1,B1,C1,Nc,Np);
E=Phi_Phi+BarR;
H=((F*x-Rs)'*Phi)';

Me = zeros(4,Nc*4);
for k = 1:Nc
    Me(:,4*k-3:4*k)=C1*A1^(Nc-k)*B1;
end
gammae=rsk-C1*A1^(Nc)*x;

options=optimset('LargeScale','off','MaxIter',300,...
        'algorithm','interior-point-convex','display','on');
global deltaU1;
deltaU1=[];
[deltaU1,fval]=quadprog(E,H,[],[],Me,gammae,[],[],[],options);
du = [deltaU1(1);deltaU1(2);deltaU1(3);deltaU1(4)];

end

