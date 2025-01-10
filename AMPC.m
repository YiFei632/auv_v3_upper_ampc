function du = AMPC(x,Rs,rsk)
%AMPC 此处显示有关此函数的摘要
%   此处显示详细说明
global hatA2;
global hatB2;
global hatC2;
global Nc;
global Np;
[F,Phi,Phi_Phi,BarR]=Gain(hatA2,hatB2,hatC2,Nc,Np);
E=Phi_Phi+BarR;
H=((F*x-Rs)'*Phi)';

M = [zeros(4,Nc*4);zeros(4,Nc*4)];
Me = zeros(4,Nc*4);
for k = 1:Nc
    Me(:,4*k-3:4*k)=hatC2*hatA2^(Nc-k)*hatB2;
end
gammae=rsk-hatC2*hatA2^(Nc)*x;

global nb2;
global alpha;
global lambda;

M(:,1:4) = [eye(4);-eye(4)];
gamma=[sqrt(nb2*((2-alpha)/lambda-x'*x))/nb2*ones(nb2,1);
sqrt(nb2*((2-alpha)/lambda-x'*x))/nb2*ones(nb2,1)];

options=optimset('LargeScale','off','MaxIter',300,...
        'algorithm','interior-point-convex','display','on');

[deltaU,fval]=quadprog(E,H,M,gamma,Me,gammae,[],[],[],options);
du = [deltaU(1);deltaU(2);deltaU(3);deltaU(4)];

end

