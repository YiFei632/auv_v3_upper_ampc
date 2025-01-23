%This function is to calculate predictive matrices F and Phi

function [F,Phi,Phi_Phi,BarR]=Gain(A,B,C,Nc,Np)

[m1,n1]=size(C);
[n1,n_in]=size(B);

n=n1+m1;
h(1:m1,:)=C;
F(1:m1,:)=C*A;
for kk=2:Np
    h((kk-1)*m1+1:kk*m1,:)=h((kk-2)*m1+1:(kk-1)*m1,:)*A;
    F((kk-1)*m1+1:kk*m1,:)=F((kk-2)*m1+1:(kk-1)*m1,:)*A;
end
v=h*B;
Phi=zeros(Np*m1,Nc*n_in);       %declare the dimention of Phi
Phi(:,1:n_in)=v;             %first n_in column of Phi
for i=2:Nc
    Phi(:,(i-1)*n_in+1:i*n_in)=[zeros((i-1)*m1,n_in);v(1:(Np-i+1)*m1,1:n_in)];  %toeplitz matrix
end

Phi_Phi=Phi'*Phi;
[nPhi,mPhi]=size(Phi_Phi);
diagR=[0*ones(nPhi/4,1);50*ones(nPhi/4,1);150*ones(nPhi/4,1);50*ones(nPhi/4,1)];
BarR=0.1*diag(diagR);

