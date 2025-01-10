function [tr,sig] = receive_callback(port,evt)
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
disp("Data Received!");
global sig;
global eta_fdb;
global eta_fdb_last;
global depth_err_init;
global eta_ref;
global nu_ref;
global nu_fdb;
global Ts;
global Np;
global tp;
global d_eta_fdb;
global d_nu_fdb;

recv_str = readline(port);
fdb_data = str2num(recv_str);
sig = sig + 1;
if sig<=200
if sig==1
    depth_err_init(1)=fdb_data(1);
end
for i=1:4
    eta_fdb(i,sig)=fdb_data(i)-depth_err_init(i);
    nu_fdb(i,sig)=(eta_fdb(i,sig)-eta_fdb_last(i))/Ts;
    d_eta_fdb(i)=eta_fdb(i,sig)-eta_fdb_last(i);
end

disp(eta_fdb(:,sig)');
eta_ref(1,sig) = (pi/6)-(pi/6)*cos((pi/5)*tp(sig+1));

Rs1 = zeros(Np*4,1);
for j = 1:Np
    Rs1(4*j-3) = (pi/6)-(pi/6)*cos((pi/5)*tp(sig+j));
end
rsk1=[(pi/6)-(3*pi/18)*cos((pi/5)*tp(sig+j));0;0;0];

% Calculate Adaptative
global hatTheta2;
global hatA2;
global hatB2;
global du1;
global du2;
global hatx2;
global na2;
global nb2;
global nc1;
global Ad1;
global Bd1;
global Cd1;

global A1;
global B1;
global C1;
% Calculate the First RTMPC
global n_x;
global m_u;
global l_y;
x1=[d_eta_fdb;eta_fdb(:,sig)];
J = eta2J_auv(eta_fdb(:,sig));
Bd1 = J;
A1 = [Ad1,zeros(n_x,l_y);Cd1*Ad1,eye(l_y)];
B1 = [Bd1;Cd1*Bd1];

du1 = MPC(x1,Rs1,rsk1);
% 以上是第一个MPC，接下来是第二个AMPC

if sig>1
    nu_ref=nu_ref+du1;
    if nu_ref(1)>1
        nu_ref(1)=1;
    elseif nu_ref(1)<-1
        nu_ref(1)=-1;
    end
    for j=2:4
        if nu_ref(j)>0.5
            nu_ref(j)=0.5;
        elseif nu_ref(j)<-0.5
            nu_ref(j)=-0.5;
        end
    end
else
    nu_ref=du1;
end
rs2 = nu_ref;
Rs2 = zeros(Np*4,1);
Rs2(1:4)=rs2;
global deltaU1;
for j = 2:Np
    Rs2(4*j-3) = Rs2(4*(j-1)-3)+deltaU1(4*j-3);
    Rs2(4*j-2) = Rs2(4*(j-1)-2)+deltaU1(4*j-2);
    Rs2(4*j-1) = Rs2(4*(j-1)-1)+deltaU1(4*j-1);
    Rs2(4*j) = Rs2(4*(j-1))+deltaU1(4*j);
end
rsk2=[Rs2(37);Rs2(38);Rs2(39);Rs2(40)];
x2=[d_nu_fdb;nu_fdb(:,sig)];

% Update the estimate states(important!) and adaptive
% 待修改
if sig>1
    hatx2=hatA2*x2+hatB2*d_nu_fdb;
    hatTheta2=Adaptative(x2,hatx2,d_nu_fdb,hatTheta2);
    hatA2=hatTheta2(:,1:na2+nc1);
    hatB2=hatTheta2(:,na2+nc1+1:na2+nc1+nb2);
end
if sig>1
    d_nu_fdb=nu_fdb(:,sig)-nu_fdb(:,sig-1);
else
    d_nu_fdb=nu_fdb(:,sig);
end

du2=AMPC(x2,Rs2,rsk2);
global tau_ref;
tau_ref=tau_ref+du2;


global torque_ref;
torque_ref=Torque(tau_ref);
disp(torque_ref');
transmit = torque_ref;
trans = typecast(single(transmit),'uint8');
tr = uint8([119;trans;255]);
global serial_tx;
% write(port,tr);
for i=1:18
    write(serial_tx,tr(i),"uint8");
    pause(1/1000);
end
disp("Data Sent!");
end


flush(port);

end