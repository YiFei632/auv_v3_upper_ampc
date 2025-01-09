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

Rs = zeros(Np*4,1);
for j = 1:Np
    Rs(4*j-3) = (pi/6)-(pi/6)*cos((pi/5)*tp(sig+j));
end
rsk=[(pi/6)-(3*pi/18)*cos((pi/5)*tp(sig+j));0;0;0];

% Calculate Adaptative
global hatTheta;
global hatA;
global hatB;
global du;
global hatx;
global na2;
global nb2;
global nc1;

% Update the estimate states(important!) and adaptive
x=[d_eta_fdb;eta_fdb(:,sig)];
if sig>1
    hatx=hatA*x+hatB*d_nu_fdb;
    hatTheta=Adaptative(x,hatx,d_nu_fdb,hatTheta);
    hatA=hatTheta(:,1:na2+nc1);
    hatB=hatTheta(:,na2+nc1+1:na2+nc1+nb2);
end
if sig>1
    d_nu_fdb=nu_fdb(:,sig)-nu_fdb(:,sig-1);
else
    d_nu_fdb=nu_fdb(:,sig);
end

du = MPC(x,Rs,rsk);


if sig>1
    nu_ref=nu_ref+du;
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
    nu_ref=du;
end

global nu_err;
global sum_nu_err;
global diff_nu_err;

nu_err(:,sig)=nu_ref-nu_fdb(:,sig);
sum_nu_err=sum_nu_err+nu_err(:,sig);
if sig>1
    diff_nu_err=nu_err(:,sig)-nu_err(:,sig-1);
else
    diff_nu_err=nu_err(:,sig);
end

global tau_ref;
tau_ref=PID_auv(nu_err(:,sig),sum_nu_err,diff_nu_err);
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