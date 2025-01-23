function [tr,sig] = receive_callback(port,evt)
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
% 开始计时
tic;

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
global lower_torque;
global lower_delta_t;


recv_str = readline(port);
fdb = str2num(recv_str);
disp(fdb);
for i=2:4
    fdb(i)=(fdb(i)/180.0)*3.1415926;
end
disp(fdb);
sig = sig + 1;
if sig<=600
fdb_data = fdb(1:4);
% lower_torque(:,sig)=fdb(5:8);
% lower_delta_t(sig)=fdb(9);
if sig==1
    depth_err_init=fdb_data;
end
for i=1:4
    eta_fdb(i,sig)=fdb_data(i)-depth_err_init(i);
    if abs(eta_fdb(1,sig))>1
        eta_fdb(1,sig)=eta_fdb(1,sig-1);
    end
    if i>1 && abs(eta_fdb(i,sig))>5
        eta_fdb(i,sig)=eta_fdb(i,sig-1);
    end
    
    d_eta_fdb(i)=eta_fdb(i,sig)-eta_fdb_last(i);
    nu_fdb(i,sig)=d_eta_fdb(i)/Ts;
    eta_fdb_last(i)=eta_fdb(i,sig); %竟然忘了更新……简直是奇耻大辱
end
% nu_fdb(2:4,sig)=fdb(5:7);
% nu_fdb(1,sig)=d_eta_fdb(1) / Ts;
%经过查找发现stm32中roll和pitch的位置反了，在这里调换一下
% tmp=eta_fdb(2,sig);
% eta_fdb(2,sig)=eta_fdb(3,sig);
% eta_fdb(3,sig)=tmp;
% tmp=nu_fdb(2,sig);
% nu_fdb(2,sig)=nu_fdb(3,sig);
% nu_fdb(3,sig)=tmp;
% tmp=d_eta_fdb(2);
% d_eta_fdb(2)=d_eta_fdb(3);
% d_eta_fdb(3)=tmp;



disp(eta_fdb(:,sig)');
% eta_ref(1,sig) = 0.05-0.05*cos((pi/10)*tp(sig+1));
eta_ref(1,sig) = 0.15;

Rs1 = zeros(Np*4,1);
for j = 1:Np
    % Rs1(4*j-3) = 0.05-0.05*cos((pi/10)*tp(sig+j));
    Rs1(4*j-3) = 0.15;
end
% rsk1=[0.05-0.05*cos((pi/10)*tp(sig+j));0;0;0];
rsk1=[0.15;0;0;0];

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
    nu_ref(:,sig)=nu_ref(:,sig-1)+du1;
    if nu_ref(1,sig)>0.5
        nu_ref(1,sig)=0.5;
    elseif nu_ref(1,sig)<-0.1
        nu_ref(1,sig)=-0.1;
    end
    for j=2:4
        if nu_ref(j,sig)>0.5
            nu_ref(j,sig)=0.5;
        elseif nu_ref(j,sig)<-0.5
            nu_ref(j,sig)=-0.5;
        end
    end
else
    nu_ref(:,sig)=du1;
end
rs2 = nu_ref(:,sig);
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
if sig>1
    tau_ref(:,sig)=tau_ref(:,sig-1)+du2;
    if tau_ref(1,sig)>20
        tau_ref(1,sig)=20;
    elseif tau_ref(1,sig)<-20
        tau_ref(1,sig)=-20;
    end
    for j=2:4
        if tau_ref(j,sig)>3
            tau_ref(j,sig)=3;
        elseif tau_ref(j,sig)<-3
            tau_ref(j,sig)=-3;
        end
    end
else
    tau_ref(:,sig)=du2;
end
tau_ref(4,sig) = 0-tau_ref(4,sig);

tau_ref(1,sig) = 500*(eta_ref(1,sig) - eta_fdb(1,sig));

global torque_ref;
torque_ref(:,sig)=Torque(tau_ref(:,sig));
if sig>1
    if torque_ref(1,sig)>9
        torque_ref(1,sig)=9;
    elseif torque_ref(1,sig)<4 && torque_ref(1,sig)>=0
        torque_ref(1,sig)=4;
    elseif torque_ref(1,sig)<-18
        torque_ref(1,sig)=-18;
    elseif torque_ref(1,sig)>-4&& torque_ref(1,sig)<0
        torque_ref(1,sig)=-4;
    end
    % for j=3:4
    %     if torque_ref(j,sig)>36
    %         torque_ref(j,sig)=36;
    %     elseif torque_ref(j,sig)<-36
    %         torque_ref(j,sig)=-36;
    %     end
    % end
    if torque_ref(3,sig)>18
        torque_ref(3,sig)=18;
    elseif torque_ref(3,sig)<4 && torque_ref(3,sig)>=0
        torque_ref(3,sig)=4;
    elseif torque_ref(3,sig)<-9
        torque_ref(3,sig)=-9;
    elseif torque_ref(3,sig)>-4&& torque_ref(3,sig)<0
        torque_ref(3,sig)=-4;
    end

    if torque_ref(4,sig)>18
        torque_ref(4,sig)=18;
    elseif torque_ref(4,sig)<4 && torque_ref(4,sig)>=0
        torque_ref(4,sig)=4;
    elseif torque_ref(4,sig)<-9
        torque_ref(4,sig)=-9;
    elseif torque_ref(4,sig)>-4&& torque_ref(4,sig)<0
        torque_ref(4,sig)=-4;
    end
end

torque_ref(1,sig) = torque_ref(1,sig)*(-1);
% torque_ref(2,sig) = torque_ref(2,sig)*(-1);
% torque_ref(3,sig) = torque_ref(3,sig)*(-1);
% torque_ref(4,sig) = torque_ref(4,sig)*(-1);

disp(torque_ref(:,sig)');
transmit = torque_ref(:,sig);
trans = typecast(single(transmit),'uint8');
tr = uint8([119;trans;255]);
global serial_tx;
% write(port,tr);
for i=1:18
    write(serial_tx,tr(i),"uint8");
    pause(1/1000);
end
disp("Data Sent!");
else
    transmit = [0.0;0.0;0.0;0.0];
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
global delta_t;
if sig<=200
    delta_t(sig)=toc;
end
end