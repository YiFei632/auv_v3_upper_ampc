clc;
clear;
close all;
%% Initialize Variants
global serial_rx;
global serial_tx;

global eta_fdb; %pos feedback
global eta_fdb_last; %last pos feedback(calculate speed feedback)
global eta_ref; %pos reference
global nu_ref; %speed reference
global nu_fdb; %speed feedback
global tau_ref; %foece reference
global torque_ref; %经过推力分解计算之后的结果
global d_eta_fdb;
global d_nu_fdb;
global depth_err_init; %初始状态下稳定后深度的误差，这个作为初始值来进行计算
d_eta_fdb = zeros(4,1);
d_nu_fdb = zeros(4,1);
depth_err_init=zeros(4,1);


global steps;
global Ts;
global Np;
global Nc;
steps = 200;
Ts = 0.1;
Np = 10;
Nc = 10;

eta_fdb = zeros(4,steps);
eta_fdb_last = zeros(4,1);
eta_ref = zeros(4,steps);
nu_ref = zeros(4,1);
nu_fdb = zeros(4,steps);
tau_ref = zeros(4,1);
torque_ref = zeros(4,1);

global hatA;
global hatB;
global hatC;
global hatTheta;
global hatx;
hatx = zeros(8,1);
global lambda;
global alpha;
alpha = 0.1;
lambda = 0.2;

global na1;
global na2;
global nb1;
global nb2;
global nc1;
global nc2;

global n_x;
global m_u;
global l_y;

J = eta2J_auv(eta_fdb(:,1));
Ad = zeros(4);
Bd = J;
Cd = eye(4);
n_x = 4;
m_u = 4;
l_y = 4;
[na1,na2]=size(Ad);
[nb1,nb2]=size(Bd);
[nc1,nc2]=size(Cd);

hatA = [Ad,zeros(n_x,l_y);Cd*Ad,eye(l_y)];
hatB = [Bd;Cd*Bd];
hatC = [zeros(l_y,n_x),eye(l_y)];
hatTheta = [hatA,hatB];

global nu_err;
global sum_nu_err;
global diff_nu_err;

global Kp;
global Ki;
global Kd;

Kp=diag([12,1.5,2,2]);
Ki=diag([1,0.4,0.4,0.4]);
Kd=diag([0.1,2.5,2.5,2.5]);

nu_err = zeros(4,steps);
sum_nu_err = zeros(4,1);
diff_nu_err = zeros(4,1);

global tp;
tp=zeros(1,steps+Np);
for i = 1:steps+Np+1
    tp(i) = (i - 1) * Ts;
end

global du;
du = zeros(4,1);

%% Start Program

serial_rx = serialport("COM13",9600);
serial_tx = serialport("COM11",115200);
global sig; %用于指示迭代次数
sig = 0;
flush(serial_rx);

configureCallback(serial_rx,"terminator",@receive_callback);