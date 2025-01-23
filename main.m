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
steps = 600;
Ts = 0.1;
Np = 10;
Nc = 10;

eta_fdb = zeros(4,steps);
eta_fdb_last = zeros(4,1);
eta_ref = zeros(4,steps);
nu_ref = zeros(4,steps);
nu_fdb = zeros(4,steps);
tau_ref = zeros(4,steps);
torque_ref = zeros(4,steps);

global A1;
global B1;
global C1;
global Ad1;
global Bd1;
global Cd1;

global hatA2;
global hatB2;
global hatC2;
global hatTheta2;
global hatx2;
hatx2 = zeros(8,1);
global lambda;
global alpha;
alpha = 0.01;
lambda = 0.02;

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
Ad1 = zeros(4);
Bd1 = J;
Cd1 = eye(4);
n_x = 4;
m_u = 4;
l_y = 4;
[na1,na2]=size(Ad1);
[nb1,nb2]=size(Bd1);
[nc1,nc2]=size(Cd1);

A1 = [Ad1,zeros(n_x,l_y);Cd1*Ad1,eye(l_y)];
B1 = [Bd1;Cd1*Bd1];
C1 = [zeros(l_y,n_x),eye(l_y)];

M = diag([28.85,0.744,0.888,0.866]);%mass matrix

C = m2c_auv(M,nu_fdb(:,1));
D = diag([-2.12,-0.18,-0.24,-0.19]);

Ac2 = -((C + D) / M);
Bc2 = eye(4) / M;
Cc2 = eye(4);
Dc2 = zeros(4);
sysc2 = ss(Ac2,Bc2,Cc2,Dc2);
sysd2 = c2d(sysc2,Ts);
Ad2 = sysd2.A;
Bd2 = sysd2.B;
Cd2 = sysd2.C;

hatA2 = [Ad2,zeros(n_x,l_y);Cd2*Ad2,eye(l_y)];
hatB2 = [Bd2;Cd2*Bd2];
hatC2 = [zeros(l_y,n_x),eye(l_y)];
hatTheta2 = [hatA2,hatB2];

% global nu_err;
% global sum_nu_err;
% global diff_nu_err;
% 
% global Kp;
% global Ki;
% global Kd;
% 
% Kp=diag([12,1.5,2,2]);
% Ki=diag([1,0.4,0.4,0.4]);
% Kd=diag([0.1,2.5,2.5,2.5]);
% 
% nu_err = zeros(4,steps);
% sum_nu_err = zeros(4,1);
% diff_nu_err = zeros(4,1);

global tp;
global t;
t=zeros(1,steps);
tp=zeros(1,steps+Np);
for i = 1:steps+Np+1
    tp(i) = (i - 1) * Ts;
    t(i) = (i - 1) * Ts;
end

global du;
du = zeros(4,1);

% 为了检测一下耗时，所以设置一个时间差分量
global delta_t;
delta_t=zeros(1,steps);

global lower_torque;
global lower_delta_t;
lower_torque=zeros(4,steps);
lower_delta_t=zeros(1,steps);
%% Start Program

serial_rx = serialport("COM8",9600);
serial_tx = serialport("COM9",115200);
global sig; %用于指示迭代次数
sig = 0;
flush(serial_rx);

configureCallback(serial_rx,"terminator",@receive_callback);