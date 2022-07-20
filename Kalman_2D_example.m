clear
close all
clc

% load X_example.mat
% load Z_example.mat


A = [1,1;0,1];%位置状态矩阵
H = [1,0;0,1];%速度状态矩阵
Q = [0.1,0;0,0.1];
R = [1,0;0,1];
I = [1,0;0,1];

X(1,1) = 0;%初始位置
X(2,1) = 1;%初始速度

P{1} = [1,0;0,1];%矫正协方差初始值
Xk_hat = [0;1];%后验估计初始值

W = normrnd(0,0.316,2,30);%随机生成位置和速度过程误差
V = normrnd(0,1,2,30);%随机生成位置和速度测量误差

for i = 1: 30
    X(:,i+1) = A * X(:,i) + W(i);%生成实际位置和速度
    Z(:,i+1) = H * X(:,i) + V(i);%生成测量位置和速度
end

% X = X_example;
% Z = Z_example;

for i = 1 : 30
    X_hat_prior(:,i+1) = A * Xk_hat(:,i);%先验估计位置和速度
    P_prior{i+1} = A * P{i} * A' + Q;%先验协方差
    K{i+1} =  P_prior{i+1} * H' * (H * P_prior{i+1} * H' + R)^(-1);%Kalman增益系数
    Xk_hat(:,i+1) = X_hat_prior(:,i+1) + K{i+1} * (Z(:,i+1) - H * X_hat_prior(:,i+1));%后验估计值
    P{i+1} = (I - K{i+1} * H) * P_prior{i+1};%更新协方差矩阵
end

figure(1)
plot(X(2,:))
hold on;
plot(Z(2,:))
hold on;
plot(X_hat_prior(2,:))
hold on;
plot(Xk_hat(2,:))
legend('实际速度','测量速度','先验估计速度','后验估计速度')