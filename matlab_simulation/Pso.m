clc
%% 参数设置
w_max = 0.6;      % 惯性因子 
w_min=0.4;
c1 = 2;       % 加速常数
c2 = 2;       % 加速常数

Dim = 7;            % 维数
SwarmSize = 30;    % 粒子群规模
ObjFun = @simfun;  % 待优化函数句柄

MaxIter = 50;      % 最大迭代次数  
MinFit = 0.2;       % 最小适应值 

Vmax = [2 0.5 0.02 0.092 0.0006 1.8 0.006];
Vmin = [-2 -0.5 -0.02 -0.092 -0.0006 -1.8 -0.006];
Ub = [35 3.5 0.35 1.63 0.0117 35 0.117];  % 解向量的最大限制
Lb = [25 14.5 0.25 1.16 0.0082 25 0.082];     % 解向量的最小限制
% Swarm=[30 1 0.3 1.4 0.01 20 0.1];
% Swarm=[30 7 0.3 1.4 0.01 28 0.1];

%% 
    Range_v = ones(SwarmSize,1)*(Vmax-Vmin);
    Range_b = ones(SwarmSize,1)*(Ub-Lb);
    Swarm = rand(SwarmSize,Dim).*Range_b + ones(SwarmSize,1)*Lb;      % 初始化粒子群（粒子群位置矩阵，每行表示一个解）
    VStep = rand(SwarmSize,Dim).*Range_v + ones(SwarmSize,1)*Vmin;              % 初始化速度（粒子群速度矩阵）
    fSwarm = zeros(SwarmSize,1);    % 预设空矩阵，存放适应值
for i=1:SwarmSize
    fSwarm(i,:) = feval(ObjFun,Swarm(i,:));    % 粒子群的适应值 （以粒子群位置的第i行为输入，求函数值，对应输出给适应值）
end

%% 记录最佳
[bestf, bestindex]=min(fSwarm); % 求得适应值中的最小适应值其所在的序列
zbest=Swarm(bestindex,:);   % 全局最佳  （所在序列的对应的解矩阵序列，全局最佳解）
gbest=Swarm;                % 个体最佳
fgbest=fSwarm;              % 个体最佳适应值
fzbest=bestf;               % 全局最佳适应值

%% 迭代
iter = 0;
y_fitness = zeros(1,MaxIter);   %  1行MaxIter列矩阵，存放MaxIter个最优值的空间矩阵，（预先产生6个空矩阵） 
eKp_rol = zeros(1,MaxIter);     % 存放x的空间    
eKp_yaw = zeros(1,MaxIter);
iKp_rol = zeros(1,MaxIter);
iKi_rol = zeros(1,MaxIter);
iKd_rol= zeros(1,MaxIter);
iKp_yaw = zeros(1,MaxIter);
iKd_yaw= zeros(1,MaxIter);
while( (iter < MaxIter) && (fzbest > MinFit) )
    w=(((w_min-w_max)*iter)/MaxIter)+w_max
    for j=1:SwarmSize
        % 速度更新
        VStep(j,:) = w*VStep(j,:) + c1*rand*(gbest(j,:) - Swarm(j,:)) + c2*rand*(zbest - Swarm(j,:));
        for n=1:Dim
        if VStep(j,n)>Vmax(n), VStep(j,n)=Vmax(n); end
        if VStep(j,n)<Vmin(n), VStep(j,n)=Vmin(n); end
        end
        % 位置更新
        Swarm(j,:)=Swarm(j,:)+VStep(j,:);
        for k=1:Dim
            if Swarm(j,k)>Ub(k), Swarm(j,k)=Ub(k); end
            if Swarm(j,k)<Lb(k), Swarm(j,k)=Lb(k); end
        end
        % 适应值
        fSwarm(j,:) = feval(ObjFun,Swarm(j,:));
        % 个体最优更新     
        if fSwarm(j) < fgbest(j)   % 如果当前函数值比个体最优值小
            gbest(j,:) = Swarm(j,:);  % 个体最优解更新
            fgbest(j) = fSwarm(j);    % 个体最优值更新
        end
        % 群体最优更新
        if fSwarm(j) < fzbest    % 如果当前函数值比群体最优值小
            zbest = Swarm(j,:);   % 群体最优解更新
            fzbest = fSwarm(j);   %  群体最优值更新
        end
    end 
%     w=wmax-(wmax-wmin)*iter/MaxIter;
    iter = iter+1                      % 迭代次数更新
    y_fitness(1,iter) = fzbest;         % 为绘图做准备
    eKp_rol(1,iter) = zbest(1);    % 存放x的空间    
    eKp_yaw(1,iter) = zbest(2);
    iKp_rol(1,iter) = zbest(3);
    iKi_rol(1,iter) = zbest(4);
    iKd_rol(1,iter)= zbest(5);
    iKp_yaw(1,iter) = zbest(6);
    iKd_yaw(1,iter) = zbest(7);
end

%% 绘图输出
figure(1)      % 绘制性能指标ITAE的变化曲线
plot(y_fitness,'LineWidth',3)
title('Optimal individual fitness','fontsize',22);
xlabel('Iterations','fontsize',22);ylabel('Fitness value','fontsize',22);
set(gca,'Fontsize',22);

subplot(1,3,1)
plot(iKd_yaw,'r','LineWidth',1.1)
title('Parameter optimization curve','fontsize',14);
xlabel('Iterations','fontsize',14);ylabel('Parameter Value','fontsize',14);
set(gca,'Fontsize',14);
legendText = legend('iKdyaw','FontSize', 14,'FontName','宋体');

subplot(1,3,2)
plot(iKd_rol,'y','LineWidth',1.1)
title('Parameter optimization curve','fontsize',14);
xlabel('Iterations','fontsize',14);ylabel('Parameter Value','fontsize',14);
set(gca,'Fontsize',14);
legendText = legend('iKdrol','FontSize', 14,'FontName','宋体');

plot(eKp_yaw)
title('Parameter optimization curve','fontsize',8);
xlabel('Iterations','fontsize',8);ylabel('Parameter Value','fontsize',8);
set(gca,'Fontsize',8);
legendText = legend('eKpyaw','FontSize', 8,'FontName','宋体');

subplot(1,3,3)
plot(iKd_rol,'k','LineWidth',1.1)
title('Parameter optimization curve','fontsize',8);
xlabel('Iterations','fontsize',8);ylabel('Parameter Value','fontsize',8);
set(gca,'Fontsize',8);
legendText = legend('iKdrol','FontSize', 8,'FontName','宋体');

plot(iKp_yaw,'k','LineWidth',1.1)
title('Parameter optimization curve','fontsize',8);
xlabel('Iterations','fontsize',8);ylabel('Parameter Value','fontsize',8);
set(gca,'Fontsize',8);
legendText = legend('iKpyaw','FontSize', 8,'FontName','宋体');

subplot(1,2,1)
plot(iKd_yaw,'r','LineWidth',1.1)
title('Parameter optimization curve','fontsize',8);
xlabel('Iterations','fontsize',8);ylabel('Parameter Value','fontsize',8);
set(gca,'Fontsize',8);
legendText = legend('iKdyaw','FontSize', 8,'FontName','宋体');

subplot(1,2,2)
plot(eKp_rol,'k','LineWidth',1.1)
title('Parameter optimization curve','fontsize',8);
xlabel('Iterations','fontsize',8);ylabel('Parameter Value','fontsize',8);
set(gca,'Fontsize',8);
legendText = legend('eKprol','FontSize', 8,'FontName','宋体');
% figure(3)    %绘制量化因子参数变化曲线
% plot(eKp_rol)
% hold on
% plot(eKp_yaw,'--r')
% % title('优化曲线','fontsize',18);
% xlabel('迭代次数','fontsize',18);ylabel('参数值','fontsize',18);
% set(gca,'Fontsize',18);
% legend('Ke','Kec',1);

%% 仿真输出绘图
time=ScopeData_pid_pit.time;
step=ScopeData_pid_pit.signals(1).values(:,1);
pit_pid=ScopeData_pid_pit.signals(3).values(:,1);
pit_fuzzypid=ScopeData_fuzzypid_pit.signals(3).values(:,1);
pit_psofuzzypid=ScopeData_psofuzzypid_pit.signals(3).values(:,1);

plot(time,step,'-','color','y','Linewidth', 1.1,'MarkerSize',8);
hold on 
plot(time,pit_pid,'-','color','k','Linewidth', 1.1,'MarkerSize',8);
hold on 
plot(time,pit_fuzzypid,'-','color','b','Linewidth', 1.1,'MarkerSize',8);
hold on 
plot(time,pit_psofuzzypid,'-','color','r','Linewidth', 1.1,'MarkerSize',8);
legendText = legend('step','pitpid','pitfuzzypid','pitpsofuzzypid','FontSize', 8,'FontName','宋体');
xlabel('Time/s','fontsize',8);ylabel('Angle/°','fontsize',8);
set(gca,'Fontsize',8);
xlim([4.95 6])
ylim([0 3.5])
%%%%%%%%%%%%%%%%%%%%%%%%5
time=ScopeData_alldis.time;
dis=ScopeData_alldis.signals(1).values(:,1);
rol_pid_dis=ScopeData_alldis.signals(2).values(:,1);
rol_fuzzypid_dis=ScopeData_alldis.signals(3).values(:,1);
rol_psofuzzypid_dis=ScopeData_alldis.signals(4).values(:,1);
MPC=ScopeData_alldis.signals(5).values(:,1);
plot(time,dis,'-','color','y','Linewidth', 1.1,'MarkerSize',8);
hold on 
plot(time,rol_pid_dis,'-','color','k','Linewidth', 2,'MarkerSize',8);
hold on 
plot(time,rol_fuzzypid_dis,'-','color','b','Linewidth', 1.1,'MarkerSize',8);
hold on 
plot(time,rol_psofuzzypid_dis,'-','color','r','Linewidth', 1.1,'MarkerSize',8);
hold on 
plot(time,MPC,'--','color','g','Linewidth', 1.1,'MarkerSize',8);
legendText = legend('dis','rolpiddis','rolfuzzypiddis','rolpsofuzzypiddis','MPC','FontSize', 8,'FontName','宋体');
xlabel('Time/s','fontsize',8);ylabel('Angle/°','fontsize',8);
set(gca,'Fontsize',8);
title('rol angle tracking curve','fontsize',8);
xlim([4.9 7.1])
ylim([-3.5 3.5])

%%%%%%%%%%%%%%%%%%%%55
time=pid_dist.time;
dis=pid_dist.signals(1).values(:,1);
rol_pid_dise=pid_dist.signals(2).values(:,1);
rol_fuzzypid_dise=pid_dist.signals(3).values(:,1);
rol_psofuzzypid_dise=pid_dist.signals(4).values(:,1);
MPC=pid_dist.signals(5).values(:,1);

plot(time,dis,'-','color','y','Linewidth', 1.1,'MarkerSize',8);
hold on 
plot(time,rol_pid_dise,'-','color','k','Linewidth', 2.1,'MarkerSize',8);
hold on 
plot(time,rol_fuzzypid_dise,'-','color','b','Linewidth', 1.1,'MarkerSize',8);
hold on 
plot(time,rol_psofuzzypid_dise,'-','color','r','Linewidth', 1.1,'MarkerSize',8);
hold on 
plot(time,MPC,'--','color','g','Linewidth', 1.1,'MarkerSize',8);
legendText = legend('dis','rolpiddise','rolfuzzypiddise','rolpsofuzzypiddise','MPC','FontSize', 8,'FontName','宋体');
xlabel('Time/s','fontsize',8);ylabel('Angle/°','fontsize',8);
set(gca,'Fontsize',8);
% title('Interference error curve','fontsize',8);
title('Interference error curve','fontsize',8);
xlim([4.9 7.3])
ylim([-3.3 3.4])
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
time=ScopeData_alldis.time;
dis=ScopeData_alldis.signals(1).values(:,1);
yaw_pid=ScopeData_alldis.signals(2).values(:,1);
yaw_fuzzypid=ScopeData_alldis.signals(3).values(:,1);
yaw_apsofuzzypid=ScopeData_alldis.signals(4).values(:,1);

plot(time,dis,'-','color','k','Linewidth', 1.1,'MarkerSize',8);
hold on 
plot(time,yaw_pid,'-','color','g','Linewidth', 2,'MarkerSize',8);
hold on 
plot(time,yaw_fuzzypid,'-','color','b','Linewidth', 1.1,'MarkerSize',8);
hold on 
plot(time,yaw_apsofuzzypid,'-','color','r','Linewidth', 1.1,'MarkerSize',8);
legendText = legend('step','yawpid','yawfuzzypid','yawapsofuzzypid','FontSize', 8,'FontName','宋体');
xlabel('Time/s','fontsize',8);ylabel('Angle/°','fontsize',8);
set(gca,'Fontsize',8);
% title('Interference error curve','fontsize',8);
title('Interference tracking curve','fontsize',8);
xlim([4.7 12])
ylim([-0.1 3.2])
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%mse1380 =6.8 1200=6
timenew=time(1000:1380);
disnew=dis(1000:1380);
rol_pid_disenew=rol_pid_dise(1000:1380);
rol_fuzzypid_disenew=rol_fuzzypid_dise(1000:1380);
rol_psofuzzypid_disenew=rol_psofuzzypid_dise(1000:1380);
MPCNEW=MPC(1000:1380);

mse_value_pid = mean(( rol_pid_disenew).^2)
mse_value_pid_fuzzy = mean((rol_fuzzypid_disenew).^2)
mse_value_pid_fuzzypso = mean((rol_psofuzzypid_disenew).^2)
mse_mpc=mean((MPCNEW).^2)