clear;
close all;
global MassEq FextOUT FkcOUT lOUT tInner PHI_OUT Fdamper;
FextOUT = 0;
FkcOUT = 0;
lOUT = 0;
MassEq = 0;% warning
tInner = 0;
PHI_OUT = zeros(2,1);%adding for Quarter
Fdamper = 0;

step = 0.001;
Tend = 1;
time = 0:step:Tend;
nstep = length(time);
VarIni = [  0;
            1; 
            -30*pi/180; 
            zeros(3,1) ];

tic;
% [Tout,Yout] = ode23(@dqSLA,time,VarIni);
% results = Yout;

VarIn_i = VarIni;
results = VarIni;
dresults = zeros(6,1);
lambda = zeros(2,1);
for i = 1:(nstep-1)
    [dVar_i,VarMOD] = dqSlider(time(i),VarIn_i);
%     VarIn_ip1 = VarIn_i + step*dVar_i;
    VarIn_ip1 = VarMOD + step*dVar_i(1:6);
    lambda = [lambda dVar_i(7:8)];
    results = [results VarIn_ip1];
    dresults = [dresults dVar_i(1:6)];
    VarIn_i = VarIn_ip1;
end
results = results';
dresults = dresults';
toc;
%% postprocessing
% figure(1);
% set(gcf,'position',[500,400,300,200]);
% plot(tInner(2:end),MassEq(2:end));
% title('Equivalent Mass (kg)','FontName','Times New Roman','FontSize',12);
% 
% figure(2);
% set(gcf,'position',[500,400,650,200]);
% plot(tInner,PHI_OUT(1,:));
% hold on;
% plot(tInner,PHI_OUT(2,:));
% plot(tInner,PHI_OUT(3,:));
% plot(tInner,PHI_OUT(4,:));
% plot(tInner,PHI_OUT(5,:));
% plot(tInner,PHI_OUT(6,:));
% plot(tInner,PHI_OUT(7,:));
% plot(tInner,PHI_OUT(8,:));
% title('Constraint Error (m)','FontName','Times New Roman','FontSize',12);
% legend('Rev1_1','Rev1_2','Rev2_1','Rev2_2',...
%     'Rev3_1','Rev3_2','Rev4_1','Rev4_2');
% 
% y2 = results(:,5);
% figure(3);
% set(gcf,'position',[500,400,300,200]);
% plot(y2,MassEq);
% 
% figure(4);
% set(gcf,'position',[500,400,300,200]);
% plot(time,-Fdamper);
% 
% figure(5);
% set(gcf,'position',[500,400,300,200]);
% plot(time,results(:,4));
% title('Position X2 (m)','FontName','Times New Roman','FontSize',12);
% 
% figure(6);
% set(gcf,'position',[500,400,300,200]);
% plot(time,results(:,5));
% title('Position Y2 (m)','FontName','Times New Roman','FontSize',12);
% 
% figure(7);
% set(gcf,'position',[500,400,300,200]);
% plot(time,results(:,6)*180/pi);
% title('Positon \phi2 (deg)','FontName','Times New Roman','FontSize',12);
% 
% figure(8);
% set(gcf,'position',[500,400,300,200]);
% plot(time,results(:,13));
% title('Velo. X2 (m/s)','FontName','Times New Roman','FontSize',12);
% 
% figure(9);
% set(gcf,'position',[500,400,300,200]);
% plot(time,results(:,14));
% title('Velo. Y2 (m/s)','FontName','Times New Roman','FontSize',12);
% 
% figure(10);
% set(gcf,'position',[500,400,300,200]);
% plot(time,results(:,15)*180/pi);
% title('Velo. \phi2 (deg/s)','FontName','Times New Roman','FontSize',12);
% 
% figure(11);
% set(gcf,'position',[500,400,300,200]);
% plot(time,dresults(:,13));
% title('Acc. X2 (m/s2)','FontName','Times New Roman','FontSize',12);
% xlabel('Time (s)','FontName','Times New Roman','FontSize',12);
% 
% figure(12);
% set(gcf,'position',[500,400,300,200]);
% plot(time,dresults(:,14));
% title('Acc. Y2(m/s2)','FontName','Times New Roman','FontSize',12);
% xlabel('Time (s)','FontName','Times New Roman','FontSize',12);
% 
% figure(13);
% set(gcf,'position',[500,400,300,200]);
% plot(time,dresults(:,15)*180/pi);
% title('Acc \phi2(deg/s2)','FontName','Times New Roman','FontSize',12);
% xlabel('Time (s)','FontName','Times New Roman','FontSize',12);

%% position
% figure(44);
% 
% xLO1 = -334/1000;%m
% yLO1 = 0;
% xLA1 = 334/1000;
% yLA1 = 0;
% xLA2 = -386/1000;
% yLA2 = -192/1000;
% xLB2 = -506/1000;
% yLB2 = 192/1000;
% xLB3 = 220/1000;
% yLB3 = 0/1000;
% xLC3 = -220/1000;
% yLC3 = 0/1000;
% 
% sLO1 = A_mat([0 0 30*pi/180])'*[xLO1;yLO1];
% sLA1 = A_mat([0 0 30*pi/180])'*[xLA1;yLA1];
% sLA2 = [xLA2;yLA2];
% sLB2 = [xLB2;yLB2];
% sLB3 = A_mat([0 0 30*pi/180])'*[xLB3;yLB3];
% sLC3 = A_mat([0 0 30*pi/180])'*[xLC3;yLC3];
% 
% i=1311;
% Var_i = results(i,:);
% X_LA = Var_i(1);
% Y_LA = Var_i(2);
% phi_LA = Var_i(3);
% X_Strut = Var_i(4);
% Y_Strut = Var_i(5);
% phi_Strut = Var_i(6);
% X_UA = Var_i(7);
% Y_UA = Var_i(8);
% phi_UA = Var_i(9);
% 
% r_LA = [X_LA;Y_LA];
% r_Strut = [X_Strut;Y_Strut];
% r_UA =[X_UA;Y_UA];
% 
% O_g = r_LA + A_mat([0 0 phi_LA])*sLO1;
% A1_g = r_LA + A_mat([0 0 phi_LA])*sLA1;
% A2_g = r_Strut + A_mat([0 0 phi_Strut])*sLA2;
% B2_g = r_Strut + A_mat([0 0 phi_Strut])*sLB2;
% B3_g = r_UA + A_mat([0 0 phi_UA])*sLB3;
% C3_g = r_UA + A_mat([0 0 phi_UA])*sLC3;
% 
% set(gcf,'position',[500,400,400,300]);
% plot([0 899 899]/1000,[-205 -205 550]/1000,'Color',[1 1 1]);grid on;
% hold on;
% plot([A1_g(1) O_g(1)],[A1_g(2) O_g(2)],'LineWidth',2);
% plot([A2_g(1) B2_g(1)],[A2_g(2) B2_g(2)],'LineWidth',2);
% plot([B3_g(1) C3_g(1)],[B3_g(2) C3_g(2)],'LineWidth',2);
% hold off;
% axis equal;
% xlabel('X (m)','FontName','Times New Roman','FontSize',12);
% ylabel('Y (m)','FontName','Times New Roman','FontSize',12);
% drawnow;
% 
% figure(55);
% i=1816;
% Var_i = results(i,:);
% X_LA = Var_i(1);
% Y_LA = Var_i(2);
% phi_LA = Var_i(3);
% X_Strut = Var_i(4);
% Y_Strut = Var_i(5);
% phi_Strut = Var_i(6);
% X_UA = Var_i(7);
% Y_UA = Var_i(8);
% phi_UA = Var_i(9);
% 
% r_LA = [X_LA;Y_LA];
% r_Strut = [X_Strut;Y_Strut];
% r_UA =[X_UA;Y_UA];
% 
% O_g = r_LA + A_mat([0 0 phi_LA])*sLO1;
% A1_g = r_LA + A_mat([0 0 phi_LA])*sLA1;
% A2_g = r_Strut + A_mat([0 0 phi_Strut])*sLA2;
% B2_g = r_Strut + A_mat([0 0 phi_Strut])*sLB2;
% B3_g = r_UA + A_mat([0 0 phi_UA])*sLB3;
% C3_g = r_UA + A_mat([0 0 phi_UA])*sLC3;
% 
% set(gcf,'position',[500,400,400,300]);
% plot([0 899 899]/1000,[-205 -205 550]/1000,'Color',[1 1 1]);grid on;
% hold on;
% plot([A1_g(1) O_g(1)],[A1_g(2) O_g(2)],'LineWidth',2);
% plot([A2_g(1) B2_g(1)],[A2_g(2) B2_g(2)],'LineWidth',2);
% plot([B3_g(1) C3_g(1)],[B3_g(2) C3_g(2)],'LineWidth',2);
% hold off;
% axis equal;
% xlabel('X (m)','FontName','Times New Roman','FontSize',12);
% ylabel('Y (m)','FontName','Times New Roman','FontSize',12);
% drawnow;



%% animation
figure(66);

xLA1 = 0/1000;
yLA1 = 0;
xLB1 = 100/1000;
yLB1 = 0/1000;

XC = sqrt(3) - 0.1*cos(30*pi/180);
YC = 0.05;
XD = sqrt(3);
YD = 0;

sLA1 = [xLA1;yLA1];
sLB1 = [xLB1;yLB1];
sLCG = [XC;YC];

v1L = [xLB1-xLA1;yLB1-yLA1];
vGL = [XD-XC;YD-YC];

for i=1:2:(nstep-1)
    Var_i = results(i,:);
    X_Sli = Var_i(1);
    Y_Sli = Var_i(2);
    phi_Sli = Var_i(3);
    
    r_Sli = [X_Sli;Y_Sli];
    
    A1_g = r_Sli + A_mat([0 0 phi_Sli])*sLA1;
    B1_g = r_Sli + A_mat([0 0 phi_Sli])*sLB1;
    
    plot([-0.5 1 1],[0 0 1.5],'Color',[1 1 1]);grid on;
    hold on;
    plot([A1_g(1) B1_g(1)],[A1_g(2) B1_g(2)],'LineWidth',4);
    plot([0 sqrt(3)],[1 0],'LineWidth',1);
    hold off;
    axis equal;
    drawnow;
end

