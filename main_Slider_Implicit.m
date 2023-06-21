clear;
close all;
global step;

step = 0.01;
Tend = 1;
time = 0:step:Tend;
nstep = length(time);
VarIni = [  0;
            1; 
            -30*pi/180; 
            zeros(3,1) ];

tic;

VarIn_i = VarIni;
results = VarIni;
dresults = zeros(6,1);
lambda = zeros(2,1);
lambda_i = lambda;
global x_i;
global PHI_look;
PHI_look = [0;0];

for i = 1:(nstep-1)
    x_i = [VarIn_i;lambda_i];
    [Var_ip1,lambda_ip1] = dqSliderImplicit(time(i),[VarIn_i;lambda_i]);
    lambda = [lambda lambda_ip1];
    results = [results Var_ip1];
    VarIn_i = Var_ip1;
    lambda_i = lambda_ip1;
end
results = results';
dresults = dresults';
toc;

PHIt1G_log = [0;0];
for i = 1:(nstep-1)
    results_i = results(i,:);
    X1 = results_i(1);
    Y1 = results_i(2);
    phi1 = results_i(3);
    %% parameters
    m1 = 10.232;%kg
    J1 = 3.82e-1;%kgm^2
    g = 9.8;%m/s2
    theta = 30*pi/180;%rad

    xLA1 = 0;
    yLA1 = 0;
    xLB1 = 100/1000;%m
    yLB1 = 0/1000;

    XC = (sqrt(3) - 0.1*cos(30*pi/180));
    YC = 0.05;
    XD = sqrt(3);
    YD = 0;

    % adding for stablization method
    % beta = 200;
    %% external forces

    %% symbolic calculation
    % mass matrix
    M1 = [m1 m1 J1];

    % variable vectors
    q1 = [X1;Y1;phi1];
    r1 = [X1;Y1];
%     dr1 = [dX1;dY1];
    qG = [0;0;0];
    rG = [0;0];

    % point position in local frames
    sLA1 = [xLA1;yLA1];
    sLCG = [XC;YC];

    v1L = [xLB1-xLA1;yLB1-yLA1];
    vGL = [XD-XC;YD-YC];

    % adding PHI
    RR = [0 -1;1 0];
    B1G = (B_mat(q1).'*B_mat(qG));

    PHIt1G = [(v1L.'*B_mat(q1)*(rG - r1) - v1L.'*B1G*sLCG - v1L.'*RR.'*sLA1)
    -v1L.'*B1G*vGL];  
    PHIt1G_log = [PHIt1G_log PHIt1G];
end

%% postprocessing



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

for i=1:1:(nstep-1)
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
    plot([0 sqrt(3)],[1 0],'LineWidth',2);
    hold off;
    axis equal;
    drawnow;
end

