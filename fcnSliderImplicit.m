function FcnOUT = fcnSliderImplicit(xIN)
%% variables and thier derivatives

X1 = xIN(1);
Y1 = xIN(2);
phi1 = xIN(3);

dX1 = xIN(4);
dY1 = xIN(5);
dphi1 = xIN(6);




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
dr1 = [dX1;dY1];
qG = [0;0;0];
rG = [0;0];

% point position in local frames
sLA1 = [xLA1;yLA1];
sLCG = [XC;YC];

v1L = [xLB1-xLA1;yLB1-yLA1];
vGL = [XD-XC;YD-YC];

% adding PHI
RR = [0 -1;1 0];
B1G = B_mat(qG-q1);

PHIt1G = [(v1L.'*B_mat(q1)*(rG - r1) - v1L.'*B1G*sLCG - v1L.'*RR.'*sLA1)
    -v1L.'*B1G*vGL];  


% Jacobian
A1G = (A_mat(q1).'*A_mat(qG));

PHIt1G_q4 = [-v1L.'*B_mat(q1).',-v1L.'*A_mat(q1).'*(-r1)-v1L.'*A1G*sLCG;
            0,0,-v1L.'*A1G*vGL];
% gamma
gamma_t1G = -[  v1L.'*(Bij_mat(q1,qG)*sLCG*(-dphi1)^2 - B_mat(q1).'*(-r1)*dphi1^2 - 2*A_mat(q1).'*(-dr1)*dphi1)
                0];

% external force
Q_g = [0;-m1*g;0];
% assemble
QA = Q_g;
M = diag(M1);

PHI_q = PHIt1G_q4;
gamma = gamma_t1G;
% adding 
PHI = PHIt1G;

%% LHS and RHS
LHS = [ M,      PHI_q'
        PHI_q,  zeros(2,2)];
RHS = [QA
    gamma];

ddqANDlam = LHS\RHS;
ddq = ddqANDlam(1:3);
lam = ddqANDlam(4:5);

dxdt = [xIN(4:6)
        ddq];
xIN(7:8) = lam;
global x_i;
global step;
global PHI_look;
PHI_look = [PHI_look PHI];
% FcnOUT = [-xIN(1:6)+x_i(1:6)+step*dxdt
%             PHI_q*[dX1;dY1;dphi1]
%             PHI];
FcnOUT = [-xIN(1:6)+x_i(1:6)+step*dxdt
            PHI_q*[dX1;dY1;dphi1]];

