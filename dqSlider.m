function [dqOUT,qOUT] = dqSlider(t,qIN)
%% variables and thier derivatives

X1 = qIN(1);
Y1 = qIN(2);
phi1 = qIN(3);

dX1 = qIN(4);
dY1 = qIN(5);
dphi1 = qIN(6);


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
beta = 100;
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
B1G = B_mat(-q1);

PHIt1G = [(v1L.'*B_mat(q1)'*(rG - r1) - v1L.'*B1G*sLCG - v1L.'*RR.'*sLA1)
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
    gamma-beta^2*PHI];

ddq = LHS\RHS;
dq = qIN(4:6);

dqOUT = [dq;ddq];
qOUT = [X1;Y1;phi1;dq];

global MassEq FextOUT FkcOUT lOUT tInner PHI_OUT Fdamper;
% MassEq = [MassEq;LHS];
% FextOUT = [FextOUT;Fext]; 
% FkcOUT = [FkcOUT;f];
% lOUT = [lOUT; l];
tInner = [tInner; t];
PHI_OUT = [PHI_OUT, PHI];
% Fdamper = [Fdamper; c*dl];


end