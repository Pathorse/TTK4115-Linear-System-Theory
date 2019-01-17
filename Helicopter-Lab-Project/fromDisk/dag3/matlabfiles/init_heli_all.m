% FOR HELICOPTER NR 3-10
% This file contains the initialization for the helicopter assignment in
% the course TTK4115. Run this file before you execute QuaRC_ -> Build 
% to build the file heli_q8.mdl.

% Oppdatert høsten 2006 av Jostein Bakkeheim
% Oppdatert høsten 2008 av Arnfinn Aas Eielsen
% Oppdatert høsten 2009 av Jonathan Ronen
% Updated fall 2010, Dominik Breu
% Updated fall 2013, Mark Haring
% Updated spring 2015, Mark Haring


%%%%%%%%%%% Calibration of the encoder and the hardware for the specific
%%%%%%%%%%% helicopter
Joystick_gain_x = 1;
Joystick_gain_y = -1;


%%%%%%%%%%% Physical constants
g = 9.81; % gravitational constant [m/s^2]
l_c = 0.46; % distance elevation axis to counterweight [m]
l_h = 0.66; % distance elevation axis to helicopter head [m]
l_p = 0.175; % distance pitch axis to motor [m]
m_c = 1.92; % Counterweight mass [kg]
m_p = 0.72; % Motor mass [kg]

j_p = 2*m_p*(l_p^2); %pitch moment of inertia [kg*m^2]
j_e = m_c*(l_c^2) + 2*m_p*(l_h^2); %elevation moment of intertia [kg*m^2]
j_lambda = m_c*(l_c^2) + 2*m_p*(l_h^2+l_p^2); %travel moment of intertia [kg*m^2]
K_f = 0.14; 
K_1 = l_p * K_f / j_p;


L_1 = l_p*K_f; % ligning 2a
L_2 = m_c*l_c*g - 2*l_h*m_p*g; % ligning 2b
L_3 = l_h*K_f; % ligning 2c

zeta = 0.2;
K_pd = 7;
K_rp = -1;
K_pp = (K_pd/2)^(2/3)/((zeta^(2/3))*K_1^(1/3));

% PART III - PROBLEM 2 --------------------------------------

A_old = [0 1 0 ; 0 0 0 ; 0 0 0];
B_old = [ 0 0 ; 0 L_1/j_p ; L_3/j_e 0 ];
C_old = [1 0 0 ; 0 0 1];

Q_old = 2*[ 67 0 0 ; 0 10 0 ; 0 0 50]; %Weighting matrix for states, initial values from brysons rule
R_old = [ 1 0 ; 0 1 ]; %Weighting matrix for inputs, leave untouched: tuning in Q

K_old = lqr(A_old,B_old,Q_old,R_old);

%P = linsolve( B,(B*K-A)*[1 0; 0 0; 0 1]);
P_old = inv(C_old*inv(-A_old+B_old*K_old)*B_old);


% PART III - PROBLEM 3 --------------------------------------
A = [0 1 0 0 0;0 0 0 0 0 ; 0 0 0 0 0; 1 0 0 0 0; 0 0 1 0 0];
B = [ 0 0 ; 0 L_1/j_p ; L_3/j_e 0; 0 0; 0 0 ];
C = [1 0 0 0 0; 0 0 1 0 0];
D = [0 0; 0 0; 0 0; -1 0; 0 -1];

Q = 2*[ 67 0 0 0 0; 0 10 0 0 0; 0 0 50 0 0; 0 0 0 24 0; 0 0 0 0 17]; %Weighting matrix for states, initial values from brysons rule
R = [ 1 0 ; 0 1 ]; %Weighting matrix for inputs, leave untouched: tuning in Q

K = lqr(A,B,Q,R);
% S = C*inv(B*K-A);
% SB = S*B;
% linsolve(S*B, eye(2))
% P = inv(C*inv(-A+B*K)*B);
