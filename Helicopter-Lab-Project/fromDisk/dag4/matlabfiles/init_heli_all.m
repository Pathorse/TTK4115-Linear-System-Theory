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


%%%%%%%%%%% Calculated constants

K_f = 0.14; 
L_1 = l_p*K_f; % ligning 2a
L_2 = m_c*l_c*g - 2*l_h*m_p*g; % ligning 2b
L_3 = l_h*K_f; % ligning 2c
L_4 = -K_f*l_h;

K_1 = L_1/j_p;
K_2 = L_3/j_e;
K_3 = -(L_2*L_4)/(L_3*j_lambda);


% PART IV - PROBLEM 1

A = [0 1 0 0 0 0; 0 0 0 0 0 0; 0 0 0 1 0 0; 0 0 0 0 0 0; 0 0 0 0 0 1; K_3 0 0 0 0 0];
B = [0 0; 0 K_1; 0 0; K_2 0; 0 0; 0 0];
C = [1 0 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 1 0];


% PART IV - PROBLEM 2

%checking observability
O = [C; C*A; C*(A^2); C*(A^3); C*(A^4)];
row_rank_o = rank(transpose(O));

%creating state observer
pole1 = -10 + 20i;%-80 + 100i;
pole2 = -6 + 10i;%-100 +50i; was -20
pole3 = -5;%-50;
poles = [pole1,pole1',pole2,pole2',pole3,pole3'];
L = place(A',C',poles)';


% CONTROLLER 1 FROM PART III

A_v1 = [0 1 0 ; 0 0 0 ; 0 0 0];
B_v1 = [ 0 0 ; 0 L_1/j_p ; L_3/j_e 0 ];
C_v1 = [1 0 0 ; 0 0 1];

Q_v1 = 2*[ 67 0 0 ; 0 10 0 ; 0 0 50]; %Weighting matrix for states, initial values from brysons rule
R_v1 = [ 1 0 ; 0 1 ]; %Weighting matrix for inputs, leave untouched: tuning in Q

K_v1 = lqr(A_v1,B_v1,Q_v1,R_v1);

%P = linsolve( B,(B*K-A)*[1 0; 0 0; 0 1]);
P_v1 = inv(C_v1*inv(-A_v1+B_v1*K_v1)*B_v1);

% CONTROLLER 2 FROM PART III (WITH INTEGRAL EFFECT)

A_v2 = [0 1 0 0 0;0 0 0 0 0 ; 0 0 0 0 0; 1 0 0 0 0; 0 0 1 0 0];
B_v2 = [ 0 0 ; 0 L_1/j_p ; L_3/j_e 0; 0 0; 0 0 ];
C_v2 = [1 0 0 0 0; 0 0 1 0 0];
D_v2 = [0 0; 0 0; 0 0; -1 0; 0 -1];

Q_v2 = 2*[ 1 0 0 0 0; 0 2 0 0 0; 0 0 2 0 0; 0 0 0 24 0; 0 0 0 0 200]; %Weighting matrix for states, initial values from brysons rule
% was 2*[ 67 0 0 0 0; 0 10 0 0 0; 0 0 50 0 0; 0 0 0 24 0; 0 0 0 0 17];
R_v2 = [ 1 0 ; 0 1 ]; %Weighting matrix for inputs, leave untouched: tuning in Q

K_v2 = lqr(A_v2,B_v2,Q_v2,R_v2);

%LQR regulator

% Q = [ 10 0 0 0 0 0; 0 50 0 0 0 0; 0 0 20 0 0 0; 0 0 0 10 0 0; 0 0 0 0 48 0; 0 0 0 0 0 34]; %Weighting matrix for states, initial values from brysons rule
% %        ---p---      --p_dot--     ---e---      --e_dot--      --lmb--    --lmb_dot--
% R = [1 0; 0 1];
% K = lqr(A,B,Q,R);
% 
% eig_values = eig(A-B*K)

% PART IV - PROBLEM 3

% C matrix for measuring only e and lambda
C_e_lmb = [0 0 1 0 0 0; 0 0 0 0 1 0];
O_e_lmb = [C_e_lmb; C_e_lmb*A; C_e_lmb*(A^2); C_e_lmb*(A^3); C_e_lmb*(A^4)];
row_rank_O_e_lmb = rank(O_e_lmb')

% C matrix for measuring only p and e
C_p_e = [1 0 0 0 0 0; 0 0 1 0 0 0];
O_p_e = [C_p_e; C_p_e*A; C_p_e*(A^2); C_p_e*(A^3); C_p_e*(A^4)];
row_rank_O_p_e = rank(O_p_e')


