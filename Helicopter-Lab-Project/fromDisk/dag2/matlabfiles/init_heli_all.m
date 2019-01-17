
%%%%%%%%%%% Calibration of the encoder and the hardware for the specific
%%%%%%%%%%% helicopter
Joystick_gain_x = 1;
Joystick_gain_y = -1;

A = [0 1 0 ; 0 0 0 ; 0 0 0];
B = [ 0 0 ; 0 L_1/j_p ; L_3/j_e 0 ];
C = [1 0 0 ; 0 0 1];

Q = 2*[ 67 0 0 ; 0 10 0 ; 0 0 50]; %Weighting matrix for states, initial values from brysons rule
R = [ 1 0 ; 0 1 ]; %Weighting matrix for inputs, leave untouched: tuning in Q

K = lqr(A,B,Q,R);

%P = linsolve( B,(B*K-A)*[1 0; 0 0; 0 1]);
P = inv(C*inv(-A+B*K)*B);
