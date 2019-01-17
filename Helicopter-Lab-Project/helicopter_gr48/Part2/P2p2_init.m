% Joystick gains for part 2, used for testing cascaded PD regulators
% joystick -> travel PD regulator -> pitch PD regulator
Joystick_gain_x = 1;
Joystick_gain_y = -1;

% We decided that critically dampening the system was too slow,
% so we concluded that a theoretical underdampened system was fast enough, 
% and proved to in reality have no oscillations.

% Pitch PD constants, recycled from part 2 task 1. 
zeta = 0.2;
K_pd = 7; % A constant for pitch differential gain, set through tuning. 
K_pp = (K_pd/2)^(2/3)/((zeta^(2/3))*K_1^(1/3)); % K_pp as a function of K_pd and zeta, provided linearized pitch. 

% Travelrate P constant. 
K_rp = -1; % Travel-rate P regulator gain. Found through tuning.  