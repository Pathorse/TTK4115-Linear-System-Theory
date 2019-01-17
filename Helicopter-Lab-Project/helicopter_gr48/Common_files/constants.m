g = 9.81; % gravitational constant [m/s^2]
l_c = 0.46; % distance elevation axis to counterweight [m]
l_h = 0.66; % distance elevation axis to helicopter head [m]
l_p = 0.175; % distance pitch axis to motor [m]
m_c = 1.92; % Counterweight mass [kg]
m_p = 0.72; % Motor mass [kg]

j_p = 2*m_p*(l_p^2); %pitch moment of inertia [kg*m^2]
j_e = m_c*(l_c^2) + 2*m_p*(l_h^2); %elevation moment of inertia [kg*m^2]
j_lambda = m_c*(l_c^2) + 2*m_p*(l_h^2+l_p^2); %travel moment of inertia [kg*m^2]
K_f = 0.14; %power constant, translation between voltage and force. [kg*m/(s^2*V)]
K_1 = l_p * K_f / j_p; %TODO, describe me. ratio of V_d to torque on pitch. 


L_1 = l_p*K_f; % equation 2a
L_2 = m_c*l_c*g - 2*l_h*m_p*g; % equation 2b
L_3 = l_h*K_f; % equation 2c