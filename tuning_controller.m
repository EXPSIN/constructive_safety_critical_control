%{
    This scrp
%} 
% Clear environment
close all; clear all; clc;

%%-------------------------------------------------------------------------
%% Step 1: Assumptions 1 and 2
% Define symbolic variables and set assumptions
syms m; 
assume(m, {'positive', 'integer'});  % Assume m is a positive integer
m = 4;                               % [User-Defined] Set the system relative degree

% Define and assume parameters for the system
syms g_bar g_ubar delta_bar; 
assume(g_bar, {'positive', 'real'});  % Assume g_bar is a positive real number
assume(g_ubar, {'positive', 'real'}); % Assume g_ubar is a positive real number
assume(delta_bar, {'positive', 'real'}); % Assume delta_bar is a positive real number

% Define symbolic functions for the system dynamics
syms f_bar_x(s) [1, m];

% Assign values to the parameters
g_bar = 1;      % [User-Defined] Set g_bar to 1
g_ubar = 1;     % [User-Defined] Set g_ubar to 1
delta_bar = 0;  % [User-Defined] Set delta_bar to 0

% Initialize system dynamics functions to zero
for i = 1:m
    eval(sprintf('f_bar_x%d(s)=0;', i));  % Set each f_bar_x to 0
end

% f_bar_x2(s) = s^2;

%%-------------------------------------------------------------------------
%% Step 2: Design Controller (Proposition 2)
% Given parameters for gamma_1_2
syms gamma_1_2(s);
k_gamma_12   = 4.0;             % [User-Defined] Define gain k_gamma_12
gamma_1_2(s) = k_gamma_12 * s;  % [User-Defined] Define gamma_1_2 as a function of s

% Define additional parameters for controller design
syms gamma_x2r_V(s);                % \gamma_{x^*_2,V}
syms k_1;
assume(k_1, {'positive', 'real'});  % Assume k_1 is positive and real
gamma_x2r_V(s) = 1/4 * s;           % [User-Defined] Set gamma_x2r_V as a function of s
k_1 = 3.49;                         % [User-Defined] Set the value of k_1

%%-------------------------------------------------------------------------
%% Step 3: Assign Gain Parameters (Theorem 3)
% Define controller gain parameters
k_gamma_ij = 1/1.001;               % [User-Defined] Define small gain factor for inter-controller coupling
k_gamma_i1 = 1/k_gamma_12/1.001;    % [User-Defined] Define the inverse gain for the first controller

%%-------------------------------------------------------------------------
%% Step 4: Design Controller (Proposition 3)
syms theta;
assume(theta, {'positive', 'real'});    % Assume theta is positive and real
theta = 1e-3;                           % [User-Defined] Set the value of theta

for i = 2:m
    fprintf('Calculating the %d-th gains.\n', i);       % Display the current controller number being calculated
    
    % Define symbolic variables for different gamma functions
    eval(sprintf('syms gamma_%d_x1(s);', i));           % \gamma_{i,x_1}, a gain function
    eval(sprintf('syms gamma_%d_x1_inv(s);', i));       % \gamma_{i,x_1}^{-1}, the inverse gain
    eval(sprintf('syms gamma_%d_rho0(s);', i));         % \gamma_{i,\rho_0}, a gain function for rho_0
    eval(sprintf('syms gamma_%d_rho0_inv(s);', i));     % \gamma_{i,\rho_0}^{-1}, the inverse gain for rho_0
    
    % Define symbolic gamma functions for inter-controller gains (i, j)
    for j = 1:i-1
        eval(sprintf('syms gamma_%d_%d(s);', i, j));    % \gamma_{i,1}, ..., \gamma_{i,i-1}
        eval(sprintf('syms gamma_%d_%d_inv(s);', i, j));% \gamma_{i,1}^{-1}, ..., \gamma_{i,i-1}^{-1}
    end
    
    eval(sprintf('syms gamma_%d_%d(s);', i, i+1));      % \gamma_{i,i+1}
    eval(sprintf('syms gamma_%d_%d_inv(s);', i, i+1));  % \gamma_{i,i+1}^{-1}
    
    % Assign values to gamma functions (linear in s)
    eval(sprintf('gamma_%d_x1(s) = %f*s;', i, 1/0.1));        % [User-Defined] \gamma_{i,x_1}
    eval(sprintf('gamma_%d_x1_inv(s) = %f*s;', i, 0.1));      % [User-Defined] \gamma_{i,x_1}^{-1}
    eval(sprintf('gamma_%d_rho0(s) = %f*s;', i, 1/0.1));      % [User-Defined] \gamma_{i,\rho_0}
    eval(sprintf('gamma_%d_rho0_inv(s) = %f*s;', i, 0.1));    % [User-Defined] \gamma_{i,\rho_0}^{-1}
    
    for j = 1:i-1
        eval(sprintf('gamma_%d_%d(s) = %f*s;', i, j, k_gamma_ij));          % [User-Defined] \gamma_{i,1}, ..., \gamma_{i,i-1}
        eval(sprintf('gamma_%d_%d_inv(s) = %f*s;', i, j, 1/k_gamma_ij));    % [User-Defined] \gamma_{i,1}^{-1}, ..., \gamma_{i,i-1}^{-1}
    end
    
    eval(sprintf('gamma_%d_%d(s) = %f*s;', i, i+1, k_gamma_ij));        % [User-Defined] \gamma_{i,i+1}
    eval(sprintf('gamma_%d_%d_inv(s) = %f*s;', i, i+1, 1/k_gamma_ij));  % [User-Defined] \gamma_{i,i+1}^{-1}
    eval(sprintf('gamma_%d_2(s) = %f*s;', i, 1/1.001));                 % [User-Defined] \gamma_{i,2}
    eval(sprintf('gamma_%d_2_inv(s) = %f*s;', i, 1.001));               % [User-Defined] \gamma_{i,2}^{-1}


    variables_str{i} = 'rho_bar_0, c_x1, V_breve';
    variables_inverse_str{i} = sprintf('gamma_%d_rho0_inv(s), gamma_%d_x1_inv(s), gamma_%d_1_inv(s)', i,i,i);
    for j = 2:i+1
        variables_str{i} = [variables_str{i}, sprintf(', x_tilde_norm_%d', j)];
        if(j == i)
            variables_inverse_str{i} = [variables_inverse_str{i}, sprintf(', s')];
        else
            variables_inverse_str{i} = [variables_inverse_str{i}, sprintf(', gamma_%d_%d_inv(s)', i, j)];
        end
        
    end

    eval(sprintf('syms alpha_%d(%s)', i, variables_str{i}));
    eval(sprintf('syms alpha_f_%d(%s)', i, variables_str{i}));

    eval(sprintf('alpha_f_%d(%s) = f_bar_x2(gamma_x2r_V(V_breve) + rho_bar_0 + x_tilde_norm_2) + c_x1;', i, variables_str{i}));
    for j = 3:i
        eval(sprintf(['alpha_f_%d(%s) = alpha_f_%d(%s) + ' ...
            'f_bar_x%d(g_ubar/(g_ubar-delta_bar) * kappa_%d(x_tilde_norm_%d) + x_tilde_norm_%d ); '], ...
            i, variables_str{i}, i, variables_str{i}, ...
            j, j-1, j-1, j));
    end

    eval(sprintf(['alpha_%d(%s) = (g_bar+delta_bar)*x_tilde_norm_%d + alpha_f_%d(%s) ' ...
        '+ k_1*k_prop(2, %d)*(c_x1 + (g_bar + delta_bar)*(gamma_x2r_V(V_breve) + rho_bar_0 + x_tilde_norm_2) );'], ...
        i, variables_str{i}, i+1, i, variables_str{i}, i-1));

    for q = 2:i-1
        eval(sprintf(['alpha_%d(%s) = alpha_%d(%s) + k_prop(%d, %d)*(alpha_f_%d(%s) + ' ...
            '(g_bar+delta_bar)*(g_ubar*kappa_%d(x_tilde_norm_%d)/(g_ubar-delta_bar) + x_tilde_norm_%d ));'], ...
            i, variables_str{i}, i, variables_str{i}, q, i-1, q, variables_str{q}, q,q,q+1));
    end
    
    eval(sprintf('kappa_%d(s) = theta*s + 1/g_ubar*alpha_%d(%s);', i, i, variables_inverse_str{i}));
    
    eval(sprintf('k_%d(s) = g_ubar/(g_ubar - delta_bar) * (kappa_%d(s)/s + diff(kappa_%d,s));', i, i, i));
end

for i = 2:m
    fprintf('kappa_%d(s) = ', i);
    disp(vpa(expand(simplify(eval( sprintf('kappa_%d', i)) )), 6))
end



function res = k_prop(q, i)
res = 1;
for j = q:i-1
    eval(sprintf('x_tilde_norm_%d=evalin(''base'', ''x_tilde_norm_%d'');', j, j));
    eval(sprintf('k_%d(x_tilde_norm_%d)=evalin(''base'', ''k_%d(x_tilde_norm_%d)'');', j, j, j, j));
    eval(sprintf('res = res * k_%d(x_tilde_norm_%d);', j, j));
end
end