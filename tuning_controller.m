%{
    This scrp
%} 
% Clear environment
close all; clear all; clc;

%%-------------------------------------------------------------------------
%% Step 1: Verify Assumptions 1 and 2
% Define symbolic variables and set assumptions
syms m; 
assume(m, {'positive', 'integer'});  % Assume m is a positive integer
m = 4;  % Set the system dimension (number of states) to 4

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



%%-------------------------------------------------------------------------
%% Step 2: Design Controller (70) (Proposition 2)
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

% Define symbolic variables for controller gains
syms k_bar_1_1;                 % \bar{k}_{1,1} = k_1
syms k_breve_1_1;               % \breve{k}_{1,1} = k_1
k_bar_1_1 = k_1;                % Assign value of k_bar_1_1
k_breve_1_1 = 1 + k_bar_1_1;    % Define k_breve_1_1 as 1 + k_bar_1_1

% Loop to define other controller gain variables
for i = 1:m
    for p = i+1:m
        eval(sprintf('syms k_bar_%d_%d;', p, i));   % Define symbolic k_bar for pair (p, i)
        eval(sprintf('k_bar_%d_%d = 0;', p, i));    % Set k_bar to 0 for now
        eval(sprintf('syms k_breve_%d_%d;', p, i)); % Define symbolic k_breve for pair (p, i)
        eval(sprintf('k_breve_%d_%d = 1;', p, i));  % Set k_breve to 1
    end
end

%%-------------------------------------------------------------------------
%% Step 3: Assign Gain Parameters (Theorem 3)
% Define controller gain parameters
k_gamma_ij = 1/1.001;               % [User-Defined] Define small gain factor for inter-controller coupling
k_gamma_i1 = 1/k_gamma_12/1.001;    % [User-Defined] Define the inverse gain for the first controller

%%-------------------------------------------------------------------------
%% Step 4: Design Controller (84) (Proposition 3)
syms theta;
assume(theta, {'positive', 'real'});    % Assume theta is positive and real
theta = 1e-3;                           % [User-Defined] Set the value of theta

% Loop to calculate the gains for controllers from i = 2 to m
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
    eval(sprintf('gamma_%d_x1(s) = %f*s;', i, 1/0.001));        % [User-Defined] \gamma_{i,x_1}
    eval(sprintf('gamma_%d_x1_inv(s) = %f*s;', i, 0.001));      % [User-Defined] \gamma_{i,x_1}^{-1}
    eval(sprintf('gamma_%d_rho0(s) = %f*s;', i, 1/0.001));      % [User-Defined] \gamma_{i,\rho_0}
    eval(sprintf('gamma_%d_rho0_inv(s) = %f*s;', i, 0.001));    % [User-Defined] \gamma_{i,\rho_0}^{-1}
    
    for j = 1:i-1
        eval(sprintf('gamma_%d_%d(s) = %f*s;', i, j, k_gamma_ij));          % [User-Defined] \gamma_{i,1}, ..., \gamma_{i,i-1}
        eval(sprintf('gamma_%d_%d_inv(s) = %f*s;', i, j, 1/k_gamma_ij));    % [User-Defined] \gamma_{i,1}^{-1}, ..., \gamma_{i,i-1}^{-1}
    end
    
    eval(sprintf('gamma_%d_%d(s) = %f*s;', i, i+1, k_gamma_ij));        % [User-Defined] \gamma_{i,i+1}
    eval(sprintf('gamma_%d_%d_inv(s) = %f*s;', i, i+1, 1/k_gamma_ij));  % [User-Defined] \gamma_{i,i+1}^{-1}
    eval(sprintf('gamma_%d_2(s) = %f*s;', i, 1/1.001));                 % [User-Defined] \gamma_{i,2}
    eval(sprintf('gamma_%d_2_inv(s) = %f*s;', i, 1.001));               % [User-Defined] \gamma_{i,2}^{-1}
    
    % Define alpha functions (control parameters)
    eval(sprintf('syms alpha_%d_x1(s);', i));   % \alpha_{i,x_1}
    eval(sprintf('syms alpha_%d_rho0(s);', i)); % \alpha_{i,\rho_0}
    eval(sprintf('syms alpha_%d_1(s);', i));    % \alpha_{i,1}
    
    for j = 1:i-1
        eval(sprintf('syms alpha_%d_%d(s);', i, j));  % \alpha_{i,j}
    end

    % Define the expression for \alpha_{i,x_1}, \alpha_{i,\rho_0}, etc.
    eval(sprintf('alpha_%d_x1(s)=k_breve_1_%d*s;', i, i-1));   % \alpha_{i,x_1}(s)
    
    % \alpha_{i,x_1}(s) = \breve{k}_{2,i-1}\bar{f}_{x_2}(4s) + \bar{k}_{1,i-1}(\bar{g}+\bar{\delta})s;
    eval(sprintf('alpha_%d_rho0(s)=k_breve_2_%d*f_bar_x2(4*s)+k_breve_1_%d*(g_bar+delta_bar)*s;', i, i-1, i-1));

    % \alpha_{i,1}(s) = \alpha_{i,\rho_0}\circ\gamma_{x^*_2,V}(s)
    eval(sprintf('alpha_%d_1(s)=alpha_%d_rho0(gamma_x2r_V(s));', i, i));   


    for j = 2:i-1
        % \alpha_{i,j}(s) = \breve{k}_{j,i-1}\bar{f}_{x_{j}}(2s) + 
        %                   \breve{k}_{j,i-1}\bar{f}_{x_{j+1}}\left(\frac{2\ubar{g} \kappa_j(s)}{\ubar{g}-\bar{\delta}}\right) + 
        %                   (\bar{g}+\bar{\delta})\left(\bar{k}_{j,i-1}\frac{\ubar{g}\kappa_j(s)}{\ubar{g}-\bar{\delta}}+\bar{k}_{j-1,i-1}s\right) 
        eval(sprintf(['alpha_%d_%d(s) = k_breve_%d_%d*f_bar_x%d(2*s) + ' ...
                      'k_breve_%d_%d*f_bar_x%d(2*g_ubar*kappa_%d(s)/(g_ubar-delta_bar)) + ' ...
                      '(g_bar+delta_bar)*(k_bar_%d_%d*g_ubar*kappa_%d(s)/(g_ubar-delta_bar) + k_bar_%d_%d*s);'], ...
            i, j, j, i-1, j, ...
            j, i-1, j+1, j, ...
            j, i-1, j, j-1, i-1));
    end
    % \alpha_{i,i}(s) = \bar{f}_{x_i}(2s) + k_{i-1}(\bar{g}+\bar{\delta})s
    eval(sprintf('alpha_%d_%d(s)=f_bar_x%d(2*s)+k_%d*(g_bar+delta_bar)*s;', i, i, i, i-1));

    % kappa_i(s) \ge \theta s + \frac{\ubar{g}+\bar{\delta}}{\ubar{g}}\gamma_{i,i+1}^{-1}(s) + \frac{1}{\ubar{g}} (\alpha_{i,x_1}\circ\gamma_{i,x_1}^{-1}(s)\notag \\ 
    %                +\alpha_{i,\rho_0}\circ\gamma^{-1}_{i,\rho_0}(s) + \sum^{i-1}_{j=1}\alpha_{i,j}\circ\gamma_{i,j}^{-1}(s) + \alpha_{i,i}(s))
    eval(sprintf('syms kappa_%d(s);', i));
    eval(sprintf(['kappa_%d(s) = theta*s + (g_ubar+delta_bar)/g_bar * gamma_%d_%d_inv + ' ...
        '1/g_ubar * (alpha_%d_x1(gamma_%d_x1_inv(s)) + alpha_%d_rho0(gamma_%d_rho0_inv(s)) + alpha_%d_%d(s));'], ...
        i, i, i+1, ...
        i, i, i, i, i, i));
    for j = 1:i-1
        eval(sprintf('kappa_%d(s) = kappa_%d(s) + alpha_%d_%d(gamma_%d_%d_inv(s));', ...
            i, i, ...
            i, j, i, j));
    end

    % k_i \ge \frac{\ubar{g}}{\ubar{g}-\bar{\delta}} \left(\frac{\bar{g}}{\ubar{g}}\frac{\kappa_i(s)}{s}+\frac{\mathrm{d} \kappa_i(s)}{\mathrm{d} s}\right)
    eval(sprintf('k_%d = g_ubar/(g_ubar - delta_bar) * (kappa_%d(s)/s + diff(kappa_%d,s));', i, i, i));

    for p = 1:i
        % \bar{k}_{p,i} = \Pi^{i}_{j=p} k_j
        eval(sprintf('k_bar_%d_%d=1;',p, i));
        for j = p:i
            eval(sprintf('k_bar_%d_%d=k_bar_%d_%d * k_%d;', p, i, p, i, j));
        end
    end

    for p = 1:i
        % \bar{k}_{p,i} = \Pi^{i}_{j=p} k_j
        eval(sprintf('k_breve_%d_%d=1;',p, i));
        for j = p:i
            eval(sprintf('k_breve_%d_%d = k_bar_%d_%d + k_bar_%d_%d;', p, i, p, i, j, i));
        end
    end
end

for i = 2:m
    fprintf('kappa_%d(s) = ', i);
    disp(vpa(simplify(eval( sprintf('kappa_%d', i) ) ), 10))
end



