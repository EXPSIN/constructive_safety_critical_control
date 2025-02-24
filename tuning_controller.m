% Clear environment
close all; clear all; clc;

%%-------------------------------------------------------------------------
% Define symbolic variables and set assumptions
syms m; 
assume(m, {'positive', 'integer'});  
m = 4;                               % [User-Defined]

% Define and assume parameters for the system
syms g_bar [1, m];
syms g_ubar [1, m];
syms delta_bar [1, m]; 

for i = 1:m
    eval(sprintf('g_bar%d=1;', i));         % [User-Defined]
    eval(sprintf('g_ubar%d=1;', i));        % [User-Defined]
    eval(sprintf('delta_bar%d=0;', i));     % [User-Defined]
end

% Initialize system dynamics functions to zero
syms f_bar_o(s) [1, m];
for i =2:m
    eval(sprintf('f_bar_o%d(s)=0;', i));  % Set each f_bar_x to 0
end

% f_bar_o4(s) = s^2;

%%-------------------------------------------------------------------------
%% Controller (64)
% Given parameters for gamma_1_2
syms gamma_1_2(s);
k_gamma_12   = 4.0;                 % [User-Defined]
gamma_1_2(s) = k_gamma_12 * s;      % [User-Defined]

% Define additional parameters for controller design
syms gamma_x2r_V(s);                % \gamma_{x^*_2,V}
syms k_1;
assume(k_1, {'positive', 'real'});  
gamma_x2r_V(s) = 1/4 * s;           % [User-Defined]
k_1 = 3.49;                         % [User-Defined]

%%-------------------------------------------------------------------------
%% Design Controller (74)
k_gamma_i  = 1/4.001;                % [User-Defined] Define gains factor for inter-controller coupling
k_gamma_i1 = k_gamma_i;               
k_gamma_ij = k_gamma_i;             
syms theta;
assume(theta, {'positive', 'real'}); 
sigma(s) = 1e-3*s;                   % [User-Defined]


for i = 2:m
    fprintf('Calculating the %d-th gains.\n', i);       % Display the current controller number being calculated
    
    
    eval(sprintf('syms gamma_%d_x1(s);', i));           % \gamma_{i,x_1}, a gain function
    eval(sprintf('syms gamma_%d_x1_inv(s);', i));       % \gamma_{i,x_1}^{-1}, the inverse gain
    eval(sprintf('syms gamma_%d_rho0(s);', i));         % \gamma_{i,\rho_0}, a gain function for rho_0
    eval(sprintf('syms gamma_%d_rho0_inv(s);', i));     % \gamma_{i,\rho_0}^{-1}, the inverse gain for rho_0
    
    for j = 1:i-1
        eval(sprintf('syms gamma_%d_%d(s);', i, j));    % \gamma_{i,1}, ..., \gamma_{i,i-1}
        eval(sprintf('syms gamma_%d_%d_inv(s);', i, j));% \gamma_{i,1}^{-1}, ..., \gamma_{i,i-1}^{-1}
    end
    
    eval(sprintf('syms gamma_%d_%d(s);', i, i+1));      % \gamma_{i,i+1}
    eval(sprintf('syms gamma_%d_%d_inv(s);', i, i+1));  % \gamma_{i,i+1}^{-1}
    
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

    f_bar_o_tmp = gamma_x2r_V(V_breve) + rho_bar_0 + x_tilde_norm_2;
    for j = 3:i
        eval(sprintf(['f_bar_o_tmp = f_bar_o_tmp + ' ...
            '(g_ubar%d/(g_ubar%d-delta_bar%d) * kappa_%d(x_tilde_norm_%d) + x_tilde_norm_%d ); '], ...
            j,j,j, j-1, j-1, j));
    end

    eval(sprintf('alpha_f_%d(%s) = c_x1 + f_bar_o%d(f_bar_o_tmp);', i, variables_str{i}, i));

    eval(sprintf(['alpha_%d(%s) = (g_bar%d+delta_bar%d)*x_tilde_norm_%d + alpha_f_%d(%s) ' ...
        '+ k_1*k_prop(2, %d)*(c_x1 + (g_bar1 + delta_bar1)*(gamma_x2r_V(V_breve) + rho_bar_0 + x_tilde_norm_2) );'], ...
        i, variables_str{i}, i,i, i+1, i, variables_str{i}, i-1));

    for q = 2:i-1
        eval(sprintf(['alpha_%d(%s) = alpha_%d(%s) + k_prop(%d, %d)*(alpha_f_%d(%s) + ' ...
            '(g_bar%d+delta_bar%d)*(g_ubar%d*kappa_%d(x_tilde_norm_%d)/(g_ubar%d-delta_bar%d) + x_tilde_norm_%d ));'], ...
            i, variables_str{i}, i, variables_str{i}, q, i-1, q, variables_str{q}, q, q,q, q,q, q,q, q+1));
    end
    
    eval(sprintf('kappa_%d(s) = sigma(s) + 1/g_ubar%d*alpha_%d(%s);', i, i, i, variables_inverse_str{i}));
    
    eval(sprintf('k_%d(s) = g_ubar%d/(g_ubar%d - delta_bar%d) * (g_ubar%d/g_ubar%d*kappa_%d(s)/s + diff(kappa_%d,s));', i,i,i,i,i, i, i, i));
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
