close all; clear all; clc;

% Define parameters.
n_x2    = 3;  
c_A     = cosd(45);

% Define the error in the cosine value.
c_A_err = min(0.05, (1-c_A)*0.5);  
o       = sqrt(2 - 2*(c_A + c_A_err)) - sqrt(2 - 2*c_A);  

% Step-1: Obtain the initial positive basis that satisfies the first and 
% third properties.
[A_L, n_l] = get_initial_vecs(n_x2, c_A + c_A_err);

% Step-2: Randomly adjust the directions of the row vectors in A_L within 
% a small range to satisfy the second property associated with A_L.
temp = A_L;
while true
    temp = A_L + (rand(size(A_L)) - 0.5) * o;  
    temp = temp ./ vecnorm(temp);
    
    if (~is_singular(temp))  
        A_L = temp;
        break;
    end
end

% Display the resulting matrix A_L
fprintf('The resulting A_L is:\n');
disp(A_L');
