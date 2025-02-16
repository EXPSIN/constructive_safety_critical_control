function [A_L, n_l] = get_initial_vecs(n, c_A)


% Calculate the minimum number of vectors required in an n-dimensional space
% given the constant c_A such that for any vector l_i, there are at least n 
% vectors l_k satisfying l_i^T * l_k >= c_A.
theta_c   = acos(c_A);
A_sphere  = 2 * pi^(n/2) / gamma(n/2);
integrand = @(theta) (sin(theta).^(n-1));
A_cap     = integral(@(theta) integrand(theta), 0, theta_c);
A_cap     = (2 * pi^(n/2) / gamma(n/2)) * A_cap;
n_l       = ceil((n * A_sphere) / A_cap);


vecs = randn(n, n_l);
vecs = vecs ./ vecnorm(vecs);

if(n <= 3)
    figure(1); clf;
    axis equal; hold on;
    title('Vectors in the Initial Positive Basis');
    if(n == 2)
        H = scatter(vecs(1, :), vecs(2, :), 'filled', 'DisplayName', 'basis');
    else
        view(3); 
        [x, y, z] = sphere(50);
        surf(x, y, z, 'FaceColor','flat','FaceAlpha', 0.3, 'linestyle', 'none', 'DisplayName', 'unit ball');
        H = scatter3(vecs(1, :), vecs(2, :), vecs(3, :), 'filled', 'DisplayName', 'basis');
    end
    legend;
end

for i = 1:500
    d_vecs = compute_repulsive_forces(vecs);
    vecs = vecs + d_vecs/n_l*0.2;
    vecs = vecs ./ vecnorm(vecs);
    
    if(n <= 3)
        H.XData = vecs(1, :);
        H.YData = vecs(2, :);
        if(n == 3)
            H.ZData = vecs(3, :);
        end
        drawnow;
    end
end

A_L = vecs';
end

function forces = compute_repulsive_forces(ves)
    % positions: n x N matrix, where n is the dimension and N is the number of points
    % forces: n x N matrix, repulsive forces for each point

    [n, N] = size(ves);
    forces = zeros(n, N);
    
    % Calculate repulsive forces for each point
    for i = 1:N
        F_i = zeros(n, 1);  % Initialize force for point i
        
        for j = 1:N
            if i ~= j
                r_ij    = ves(:, i) - ves(:, j); % Vector from r_j to r_i
                dist_ij = norm(r_ij);            % Distance between r_i and r_j
                
                if dist_ij > 1e-10
                    F_ij = r_ij / dist_ij;
                    F_i  = F_i + F_ij;  % Sum the forces
                end
            end
        end
        
        forces(:, i) = F_i-(ves(:, i)' * F_i) * ves(:, i);
    end
end
