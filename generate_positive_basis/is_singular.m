function isSingular = is_singular(A_L)
[n_l, n_x2] = size(A_L);
isSingular = false;

combs = nchoosek(1:n_l, n_x2);

for i = 1:size(combs, 1)
    selected_vectors = A_L(combs(i, :), :);
    det_val = det(selected_vectors);

    if abs(det_val) < 1e-10
        isSingular = true;
        break;
    end
end
end

