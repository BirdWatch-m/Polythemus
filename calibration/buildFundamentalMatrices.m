function calibration = buildFundamentalMatrices(calibration, N)
% BUILDFUNDAMENTALMATRICES Computes fundamental matrices for all camera pairs.


calibration.F = cell(N, N);

for i = 1:N
    for j = 1:N
        if i == j, continue; end

        K_i = calibration.intrinsics{i}.IntrinsicMatrix';
        K_j = calibration.intrinsics{j}.IntrinsicMatrix';

        R_rel = calibration.R{j} * calibration.R{i}';
        t_rel = calibration.t{j} - R_rel * calibration.t{i};

        tx = t_rel(1); ty = t_rel(2); tz = t_rel(3);
        T_skew = [ 0,  -tz,  ty;
                   tz,  0,  -tx;
                  -ty,  tx,   0];

        E = T_skew * R_rel;
        calibration.F{i,j} = inv(K_j)' * E * inv(K_i);
    end
end

end
