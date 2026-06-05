function calibration = buildFundamentalMatrices(calibration, N)
% BUILDFUNDAMENTALMATRICES  Add fundamental matrices F{i,j} to a calibration.
%
%   calibration = buildFundamentalMatrices(calibration, N)
%
%   Computes F{i,j} for every ordered camera pair from the .intrinsics, .R, .t
%   fields and stores them in calibration.F (an N x N cell, diagonal empty).
%   F{i,j} maps a point in camera i to its epipolar line in camera j:
%       x_j' * F{i,j} * x_i = 0   for corresponding points.
%   Relation used: F = K_j^-T * [t_rel]_x * R_rel * K_i^-1, with R_rel, t_rel
%   the pose of camera j relative to camera i.
%
%   Convention note: assumes R, t are world-to-camera (see ASSUMPTIONS.md / BUG-5).
%
%   INPUTS
%     calibration — struct with .intrinsics{i}, .R{i} (3x3), .t{i} (3x1)
%     N           — number of cameras
%
%   OUTPUT
%     calibration — same struct with .F{i,j} added (3x3 per pair)
%
%   See also: initSystem, associateViews

calibration.F = cell(N, N);

for i = 1:N
    for j = 1:N
        if i == j, continue; end

        % Intrinsic matrices (IntrinsicMatrix is stored transposed; ' gives K).
        K_i = calibration.intrinsics{i}.IntrinsicMatrix';
        K_j = calibration.intrinsics{j}.IntrinsicMatrix';

        % Relative rotation/translation: world <- i, then i <- j.
        R_rel = calibration.R{j} * calibration.R{i}';
        t_rel = calibration.t{j} - R_rel * calibration.t{i};

        tx = t_rel(1); ty = t_rel(2); tz = t_rel(3);
        T_skew = [ 0,  -tz,  ty;
                   tz,  0,  -tx;
                  -ty,  tx,   0];

        E = T_skew * R_rel;                            % essential matrix
        calibration.F{i,j} = inv(K_j)' * E * inv(K_i); % fundamental matrix
    end
end

end
