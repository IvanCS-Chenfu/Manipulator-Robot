function [P, Q] = generate_smooth_path(P0, P1, P2, tau, T, t)
% Computes a smooth Cartesian path through P0 -> P1 -> P2
% using Taylor method with smoothing.
% Outputs:
%   P = position vector [Px, Py, Pz]
%   Q = quaternion [S, X, Y, Z]

if (t < -T || t > T)
    disp('Parameter t out of range');
    P = []; Q = [];
    return;
end

% Extract positions
p0 = P0(1:3,4);
p1 = P1(1:3,4);
p2 = P2(1:3,4);

% Extract and normalize quaternions
q0 = tr2q(P0); q0 = q0 / norm(q0);
q1 = tr2q(P1); q1 = q1 / norm(q1);
q2 = tr2q(P2); q2 = q2 / norm(q2);

% Deltas
dp1 = p1 - p0;
dp2 = p2 - p1;

% Time normalization
lambda1 = (t + T) / T;
lambda2 = t / T;

if t <= -tau
    % First segment (P0 → P1)
    [P, Q] = qpinter(P0, P1, lambda1);

elseif t >= tau
    % Third segment (P1 → P2)
    [P, Q] = qpinter(P1, P2, lambda2);

else
    % Smoothing segment

    % --- Position smoothing (Eq. 8) ---
    P = p1 - ((tau - t)^2 / (4 * tau * T)) * dp1 + ((tau + t)^2 / (4 * tau * T)) * dp2;

    % --- Orientation smoothing (Eq. 9–14) ---
    % q01 = rotation from q0 to q1
    q01 = quatmultiply(quatconj(q0), q1);
    theta01 = 2 * acos(q01(1));
    if abs(theta01) < 1e-8
        n01 = [1 0 0];
    else
        n01 = q01(2:4) / sin(theta01 / 2);
    end

    % q12 = rotation from q1 to q2
    q12 = quatmultiply(quatconj(q1), q2);
    theta12 = 2 * acos(q12(1));
    if abs(theta12) < 1e-8
        n12 = [1 0 0];
    else
        n12 = q12(2:4) / sin(theta12 / 2);
    end

    % Smooth angles
    theta_k1 = -((tau - t)^2 / (4 * tau * T)) * theta01;
    theta_k2 =  ((tau + t)^2 / (4 * tau * T)) * theta12;

    % Quaternion increments
    qk1 = [cos(theta_k1/2), n01 * sin(theta_k1/2)];
    qk2 = [cos(theta_k2/2), n12 * sin(theta_k2/2)];

    % Final quaternion
    Q = quatmultiply(quatmultiply(q1, qk1), qk2);
end
end
