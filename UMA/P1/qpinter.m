function [p_interp, q_interp] = qpinter(P1, P2, lambda)
% Interpolates between two homogeneous transformation matrices P1 and P2.
% lambda ∈ [0, 1] is the interpolation parameter.

% Extract positions from homogeneous matrices
p1 = P1(1:3, 4);
p2 = P2(1:3, 4);

% Convert rotation matrices to quaternions
q1 = tr2q(P1); % returns [w, x, y, z]
q2 = tr2q(P2);

% Normalize quaternions
q1 = q1 / norm(q1);
q2 = q2 / norm(q2);

% Ensure shortest path (dot product must be positive)
if dot(q1, q2) < 0
    q2 = -q2;
end

% Compute relative rotation quaternion
q1_conj = [q1(1), -q1(2:4)]; % conjugate of q1 = [w, -x, -y, -z]
qc = quatmultiply(q1_conj, q2);

% Extract angle and axis from qc
theta = 2 * acos(qc(1));
if abs(theta) < 1e-8
    axis = [1; 0; 0]; % Arbitrary axis if rotation is negligible
else
    axis = qc(2:4) / sin(theta / 2);
end

% Interpolate angle
theta_lambda = lambda * theta;

% Rotation quaternion for interpolated angle
q_rot = [cos(theta_lambda / 2), axis * sin(theta_lambda / 2)];

% Final interpolated quaternion
q_interp = quatmultiply(q1, q_rot);

% Interpolated position
p_interp = (1 - lambda) * p1 + lambda * p2;
end
