function tau_g = staticTorquePayloadMDH(MDH, q, m_payload, g)
% STATICTORQUEPAYLOADMDH  Gravity torque due to a payload at the wrist.
%
%   tau_g = staticTorquePayloadMDH(MDH, q, m_payload, g)
%
%   Inputs:
%     MDH        : n x 5 MDH table [theta0 d a alpha type]
%     q          : n x 1 joint configuration [rad]
%     m_payload  : payload mass at wrist center [kg]
%     g          : scalar gravity (default 9.81 m/s^2)
%
%   Output:
%     tau_g      : n x 1 joint torque vector [N·m]
%
%   Notes:
%     - Uses your geometric Jacobian: J = jacobianMDH(MDH, q)
%     - Assumes payload acts at frame {n} origin (wrist center).
%     - Ignores link self-weights (only payload contribution).

    if nargin < 4 || isempty(g)
        g = 9.81;
    end

    q = q(:);  % ensure column

    % Geometric Jacobian (6 x n), expressed in base frame
    J = jacobianMDH(MDH, q);

    % Wrench due to gravity on payload in base frame:
    % force = [0; 0; -m*g], no moment at the contact point
    Fg = [0; 0; -m_payload*g; 0; 0; 0];  % 6 x 1

    % Joint torques: tau = J^T * F
    tau_g = J.' * Fg;   % n x 1
end
