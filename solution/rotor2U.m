function [U, Omr] = rotor2U(w, P)
%ROTOR2U  Map the four rotor speeds to the virtual control inputs.
%
%   [U, Omr] = ROTOR2U(w, P)   with w = [w1 w2 w3 w4] in rad/s
%
%   Geometry from Fig. 1 (cross / "plus" configuration):
%       rotor 1 on +x_B, rotor 2 on -y_B, rotor 3 on -x_B, rotor 4 on +y_B
%       rotors 1 and 3 spin one way, rotors 2 and 4 the other
%
%   Each rotor produces thrust  f_i = k*w_i^2  along +z_B
%   and reaction torque         tau_i = b*w_i^2  about z_B.
%
%   Taking tau = r x F for each arm gives:
%       U1 = k(w1^2 + w2^2 + w3^2 + w4^2)     total thrust        [N]
%       U2 = k(w4^2 - w2^2)                   roll  input         [N]
%       U3 = k(w3^2 - w1^2)                   pitch input         [N]
%       U4 = b(w1^2 + w3^2 - w2^2 - w4^2)     yaw torque          [N.m]
%
%   The applied body torques are then  [l*U2, l*U3, U4].
%
%   NOTE ON SIGNS: verify these against Fig. 1 before you present. If your
%   roll response comes out inverted, swap the sign of U2 (and likewise U3
%   for pitch). This is the single most common sign error in quadcopter
%   models and takes ten seconds to check with a step test.

w = w(:).';
w2 = w.^2;

U    = zeros(4,1);
U(1) = P.k * sum(w2);
U(2) = P.k * (w2(4) - w2(2));
U(3) = P.k * (w2(3) - w2(1));
U(4) = P.b * (w2(1) + w2(3) - w2(2) - w2(4));

% Residual rotor speed drives the gyroscopic term
Omr = w(1) - w(2) + w(3) - w(4);
end
