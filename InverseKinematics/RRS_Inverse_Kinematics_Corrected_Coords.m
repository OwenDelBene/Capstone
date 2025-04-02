clc; clear; close all;

% More of either narrows the range of the other! 
% Roll Range = [-7.9, 7.9] deg
% Pitch Range = [-6.9, 11.7] deg

% Given from IMU Readings
Roll = deg2rad(-2);
Pitch = deg2rad(7);

% Rotation Matrices
R1 = [1, 0, 0;
      0, cos(Roll), -sin(Roll);
      0, sin(Roll), cos(Roll)];
  
R2 = [cos(Pitch), 0, sin(Pitch);
      0, 1, 0;
      -sin(Pitch), 0, cos(Pitch)];

R3 = eye(3);

Rop = R3 * R2 * R1;

rp = [0; 0; 112.28]; % Units in mm

% Transformation Matrices
Tai = @(angle) [cos(angle), -sin(angle), 0;
                sin(angle), cos(angle), 0;
                0, 0, 1];
            
Tbi = @(angle) [cos(angle), -sin(angle), 0;
                sin(angle), cos(angle), 0;
                0, 0, 1];

% Attachment points
ra_nY = Tai((4*pi)/3) * [294.74; 0; 0];
ra_pY = Tai((2*pi)/3) * [294.74; 0; 0];
ra_X  = [294.74; 0; 0];

rb_nY = Tbi((4*pi)/3) * [224.4; 0; 0];
rb_pY = Tbi((2*pi)/3) * [224.4; 0; 0];
rb_X  = [224.4; 0; 0];

% Function to compute link vectors
L_bai = @(rpp, rop, ra, rb) (rpp + (rop * ra)) - rb;

L_banY = L_bai(rp, Rop, ra_nY, rb_nY);
L_bapY = L_bai(rp, Rop, ra_pY, rb_pY);
L_baX  = L_bai(rp, Rop, ra_X, rb_X);

L_bc = 48.26; % mm
L_ca = 114.43; % mm

% Yaw-pitch transformation
ypbi = @(t) [cos(t), -sin(t), 0;
             sin(t), cos(t), 0;
             0, 0, 1] * [-1; 0; 0];
         
yp_bnY = ypbi((4*pi)/3);
yp_bpY = ypbi((2*pi)/3);
yp_bX  = [-1; 0; 0];

% Angle calculations
alpha = @(lbc, lca, lbai) acos((lbc^2 + norm(lbai)^2 - lca^2) / (2 * lbc * norm(lbai)));

beta = @(lbai, yp) acos(dot(lbai, yp) / norm(lbai));

alpha_ny = alpha(L_bc, L_ca, L_banY);
alpha_py = alpha(L_bc, L_ca, L_bapY);
alpha_X  = alpha(L_bc, L_ca, L_baX);

beta_ny = beta(L_banY, yp_bnY);
beta_py = beta(L_bapY, yp_bpY);
beta_X  = beta(L_baX, yp_bX);

q_nY = rad2deg(alpha_ny + beta_ny - (pi/2));
q_pY = rad2deg(alpha_py + beta_py - (pi/2));
q_X  = rad2deg(alpha_X + beta_X - (pi/2));

% Check for complex numbers
if ~isreal(q_nY), q_nY = NaN; end
if ~isreal(q_pY), q_pY = NaN; end
if ~isreal(q_X), q_X = NaN; end

% Display results
fprintf('The Inverse Kinematic Base Motor Angles Are:\n');
fprintf('-Y = %.8f deg\n', 90 - q_nY);
fprintf('+Y = %.8f deg\n', 90 - q_pY);
fprintf(' X = %.8f deg\n', 90 - q_X);
