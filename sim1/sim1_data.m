% known params
m1 = 6;
m2 = 4;
l1 = 0.5;
l2 = 0.4;
g_const = 9.81;

% simulation params
lc1 = 0.2;
lc2 = 0.1;
I1 = 0.43;
I2 = 0.05;
ml = 0.5;

% unknown params - max estimation error
lc1_tilde = 0.3;
lc2_tilde = 0.25;
I1_tilde = 0.48;
I2_tilde = 0.14;
ml_tilde = 2;

%% simulation matrices

% H simulation
h11 = @(q1, q2) m1 * lc1^2 + m2 * (lc2 ^ 2 + l1^2 + 2 * l1 * l2 * cos(q2)) + ml * (l2^2 + l1^2 +2 * l1 * l2 * cos(q2)) + I1 + I2;
h12 = @(q1, q2) m2 * lc2 * (lc2 + l1 * cos(q2)) + ml * l2 * (l2 + l1 * cos(q2)) + I2;
h22 = @(q1, q2) lc2^2 * m2 + l2^2 * ml + I2;

H = @(q1, q2) [h11(q1, q2) h12(q1, q2); h12(q1, q2) h22(q1, q2)];

% C simulation
c11 = @(q1, q2, q1_dot, q2_dot) -l1 * (m2 * lc2 + ml * l2) * sin(q2) * q2_dot;
c12 = @(q1, q2, q1_dot, q2_dot) -l1 * (m2 * lc2 + ml * l2) * sin(q2) * (q2_dot + q1_dot);
c21 = @(q1, q2, q1_dot, q2_dot) l1 * (m2 * lc2 + ml * l2) * sin(q2) * q1_dot;
c22 = @(q1, q2, q1_dot, q2_dot) 0;

C = @(q1, q2, q1_dot, q2_dot) [c11(q1, q2, q1_dot, q2_dot) c12(q1, q2, q1_dot, q2_dot); c21(q1, q2, q1_dot, q2_dot) c22(q1, q2, q1_dot, q2_dot)];

% g simulation
g1 = @(q1, q2) (m2 * lc2 + ml * l2) * g_const * cos(q1 + q2) + (m2 * l1 + ml * l1 + m1 * lc1) * g_const * cos(q1);
g2 = @(q1, q2) (m2 * lc2 + ml * l2) * g_const * cos(q1 + q2);

g = @(q1, q2) [g1(q1, q2); g2(q1, q2)];

%% controller matrices

% H_tilde
h11_tilde = @(q1, q2) m1 * lc1_tilde + m2 * (lc2_tilde + 2 * l1 * cos(q2) * lc2_tilde) + ...
            ml_tilde * (l1^2 + l2^2 + 2 * l1 * l2 * cos(q2)) + I1_tilde + I2_tilde;
h12_tilde = @(q1, q2) m2 * (lc2_tilde + l1 * cos(q2) * lc2_tilde) + ...
            l2 * ml_tilde * (l2 + l1 * cos(q2)) + I2_tilde;
h22_tilde = @(q1, q2) ...
              m2 * lc2_tilde + l2^2 * ml_tilde + I2_tilde;

e1_max = @(q1, q2) ...
         [h11_tilde(q1, q2) h12_tilde(q1, q2); ...
          h12_tilde(q1, q2) h22_tilde(q1, q2)];

% C_tilde
c11 = @(q1, q2, q1_dot, q2_dot) ...
        -q2_dot;
    
c12 = @(q1, q2, q1_dot, q2_dot) ...
        -(q2_dot + q1_dot);
    
c21 = @(q1, q2, q1_dot, q2_dot) ...
        q1_dot;
    
e2_max = @(q1, q2, q1_dot, q2_dot) ...
          (m2 * lc2_tilde + ml_tilde * l2) * (l1 * sin(q2)) * ... 
          [c11(q1, q2, q1_dot, q2_dot) c12(q1, q2, q1_dot, q2_dot); ...
           c21(q1, q2, q1_dot, q2_dot) 0];
      
% g_tilde
g1_tilde = @(q1, q2) ...
            (m2 * lc2_tilde + ml_tilde * l2) * g_const * cos(q1 + q2) + ... 
            (ml_tilde * l1 + m1 * lc1_tilde) * g_const * cos(q1);

g2_tilde = @(q1, q2) ...
            (m2 * lc2_tilde + ml_tilde * l2) * g_const * cos(q1 + q2);

e3_max = @(q1, q2) [g1_tilde(q1, q2); 
                    g2_tilde(q1, q2)];
                
% e, e_dot
e = @(q, qd) q - qd;
e_dot = @(q_dot, qd_dot) q_dot - qd_dot;
                
% r
L = [2 0; 0 5];
C_const = 10;
r = @(q, q_dot, qd, qd_dot, qd_ddot) ... 
      e1_max(q(1), q(2)) * abs(qd_ddot - L * e_dot(q_dot, qd_dot)) + ... 
      e2_max(q(1), q(2), q_dot(1), q_dot(2)) * abs(q_dot) + e3_max(q(1), q(2)) + C_const;

% parameter estimations
lc1_hat = 0.25;
lc2_hat = 0.15;
I1_hat = 0.2;
I2_hat = 0.1;
ml_hat = 1;

% C_hat
c11_hat = @(q1, q2, q1_dot, q2_dot) -l1 * (m2 * lc2_hat + ml_hat * l2) * sin(q2) * q2_dot;
c12_hat = @(q1, q2, q1_dot, q2_dot) -l1 * (m2 * lc2_hat + ml_hat * l2) * sin(q2) * (q2_dot + q1_dot);
c21_hat = @(q1, q2, q1_dot, q2_dot) l1 * (m2 * lc2_hat + ml_hat * l2) * sin(q2) * q1_dot;
c22_hat = @(q1, q2, q1_dot, q2_dot) 0;

C_hat = @(q, q_dot) ...
        [c11_hat(q(1), q(2), q_dot(1), q_dot(2)) c12_hat(q(1), q(2), q_dot(1), q_dot(2)); ...
         c21_hat(q(1), q(2), q_dot(1), q_dot(2)) c22_hat(q(1), q(2), q_dot(1), q_dot(2))];
 
% H_hat
h11_hat = @(q1, q2) m1 * lc1_hat^2 + m2 * (lc2_hat ^ 2 + l1^2 + 2 * l1 * l2 * cos(q2)) + ml_hat * (l2^2 + l1^2 +2 * l1 * l2 * cos(q2)) + I1_hat + I2_hat;
h12_hat = @(q1, q2) m2 * lc2_hat * (lc2_hat + l1 * cos(q2)) + ml_hat * l2 * (l2 + l1 * cos(q2)) + I2_hat;
h22_hat = @(q1, q2) lc2_hat^2 * m2 + l2^2 * ml_hat + I2_hat;

H_hat = @(q) ...
         [h11_hat(q(1), q(2)) h12_hat(q(1), q(2)); ...
          h12_hat(q(1), q(2)) h22_hat(q(1), q(2))];
      
% g_hat
g1_hat = @(q1, q2) (m2 * lc2_hat + ml_hat * l2) * g_const * cos(q1 + q2) + ...
          (m2 * l1 + ml_hat * l1 + m1 * lc1_hat) * g_const * cos(q1);
g2_hat = @(q1, q2) (m2 * lc2_hat + ml_hat * l2) * g_const * cos(q1 + q2);

g_hat = @(q) [g1_hat(q(1), q(2)); g2_hat(q(1), q(2))];

% s
s = @(q, q_dot, qd, qd_dot) ...
        e_dot(q_dot, qd_dot) + L * e(q, qd);
    
% controller
e_sat = 0.05;
u = @(q, q_dot, qd, qd_dot, qd_ddot, s_cur) ...
        C_hat(q, q_dot) * q_dot + g_hat(q) + H_hat(q) * qd_ddot - ... 
        H_hat(q) * L * e_dot(q_dot, qd_dot) - ... 
        r(q, q_dot, qd, qd_dot, qd_ddot) .* [sat(s_cur(1), e_sat); sat(s_cur(2), e_sat)];