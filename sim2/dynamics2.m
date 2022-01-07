function ret = dynamics2(t, x, u, H, C, g, qd, qd_dot, qd_ddot, s)

    % x1 = q = [q1 q2]
    % x2 = q_dot = [q1_dot q2_dot]
    % x = [x1; x2] = [q1 q2; q1_dot q2_dot]
    % x_dot = [q1_dot q2_dot; q1_ddot q2_ddot]
    
    % x1_dot = x2
    % x2_dot = 1/H * u - C/H * x2 - g/H

    x1 = [x(1); x(3)]; % [q1, q2]
    x2 = [x(2); x(4)]; % [q1_dot, q2_dot]

    s_cur = s(x1, x2, qd(t), qd_dot(t));
    
    dx = zeros(2, 2);

    dx(1, :) = x2';
    dx(2, :) = (H(x1(1), x1(2)) \ u(x1, x2, qd(t), qd_dot(t), qd_ddot(t), s_cur) - ...
                H(x1(1), x1(2)) \ C(x1(1), x1(2), x2(1), x2(2)) * x2 - ...
                H(x1(1), x1(2)) \ g(x1(1), x1(2)))';
    
    ret = [dx(1, 1); dx(2, 1); dx(1, 2); dx(2, 2)];
end