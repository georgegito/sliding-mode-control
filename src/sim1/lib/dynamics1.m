function ret = dynamics1(t, x, u, H, C, g, qd, s)
    
    % x1 = q = [q1, q2] : 
    x1 = [x(1); x(3)]; 
    
    % x2 = q_dot = [q1_dot, q2_dot] :
    x2 = [x(2); x(4)];

    s_cur = s(x1, x2, qd, [0; 0]);    
    dx = zeros(2, 2);

    % x1_dot = q_dot = x2 :
    dx(1, :) = x2';
    
    % x2_dot = q_ddot = 1/H * u - C/H * x2 - g/H :
    dx(2, :) = (H(x1(1), x1(2)) \ u(x1, x2, qd, [0; 0], [0; 0], s_cur) - ...
                H(x1(1), x1(2)) \ C(x1(1), x1(2), x2(1), x2(2)) * x2 - ...
                H(x1(1), x1(2)) \ g(x1(1), x1(2)))';

    % return [x1_dot(1) = q1_dot, x2_dot(1) = q1_ddot, 
            % x1_dot(2) = q2_dot, x2_dot(2) = q2_ddot]
    ret = [dx(1, 1); dx(2, 1); dx(1, 2); dx(2, 2)];            
end