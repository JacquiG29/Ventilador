function dfx = linearize_dynamics(x, u, xe, ue, parameters)
    [n, ~] = size(x);
    [m, ~] = size(u);
    
    dA = zeros(n,n);
    dB = zeros(n,m);
    delta = 1e-6;
    f = dynamics(xe, ue, parameters);
    
    for i = 1:n
        dx = zeros(n,1);
        dx(i) = delta;
        df = dynamics(xe+dx, ue, parameters);
        dA(:,i) = (df-f)/delta;
    end
    
    for j = 1:m
        du = zeros(m,1);
        du(j) = delta;
        df = dynamics(xe, ue+du, parameters);
        dB(:,j) = (df-f)/delta;
    end

    dfx = dA*(x-xe) + dB*(u-ue);
end