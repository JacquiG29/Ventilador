function [DX, DY] = phaseportrait2d(X, Y, dim_u, normalize, parameters)
    [I, J] = size(X);
    DX = zeros(I,J);
    DY = zeros(I,J);
    u = zeros(dim_u, 1);
    
    for i = 1:I
        for j = 1:J
            x = [X(i,j); Y(i,j)];
            f = dynamics(x, u, parameters);
            if(normalize) f = f/norm(f); end
            DX(i,j) = f(1); 
            DY(i,j) = f(2);
        end
    end
end