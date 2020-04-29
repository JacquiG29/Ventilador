% La función f espera como parámetros
% x vector de dos entradas
% x = [x1 x2]
% y un escalar u
function f = dynamics(x, u, parameters)

    parameters.m = 0.148;%*1.5; 
    parameters.ell = 0.29; 
    parameters.b = 0.1111; % Probé variar este parámetro y lo más que deja es ese valor. Ahí está perfecto.
    parameters.g = 9.81;
    parameters.K2 = 1/41;%97.3644/90;
    parameters.Jb2 = 0.0012;
    parameters.xe = [0,0]';
    parameters.ue = 0;

    m = parameters.m; 
    ell = parameters.ell; 
    D2 = parameters.b;
    b = parameters.b;
    g = parameters.g;
    K2 = parameters.K2;
    Jb2 = parameters.Jb2;
    
    f = [x(2);(u-x(2)*D2-K2*x(1)-m*g*ell*0.5*sin(x(1)))*(1/Jb2)];
    
    %f = [x(2); -(g/ell)*sin(x(1)) - (b/(m*ell^2))*x(2) + (1/(m*ell^2))*u];
end