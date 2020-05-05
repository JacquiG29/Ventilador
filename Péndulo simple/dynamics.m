% La función f espera como parámetros
% x vector de dos entradas
% x = [x1 x2]
% y un escalar u
function f = dynamics(x, u, parameters)

    parameters.m = 0.148;%*1.5;
    parameters.ell = 0.29;
    parameters.b = 0.1111; % Probé variar este parámetro y lo más que deja es ese valor. Ahí está perfecto.
    parameters.g = 9.81;
    %parameters.K2 = 1/41;%97.3644/90;
    parameters.Kambu = 97.3644;
    parameters.K2 = 164.2222;
    parameters.Jb2 = 0.0012;
    parameters.xe = [0,0]';
    parameters.ue = 0;
    parameters.Kf = 1;

    m = parameters.m; 
    ell = parameters.ell; 
    D2 = parameters.b;
    b = parameters.b;
    g = parameters.g;
    Kambu = parameters.Kambu;
    Jb2 = parameters.Jb2;
    Kf=parameters.Kf;
    
    f = [x(2);(Kf*u-x(2)*D2-Kambu*x(1)-m*g*ell*0.5*sin(x(1)))*(1/Jb2)];
    %(Kf*x(2)-x(4)*(D2)-K2*x(3)-m*g*ell*0.5*sin(x(3)))*(1/Jb2)
    
    %f = [x(2); -(g/ell)*sin(x(1)) - (b/(m*ell^2))*x(2) + (1/(m*ell^2))*u];
end