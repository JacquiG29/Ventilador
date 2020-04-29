% Proyecto del ventilador mecánico
% Jacqueline Guarcax y Gabriela Iriarte
% 28/04/2020
% -------------------------------------------------------------------------
% PARÁMETROS DEL SISTEMA
% -------------------------------------------------------------------------
% -------- Engranes -------
sf = 1.5; % Security factor
N1N2 = 1/3; % Relación de engranajes

diam2 = 157.547/(10*100); % m
diam1 = 51.871/(10*100); % m
r2 = diam2/2; % m
r1 = diam1/2; % m
h = 0.01; % m  (10 mm)
V2 = pi*h*r2^2; % m3
V1 = pi*h*r1^2;% m3
d = 1180; % kg/m3
m1 = d*V1; % kg
m2 = d*V2; % kg
I1 = 0.5*m1*r1^2; 
I2 = 0.5*m2*r2^2; 
wnl = 3*pi/2; % rad/s
J1 = I1*wnl*sf; % J del engrane del lado del motor
J2 = I2*wnl*sf; % J del engrane del lado del péndulo

% -------- AMBU -------
k = 876.28; % cte. del resorte del Ambú (experimental)
D2o = 1; % Damping ambú FALTA ESTIMAR
K2 = k*N1N2^2; % K reflejada
D2 = D2o*N1N2^2; % Damping ambú reflejado

% ------- BRAZO ------- 
Dp = 1;% fricción de engranes FALTA ESTIMAR 
Dp2 = Dp*N1N2^2; % fricción de engranes reflejado

% J del péndulo. Cambiar este para hacerlo no lineal, etc.
modelo_pendulo = 1;
if(modelo_pendulo == 1)
    Jb = 0.00362641*sf; % kg/m2
elseif(modelo_pendulo == 2)
    %Jb = ; 
elseif(modelo_pendulo == 3)
    %Jb = ; 
end
Jb2 = (J2+Jb)*N1N2^2+J1; % J del péndulo reflejado

% ------- MOTOR -------
Da = 0.002584*sf; % damping de armadura de motor
Dm = Da + Dp2; % Damping motor
Ja = 2.45*10^-3; % kg*m2 J armadura
Jm = Ja*sf; % J motor

% Del datasheet del motor:
ea = 13.5; % V
% wnl está arriba porque se necesitaba para la aproximación de J en
% engranajes
Tstall = 29; % Nm

alpha = Tstall/(ea*Jm);
Kb = ea/wnl;
Kf = 1;% K fumada FALTA ESTIMAR
m = 0.148; % kg
g = 9.81; % m/s2
ell = 0.29; % metros

% -------------------------------------------------------------------------
% tf del motor
SYS = tf(alpha,[1,(Dm/Jm)+alpha*Kb,0]);
step(SYS)
% -------------------------------------------------------------------------

% -------------------------------------------------------------------------
% CAMPOS VECTORIALES DEL SISTEMA
% -------------------------------------------------------------------------
% x = [x(1) x(2) x(3) x(4)]
% u = u
f = @(x,u) [x(2);...
            alpha*u-(Dm/Jm + alpha*Kb)*x(2);... 
            x(4);...
            (x(4)*(Kf-D2)-K2*x(3)-m*g*ell*0.5*sin(x(3)))*(1/Jb2)];
        
%h = @(x, u) 5 ; %FALTA ESTIMAR
%%
% -------------------------------------------------------------------------
% LINEALIZACIÓN DEL PÉNDULO
% -------------------------------------------------------------------------
f1 = @(x,u) [x(1)*u;...
            (x(1)*(Kf-D2)-K2*x(2)-m*g*ell*0.5*sin(x(2)))*(1/Jb2)];
        
h1 = @(x, u)  atan(x(1)); 

xss=[pi/6,0]'
uss=m*g*ell*sin(pi/6);
[A,B,C,D]=linloc(f1,h1,xss,uss);

SYS2 = ss(A,B,C,D);
step(SYS2)

% Verificación de la controlabilidad
size_a = size(A);
n = size_a(1);
rank_calculado = rank(ctrb(A,B));

if rank_calculado == n
    disp("¡Felicidades! Matriz sí es controlable.")
else
    disp("¡Rayos! Matriz NO es controlable, intenta de nuevo.")
end

%%
% -------------------------------------------------------------------------
% SIMULACIÓN
% -------------------------------------------------------------------------
% Parámetros de la simulación
t0 = 0;
tf = 10; % tiempo de simulación
dt = 0.01; % tiempo de muestreo
K = (tf - t0) / dt;

% Condición inicial
x0 = [r0; 0; theta0; w0];

% Inicialización
x = x0; % vector de estado
u = [0; 0]; % vector de entradas
X = x; % array para almacenar las trayectorias de las variables de estado
U = u; % array para almacenar la evolución de las entradas
