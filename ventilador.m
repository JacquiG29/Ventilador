% Proyecto del ventilador mecánico
% Jacqueline Guarcax y Gabriela Iriarte
% 28/04/2020
% -------------------------------------------------------------------------
% PERFILES DE AMBO VENT
% -------------------------------------------------------------------------
pos = [  0,  0,  1,  2,  4,  6,  8, 10, 13, 15, 18, 21, 25, 28, 31, 35, 38, 42, 46, 50,...
        54, 57, 61, 66, 70, 74, 78, 82, 86, 91, 95, 99,104,108,112,117,121,125,130,134,...
       138,143,147,151,156,160,164,169,173,177,181,185,189,194,198,201,205,209,213,217,...
       220,224,227,230,234,237,240,242,245,247,249,251,253,254,255,255,255,255,255,255,...
       255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,...
       255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,...
       255,255,255,255,255,255,255,254,253,252,250,248,246,244,241,238,235,232,229,225,...
       222,218,214,210,206,202,198,193,189,184,180,175,171,166,162,157,152,148,143,138,...
       134,129,124,120,115,111,106,102, 97, 93, 89, 84, 80, 76, 72, 68, 64, 61, 57, 54,...
        50, 47, 44, 41, 38, 36, 33, 31, 29, 27, 25, 23, 22, 20, 19, 17, 16, 15, 13, 12,...
        11, 10,  9,  8,  7,  6,  6,  5,  4,  3,  3,  2,  2,  1,  1,  1,  0,  0,  0,  1,...
         1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,  2,  2,  1,  1,  1,  1,  1,  1,...
         1,  1,  1,  0,  0,  0,  0,  0,  0,  0];

vel = [129,132,134,136,137,139,140,141,142,143,143,144,144,145,146,146,146,147,147,147,...
       148,148,148,148,149,149,149,149,149,149,150,150,150,150,150,150,150,150,150,150,...
       150,150,150,150,150,149,149,149,149,149,149,148,148,148,148,147,147,147,146,146,...
       146,145,144,144,143,143,142,141,140,139,137,136,134,132,129,128,128,128,128,128,...
       128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,...
       128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,...
       128,128,128,128,128,127,125,123,121,120,119,117,116,115,114,113,112,111,111,110,...
       109,109,108,108,107,107,106,106,106,106,105,105,105,105,105,105,105,105,105,105,...
       105,105,105,105,105,105,106,106,106,107,107,107,108,108,109,109,110,110,111,111,...
       112,113,113,114,115,116,117,118,118,119,119,120,120,120,121,121,121,122,122,122,...
       123,123,123,124,124,124,124,125,125,125,125,125,126,126,126,126,126,127,127,127,...
       127,127,127,127,128,128,128,128,128,128,128,128,128,128,128,128,129,129,129,129,...
       129,129,129,129,129,128,128,128,128,128];
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
k = 1478; %3462.75; % cte. del resorte del Ambú (experimental)
D2o = 5; % Damping ambú FALTA ESTIMAR
K2 = k*N1N2^2; % K reflejada
D2 = D2o*N1N2^2; % Damping ambú reflejado

% ------- BRAZO ------- 
Dp = 30;% fricción de engranes FALTA ESTIMAR 
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
Kf = 50;% K fumada FALTA ESTIMAR
m = 0.148; % kg
g = 9.81; % m/s2
ell = 0.29; % metros

% -------------------------------------------------------------------------
% tf del motor
%SYS = tf(alpha,[1 (Dm/Jm)+alpha*Kb 0]);
%step(SYS)
% -------------------------------------------------------------------------

% -------------------------------------------------------------------------
% CAMPOS VECTORIALES DEL SISTEMA
% -------------------------------------------------------------------------
% x = [x(1) x(2) x(3) x(4)]
% u = u
f = @(x,u) [x(2);...
            alpha*u-(Dm/Jm + alpha*Kb)*x(2);... 
            x(4);...
            (Kf*x(2)-x(4)*(D2)-K2*x(3)-m*g*ell*0.5*sin(x(3)))*(1/Jb2)];
        
h = @(x, u) [x(1);...
              0;... 
             x(3);...
              0];
%% 
% -------------------------------------------------------------------------
% SIMULACIÓN DE VARIABLES DE ESTADO
% -------------------------------------------------------------------------
%Parámetros de la simulación
dt = 0.001; % período de muestreo
t0 = 0; % tiempo inicial
tf = 20; % tiempo final
N = (tf-t0)/dt; % número de iteraciones

%Inicialización y condiciones iniciales
x0 = [0,0,0,0]';
u0 = 0;
x = x0; % vector de estado 
u = u0; % vector de entradas
% Arrays para almacenar las trayectorias de las variables de estado,
% entradas y salidas del sistema
X = zeros(numel(x),N+1);
U = zeros(numel(u),N+1);
% Inicialización de arrays
X(:,1) = x0;
U(:,1) = u0;

for n = 0:N
    % Se define la entrada para el sistema. Aquí debe colocarse el
    % controlador al momento de querer estabilizar el sistema
    %u = [0; 0];
    [A,B,C,D]=linloc(f,h,x_lin,u_lin);
    u = 1;%5*sin(2*pi*dt*n);    

    % Se actualiza el estado del sistema mediante una discretización por 
    % el método de Runge-Kutta (RK4)
    k1 = f(x, u);
    k2 = f(x+(dt/2)*k1, u);
    k3 = f(x+(dt/2)*k2, u);
    k4 = f(x+dt*k3, u);
    x = x + (dt/6)*(k1+2*k2+2*k3+k4);
    
    % Se guardan las trayectorias del estado y las entradas
    X(:,n+1) = x;
    U(:,n+1) = u;
    
    
    
end
t = t0:dt:tf;
figure()
plot(t,X(1,:),t,X(2,:),t,X(3,:),t,X(4,:),t,U(1,:));
%figure(3)
%plot(t,X(4,:));
leg1 = legend('$\theta_m$','$\dot{\theta_m}$','$\theta _p$','$\dot{\theta _p}$','$V_{in}$');
set(leg1,'Interpreter','latex');
%%
% -------------------------------------------------------------------------
% LINEALIZACIÓN 
% -------------------------------------------------------------------------
x_lin=[0,0,0,0]'
u_lin=0;


% Verificación de la controlabilidad
size_a = size(A);
n = size_a(1);
rank_calculado = rank(ctrb(A,B));

if rank_calculado == n
    disp("¡Felicidades! Matriz sí es controlable.")
else
    disp("¡Rayos! Matriz NO es controlable, intenta de nuevo.")
end

%% PRUEBA LQR
% Q=eye(4);%matriz de nxn # de var. de estado
% R=eye(1);%Numero de entradas
% K = lqr(A,B,Q,R); 
% e_lqr=eig(A-B*K);

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
