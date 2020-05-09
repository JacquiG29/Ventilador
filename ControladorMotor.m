% ventilador.m
% Proyecto del ventilador mecánico
% Jacqueline Guarcax y Gabriela Iriarte
% 28/04/2020
% Descripción: Main del proyecto final de control 2: sistema de control
% para un ventilador mecánico de bajo costo basado en el prototipo AmboVent
% de Israel. 

%% ------------------------------------------------------------------------
% PERFILES DE AMBO VENT
% -------------------------------------------------------------------------
load('variables_linloc') % Importamos el workspace de los perfiles de
% AmboVent ya normalizados, listos como uss y xss para linealizar

%% ------------------------------------------------------------------------
% PARÁMETROS DEL SISTEMA
% -------------------------------------------------------------------------
load('params');

%% ------------------------------------------------------------------------
% PRUEBA DE ESCALÓN UNITARIO A LA FUNCIÓN DE TRANSFERENCIA DEL MOTOR
% -------------------------------------------------------------------------
% Para ver la respuesta colocar un 1 en probar_motor
probar_motor = 0;

if (probar_motor == 1)
    G0 = tf(alpha,[1 (Dm/Jm)+alpha*Kb 0]); % Sacamos la función de t.
    step(motor_tf) % Graficamos su respuesta
end

G0 = tf(alpha,[1,(Dm/Jm)+alpha*Kb,0]);
[A, B, C, D]= ssdata(G0)
Q=[1  0    0   ;...
   0    0.1   0   ;...
   0    0     35000];
R=0.1
sis=ss(A,B,C,D);
Klqi = lqi(sis,Q,R);

%%
% -------------------------------------------------------------------------
% CAMPOS VECTORIALES DEL SISTEMA
% -------------------------------------------------------------------------
f = @(x,u)[x(2);...
           u-x(2)*((Dm/Jm)+alpha*Kb)];       
h = @(x, u)x(1);
%f2=@(s)dynamics(s,)
%%
% -------------------------------------------------------------------------
% CONTROLADOR y LINEALIZACIÓN
% -------------------------------------------------------------------------
f_sigma = @(x,u,r) h(x,u) - r;
F = @(xi,u,r) [f(xi(1:2),u); f_sigma(xi(1:2),u,r)];

% fs=@(s) dynamics([s,theta_des,0],Ks);
% vm=fsolve(fs,0);
% xss=[vm,theta_des,0]
% uss=Kf*vm;

%xss=[100,pi/4,0]';
%uss=Kf*xss(1);%AUN NO ESTOY SEGURA DE ESTO

r0 = 1; %referencia
r = r0;
% 
% [A,B,C,D]=linloc(f,h,xss,uss);
% Q=[1  0    0   0;...
%    0    1  0   0;...
%    0    0    1   0;...
%    0    0    0   1000];
% R=1;
% 
% sis=ss(A,B,C,D);
% Klqi = lqi(sis,Q,R);

%Habilitar controlador en RK4,1 para habilitar
enable_control = 1;
%% ------------------------------------------------------------------------
% SIMULACIÓN DE VARIABLES DE ESTADO
% -------------------------------------------------------------------------
% Parámetros de la simulación
dt = 0.001; % 1ms = período de muestreo
t0 = 0; % tiempo inicial
tf1 = 5; % tiempo final
N = (tf1-t0)/dt; % número de iteraciones
t = t0:dt:tf1; % Tiempo para el ploteo

% Para ver la gráfica colocar un 1 en plot_sis
plot_sis = 1;

% Cambiar tipo de estímulo para u en la etapa de fumada de parámetros
tipo_graf = 'seno'; % 'seno' o 'step'

%Inicialización y condiciones iniciales
delta = 0.5;
%x0 = [xss(1)-delta,xss(2)-delta,xss(3)-delta]';
x0 = [-delta,-delta]';
u0 = 0;
sigma0 = h(x0, u0) - r0;
%sigma0=-0.5;
xi0 = [x0; sigma0];%Vector aumentado

x = x0; % vector de estado init
u = u0; % vector de entradas init
sigma = sigma0; % vector de integradores
xi = xi0; % vector de estado aumentado

% Arrays para almacenar las trayectorias de las variables de estado,
% entradas y salidas del sistema
X = zeros(numel(x),N+1);
U = zeros(numel(u),N+1);
S = zeros(numel(sigma),N+1);%sigma
XI = zeros(numel(xi), N+1);

% Inicialización de arrays
X(:,1) = x0;
U(:,1) = u0;
S(:,1) = sigma0;
XI(:,1) = xi0;

var=0;
for n = 0:N
    var=var+1;
    % Se define la entrada para el sistema. Aquí debe colocarse el
    % controlador al momento de querer estabilizar el sistema
    %u = [0; 0];
    %[A,B,C,D]=linloc(f,h,x_lin,u_lin);% En estos mis puntos de operacion
    %serian los valores de array de pos 
    if (strcmp(tipo_graf,'seno'))
        u = 5*sin(2*pi*dt*n);
    elseif (strcmp(tipo_graf,'step'))
        u = 1;
    end

    % Se actualiza el estado del sistema mediante una discretización por 
    % el método de Runge-Kutta (RK4)
    if (enable_control == 0)
        % Se actualiza la referencia (si aplica)
        k1 = f(x, u);
        k2 = f(x+(dt/2)*k1, u);
        k3 = f(x+(dt/2)*k2, u);
        k4 = f(x+dt*k3, u);
        x = x + (dt/6)*(k1+2*k2+2*k3+k4);
        
        % Se guardan las trayectorias del estado y las entradas
        X(:,n+1) = x;
        U(:,n+1) = u;
    end 
    
    if (enable_control == 1)
        % Se actualiza la referencia (si aplica)
        u = -Klqi*xi;%como xi incluye a sigma esto es igual a -K1*x-K2*sigma
        
        if (var==length(pos))%Busco recorrer el array como un bucle
            var = 0;
        end
        
        r = 8;%asigno el perfil de posición como referencia
        
        % Se actualiza el estado del sistema mediante una discretización por
        % el método de Runge-Kutta (RK4)
        k1 = F(xi, u, r);
        k2 = F(xi+(dt/2)*k1, u, r);
        k3 = F(xi+(dt/2)*k2, u, r);
        k4 = F(xi+dt*k3, u, r);
        xi = xi + (dt/6)*(k1+2*k2+2*k3+k4);
        
        % Se extrae el vector de estado original y el de los integradores
        x = xi(1:length(x));
        sigma = xi(1:length(sigma));
        
        % Se guardan las trayectorias del estado y las entradas
        X(:,n+1) = x;
        U(:,n+1) = u;
        S(:,n+1) = sigma;
        XI(:,n+1) = xi;
    end 
end


if (plot_sis == 1)
    figure()
    plot(t,X(1,:),t,X(2,:),t,U(1,:));
    %figure(3)
    %plot(t,X(4,:));
    leg1 = legend('${\theta_m}$','$\dot{\theta _m}$','$V_{in}$');
    set(leg1,'Interpreter','latex');
end