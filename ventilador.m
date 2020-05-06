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
% load('variables_linloc') % Importamos el workspace de los perfiles de
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
    motor_tf = tf(alpha,[1 (Dm/Jm)+alpha*Kb 0]); % Sacamos la función de t.
    step(motor_tf) % Graficamos su respuesta
end

% -------------------------------------------------------------------------
% CAMPOS VECTORIALES DEL SISTEMA
% -------------------------------------------------------------------------
% Significado de las variables de estado y el vector de entradas
% x = [motor angle; motor speed; arm angle; arm speed]
% x = [   x(1)          x(2)       x(3)       x(4)   ]
% u = e(t)

f = @(x,u) [x(2);...
            alpha*u-(Dm/Jm + alpha*Kb)*x(2);... 
            x(4);...
            (Kf*x(2)-x(4)*(D2)-K2*x(3)-m*g*ell*0.5*sin(x(3)))*(1/Jb2)];
        
h = @(x, u) [x(1);...
              0;... 
             x(3);...
              0];
          
%% ------------------------------------------------------------------------
% SIMULACIÓN DE VARIABLES DE ESTADO
% -------------------------------------------------------------------------
% Parámetros de la simulación
dt = 0.001; % 1ms = período de muestreo
t0 = 0; % tiempo inicial
tf = 10; % tiempo final
N = (tf-t0)/dt; % número de iteraciones
t = t0:dt:tf; % Tiempo para el ploteo

% Para ver la gráfica colocar un 1 en plot_sis
plot_sis = 1;

% Cambiar tipo de estímulo para u en la etapa de fumada de parámetros
tipo_graf = 'step'; % 'seno' o 'step'

%Inicialización y condiciones iniciales
x0 = [0,0,0,0]';
u0 = 0;
x = x0; % vector de estado init
u = u0; % vector de entradas init

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
    %[A,B,C,D]=linloc(f,h,x_lin,u_lin);% En estos mis puntos de operacion
    %serian los valores de array de pos 
    if (strcmp(tipo_graf,'seno'))
        u = 5*sin(2*pi*dt*n);
    elseif (strcmp(tipo_graf,'step'))
        u = 1;
    end

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


if (plot_sis == 1)
    figure()
    plot(t,X(1,:),t,X(2,:),t,X(3,:),t,X(4,:),t,U(1,:));
    %figure(3)
    %plot(t,X(4,:));
    leg1 = legend('$\theta_m$','$\dot{\theta_m}$','$\theta _p$','$\dot{\theta _p}$','$V_{in}$');
    set(leg1,'Interpreter','latex');
end
%%
% % Verificación de la controlabilidad
% size_a = size(A);
% n = size_a(1);
% rank_calculado = rank(ctrb(A,B));
% 
% if rank_calculado == n
%     disp("¡Felicidades! Matriz sí es controlable.")
% else
%     disp("¡Rayos! Matriz NO es controlable, intenta de nuevo.")
% end
% 
% %% PRUEBA LQR
% % Q=eye(4);%matriz de nxn # de var. de estado
% % R=eye(1);%Numero de entradas
% % K = lqr(A,B,Q,R); 
% % e_lqr=eig(A-B*K);
% 
% %%
% % -------------------------------------------------------------------------
% % SIMULACIÓN
% % -------------------------------------------------------------------------
% % Parámetros de la simulación
% t0 = 0;
% tf = 10; % tiempo de simulación
% dt = 0.01; % tiempo de muestreo
% K = (tf - t0) / dt;
% 
% % Condición inicial
% x0 = [r0; 0; theta0; w0];
% 
% % Inicialización
% x = x0; % vector de estado
% u = [0; 0]; % vector de entradas
% X = x; % array para almacenar las trayectorias de las variables de estado
% U = u; % array para almacenar la evolución de las entradas
