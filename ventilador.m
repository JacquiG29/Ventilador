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
load('jaja'); % Importamos la variable "prueba" que almacena el perfil con 
% muestreado a 0.001 para que coincida con la animación.
Ts=0.01;%Tiempo de muestreo
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
% x = [ motor speed; arm angle; arm speed]
% x = [     x(2)       x(3)       x(4)   ]
% u = e(t)

siso = 1;
if (siso == 0)
    
    f = @(x,u) [alpha*u-(Dm/Jm + alpha*Kb)*x(1);...
        x(3);...
        (Kf*x(1)-x(3)*(D2)-K2*x(2)-m*g*ell*0.5*sin(x(2)))*(1/Jb2)];
    h = @(x, u) [x(2);...
                 x(3)];
    
elseif (siso == 1)
    f = @(x,u) [alpha*u-(Dm/Jm + alpha*Kb)*x(1);...
        x(3);...
        (Kf*x(1)-x(3)*(D2)-K2*x(2)-m*g*ell*0.5*sin(x(2)))*(1/Jb2)];
    h = @(x, u) x(2);
end
%f2=@(s)dynamics(s,)
%% ------------------------------------------------------------------------
% LINEALIZACIÓN
% -------------------------------------------------------------------------
r0 = 1; %referencia
r = r0;

% Campo vectorial de los integradores
f_sigma = @(x,u,r) h(x,u)- r; %Definido en leccion de LQI
% Campo vectorial aumentado
F = @(xi,u,r) [f(xi(1:3),u); f_sigma(xi(1:3),u,r)];

% fs=@(s) dynamics([s,theta_des,0],Ks);
% vm=fsolve(fs,0);
% xss=[vm,theta_des,0]
% uss=Kf*vm;

xss = [100,pi/4,0]'; % No importa mucho el punto de operación.
uss = Kf*xss(1); % Sí está bien

%xss = [5,  pi/4, 0]';          % Se define el punto de operación
%fu = @(u) f(xss, u);              
%uss = fsolve(fu, 1000);             % Se encuentra el uss para el punto de operación

[A,B,C,D] = linloc(f,h,xss,uss); % Linealización local
sis = ss(A,B,C,D); % Espacio de estados a partir de las matrices
%% Verificación de la controlabilidad

size_a = size(A);
n = size_a(1);
rank_calculado = rank(ctrb(A,B));

if rank_calculado == n
    disp("¡Felicidades! Matriz sí es controlable.")
else
    disp("¡Rayos! Matriz NO es controlable, intenta de nuevo.")
end
%% Verificación de observabilidad

OB = rank(obsv(A,C));

if OB == n
    disp("¡Felicidades! Matriz sí es observable.")
else
    disp("¡Rayos! Matriz NO es observable, intenta de nuevo.")
end
%% ------------------------------------------------------------------------
% CONTROLADORES
% -------------------------------------------------------------------------

% 1 - Pole Placement
% 2 - LQI

controlador = 1;

if (controlador == 1)
    % -------------------------------------------------------------------------
    % Diseño de control por pole placement
    % -------------------------------------------------------------------------
    if (siso == 1)
        polo_fav = [-1000,-100+15i,-100-15i];
        K = place(A, B, polo_fav);
        Nbar = rscale(sis,K);
        % Observador
        p2 = polo_fav-1000;
        L = place(A',C',p2)';
        
        A_obs = A-L*C;
        B_obs = [B L];
        C_obs = eye(3);
        D_obs = zeros(3,2);
        
    elseif(siso == 0)
    disp("No hay LQR MIMO\n")
    end
    
elseif (controlador == 2)
    % LQI
    if (siso == 1)
        Q = [0.3    0    0   0;...
            0    0.01    0   0;...
            0    0    0.01   0;...
            0    0    0   500000];
        R = 0.001;
        Klqi = lqi(sis,Q,R);
        
        % Observador
        P = are(A, B*inv(R)*B', Q(1:3,1:3));
        R_L=1;
        L=P*C'*(1/R_L);
        Cr=[0 0 1];
        
    elseif(siso == 0)
        Q = [0.3    0    0   0;...
            0    0.01    0   0;...
            0    0    0.01   0;...
            0    0    0   500000];
        R = 0.001;
        sis = ss(A,B,C(1,:),zeros(1,size(B,2)));
        Klqi = lqi(sis,Q,R);
        Klqi = [Klqi Klqi(end)];
    end
end


%% ------------------------------------------------------------------------
% SIMULACIÓN DE VARIABLES DE ESTADO
% -------------------------------------------------------------------------

%% ------------------------------------------------------------------------
% LQI - Referencia de posición
% -------------------------------------------------------------------------
if (controlador == 2)
    
    
    disp("Falta agregar simulación LQI\n")
    
    
    
end
%% ------------------------------------------------------------------------
% LQR - Referencia de posición
% -------------------------------------------------------------------------
% Init cond
if (controlador == 1)
    x0 = 0.1 .* ones(3,1);
    
    % Parámetros de la simulación
    dt = 0.001; % 1ms = período de muestreo
    t0 = 0; % tiempo inicial
    tf1 = 5; % tiempo final
    N = (tf1-t0)/dt; % número de iteraciones
    
    % Discretización por Euler
    use_euler = false; % verdadero si se quiere discretizar con euler, falso si
    % se quiere ZOH
    if(use_euler)
        % Completar las matrices discretas obtenidas mediante Euler
        Ad = eye(3)+A*dt;
        Bd = B*dt;
        Cd = C;
        Dd = D;
    else
        % Completar las matrices discretas obtenidas mediante ZOH, las
        % funciones integral y expm de MATLAB pueden serle de utilidad
        Ad = expm(A*dt);
        
        fun=@(s)expm(s*A);%definir funcion a integrar
        Bd = (integral(fun,0,dt,'ArrayValued',true))*B;%opcion elegida porque se trabaja con matrices
        Cd = C;
        Dd = D;
        %integral y expm para funciones
    end
    
    % Simulación del sistema
    x = x0; % Inicialización del vector de estado
    xhat = zeros(3,1); % Inicializamos el estimador
    X = x; % Array para guardar trayectorias de variables de estado
    Xhat = xhat; % Array para guardar trayectorias del estimado de x
    y = Cd*x; % Inicialización de la salida real
    yhat = Cd*xhat; % Inicialización de la salida estimada
    U = zeros(1,1); % Array para guardar entradas al sistema
    Y = y; % yhat para ver cómo funciona con observador
    
    % Parámetros para la animación
    max_angle = pi/3;
    min_angle = pi/4;
    pendrod = plot([0,0], [0,0], 'Color', [0.12, 0.12, 0.12], 'LineWidth', 5);
    xlim([0 0.3]);
    ylim([0 0.3]);
    pos = [0.1 0 0.1 0.1];
    rectangle('Position',pos,'Curvature',[1 1],'EdgeColor',[0 .5 .5],'FaceColor',[0 .5 .5])
    xlabel('$x$ (m)','Interpreter','latex','FontSize', 16);
    ylabel('$y$ (m)', 'Interpreter', 'latex', 'FontSize', 16);
    
    for n = 0:N-2
        
        r = prueba(n+1,2); % Asignamos el perfil de posición como referencia
        u = r*Nbar-K*xhat; % Aplicamos control LQR
        
        % Simulación del sistema para el observador
        xhat = Ad*xhat + Bd*u + L*(y - yhat)*dt;
        yhat = Cd*xhat;
        
        % Sistema real
        x = Ad*x + Bd*u;
        y = Cd*x;
        
        % Recopilación de datos
        X = [X, x];
        Xhat = [Xhat, xhat];
        U = [U, u];
        Y = [Y, y]; % Y, yhat para ver cómo funciona con observador
        
        x2_mapped = map(0,1,min_angle,max_angle,x(2));
        pendrod.XData = [ell*sin(x2_mapped) 0];%[0,ell*sin(x)];
        pendrod.YData = [ell*cos(x2_mapped) 0];%[0,ell*cos(x)];
        title('Ventilador con LQR y referencia de posición')
        drawnow limitrate
        pause(dt);
        
    end
    
    % Graficamos los resultados
    t = t0:dt:tf1;
    
    figure;
    plot(t(1:5000), Y', 'LineWidth', 1);
    hold on;
    plot(prueba(:,1), prueba(:,2), '--','LineWidth', 1);
    hold off;
    title('Trayectoria de la salida');
    xlabel('$t$','Interpreter','latex','FontSize', 16);
    ylabel('$y(t)$', 'Interpreter', 'latex', 'FontSize', 16);
    l = legend('$y(t)$', '$r(t)$', 'Location', 'southeast');
    set(l, 'Interpreter', 'latex', 'FontSize', 12);
    
    figure;
    plot(t(1:5000), X', 'LineWidth', 1);
    hold on;
    ax = gca;
    ax.ColorOrderIndex = 1;
    plot(t(1:5000), Xhat', '--', 'LineWidth', 1);
    hold off;
    title('Trayectorias de variables de estado');
    xlabel('$t$','Interpreter','latex','FontSize', 16);
    ylabel('$\mathbf{x}(t)$', 'Interpreter', 'latex', 'FontSize', 16);
    l = legend('$x_1(t)$', '$x_2(t)$', '$x_3(t)$', '$\hat{x_1}(t)$', '$\hat{x_2}(t)$', ...
        '$\hat{x_3}(t)$', 'Location', 'southeast');
    set(l, 'Interpreter', 'latex', 'FontSize', 12);
    
    figure;
    plot(t(1:5000), U', 'LineWidth', 1);
    title('Entrada del sistema con respecto al tiempo');
    xlabel('$t$','Interpreter','latex','FontSize', 16);
    ylabel('$\mathbf{u}(t)$', 'Interpreter', 'latex', 'FontSize', 16);
    l = legend('$u_1(t)$','Location', 'southeast');
    set(l, 'Interpreter', 'latex', 'FontSize', 12);
end
