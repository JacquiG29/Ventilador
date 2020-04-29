%% PAR�METROS A MODIFICAR
% -------------------------------------------------------------------------
% PAR�METROS DEL SISTEMA
% -------------------------------------------------------------------------
parameters.m = 0.148;%*1.5; 
parameters.ell = 0.29; 
parameters.b = 0.1111; % Prob� variar este par�metro y lo m�s que deja es ese valor. Ah� est� perfecto.
parameters.g = 9.81;
parameters.K2 = 1/41;%97.3644/90;
parameters.Jb2 = 0.0012;
parameters.xe = [0,0]';
parameters.ue = 0;

% etiquetas para las variables de estado y las entradas
statevars = {'$\theta$', '$\dot{\theta}$'};
inputvars = {'$\tau$'};

% -------------------------------------------------------------------------
% PAR�METROS DE LA SIMULACI�N
% -------------------------------------------------------------------------
t0 = 0; % tiempo inicial
tf = 20; % tiempo de simulaci�n
dt = 0.01; % per�odo de muestreo
% si se desea o no incluir gr�ficas del comportamiento del sistema
include_plot = true;
% si se desea o no incluir las trayectorias del control
plot_control = false; 
% si se desea o no incluir gr�ficas adicionales (diagrama de fase 2D o 3D,
% trayectorias 2D o 3D, funciones de Lyapunov en 2D)
include_extra_plots = false;
animate_simulation = true; % si se desea o no animar la simulaci�n

% No linealizado
% xss = [0; 0];
% uss = 0;
% [A,B,C,D] = linloc(@dynamics, @dynamics, xss, uss);
% [b,a] = ss2tf(A,B,C,D);
% SYS = tf(b,a);
% step(SYS);
% -------------------------------------------------------------------------
% CONDICIONES INICIALES
% -------------------------------------------------------------------------
x0 = [pi*9/10; 0];
%x0 = parameters.xe + 1*ones(2,1);
u0 = 0;


%% SOLUCI�N NUM�RICA DEL SISTEMA DIN�MICO
K = (tf - t0) / dt; % n�mero de iteraciones
x = x0; % vector de estado
u = u0; % vector de entradas
% array para almacenar las trayectorias de las variables de estado
% Inicialmente todo es cero exceptuando el primer elemento pues ser� igual
% a la condici�n inicial. Esto aplica para el vector de estados x y para
% las entradas u
X = zeros(numel(x0), K+1); X(:,1) = x;
% array para almacenar la evoluci�n de las entradas 
U = zeros(numel(u0), K+1); U(:,1) = u; 

% Soluci�n recursiva del sistema din�mico
for k = 1:K
    % Se�ales de entrada
    u = 0;%control(x, parameters);

    % Runge Kutta lo que hace es resolver ecuaciones diferenciales que
    % tienen la forma "dy/dx = f(x,y)"
    % M�todo RK4 para la aproximaci�n num�rica de la soluci�n
    % Se eval�a la funci�n f(x,y) en el x anterior y y anterior
    % en este caso ser�a en el x vector y u escalar anteriores
    % dynamics es la funci�n f(x,u)
    k1 = dynamics(x, u, parameters);
    k2 = dynamics(x+(dt/2)*k1, u, parameters);
    k3 = dynamics(x+(dt/2)*k2, u, parameters);
    k4 = dynamics(x+dt*k3, u, parameters);
    x = x + (dt/6)*(k1+2*k2+2*k3+k4);
    
    % Se guardan las trayectorias del estado y las entradas
    X(:, k+1) = x;
    U(:, k+1) = u;
end

%% GENERACI�N DE GR�FICAS
% -------------------------------------------------------------------------
% GR�FICAS POR DEFECTO DE LAS VARIABLES DE ESTADO Y LAS ENTRADAS
% -------------------------------------------------------------------------
if(include_plot)
    t = t0:dt:tf;
    figure;
    if(plot_control) 
        subplot(1,2,1);
    end
    
    plot(t, X', 'LineWidth', 1);
    xlabel('$t$', 'Interpreter', 'latex', 'Fontsize', 16);
    ylabel('$\mathbf{x}(t)$', 'Interpreter', 'latex', 'Fontsize', 16);
    axsl1 = legend(statevars, 'Location', 'best', 'Orientation', ...
        'vertical');
    set(axsl1, 'Interpreter', 'latex', 'FontSize', 12);
    grid minor;
    
    if(plot_control) 
        subplot(1,2,2);
        plot(t, U', 'LineWidth', 1);
        xlabel('$t$', 'Interpreter', 'latex', 'Fontsize', 16);
        ylabel('$\mathbf{u}(t)$', 'Interpreter', 'latex', 'Fontsize', 16);
        axsl2 = legend(inputvars, 'Location', 'best', 'Orientation', ...
        'vertical');
        set(axsl2, 'Interpreter', 'latex', 'FontSize', 12);
        grid minor;
    end
end

% -------------------------------------------------------------------------
% GR�FICAS ADICIONALES
% -------------------------------------------------------------------------
if(include_extra_plots)
    figure;
    extraplots(X, U, parameters, t0, tf, dt, statevars, inputvars);
end

% -------------------------------------------------------------------------
% ANIMACI�N (OPCIONAL)
% -------------------------------------------------------------------------
if(animate_simulation)
    figure;
    animation(X, U, parameters, t0, tf, dt, statevars, inputvars);
end