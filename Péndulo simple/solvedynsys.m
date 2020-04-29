function [X,U] = solvedynsys(x0,u0,t0,tf,dt,parameters)
    K = (tf - t0) / dt; % número de iteraciones
    x = x0; % vector de estado
    u = u0; % vector de entradas
    % array para almacenar las trayectorias de las variables de estado
    X = zeros(numel(x0), K+1); X(:,1) = x;
    % array para almacenar la evolución de las entradas 
    U = zeros(numel(u0), K+1); U(:,1) = u; 
    
    % Solución recursiva del sistema dinámico
    for k = 1:K
        % Señales de entrada
        u = control(x, parameters);

        % Método RK4 para la aproximación numérica de la solución
        k1 = dynamics(x, u, parameters);
        k2 = dynamics(x+(dt/2)*k1, u, parameters);
        k3 = dynamics(x+(dt/2)*k2, u, parameters);
        k4 = dynamics(x+dt*k3, u, parameters);
        x = x + (dt/6)*(k1+2*k2+2*k3+k4);
    
        % Se guardan las trayectorias del estado y las entradas
        X(:, k+1) = x;
        U(:, k+1) = u;
    end
end