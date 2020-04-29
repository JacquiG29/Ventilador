function animation(X, U, parameters, t0, tf, dt, statevars, inputvars)
    m = parameters.m; 
    ell = parameters.ell; 
    b = parameters.b;
    g = parameters.g;
    xe = parameters.xe;
    ue = parameters.ue;
    disp_phasep = true;
    disp_control = false;
    disp_loclinpp = true;
    disp_locallin = true;
    disp_trajlin = true;
    
    K = (tf - t0) / dt; % número de iteraciones    
    t = t0:dt:tf;
    
    % Posición de la masa
    p1 = @(x) ell*[sin(x(1)); -cos(x(1))];
    
    if(disp_phasep)
        subplot(1,2,1);
        s = 6;
        [mX, mY] = meshgrid(xe(1)-s:0.4:xe(1)+s, xe(2)-s:0.4:xe(2)+s);
        [mDX, mDY] = phaseportrait2d(mX, mY, 1, false, parameters);

        phasep = quiver(mX, mY, mDX, mDY);
        phasep.Color = [0.5, 0.5, 0.5];
        hold on;
        
        if(disp_control)
           mUX = zeros(size(mX));
           mUY = -mY;%-mX-mY;
           controlp = quiver(mX, mY, mUX, mUY);
           controlp.Color = 'r';
        end
        
        if(disp_loclinpp)
            [I,J] = size(mX);
            lDX = zeros(I,J);
            lDY = lDX;
            for i = 1:I
                for j = 1:J
                    x = [mX(i,j); mY(i,j)];
                    linf = linearize_dynamics(x,0,xe,ue, parameters);
                    lDX(i,j) = linf(1); 
                    lDY(i,j) = linf(2);
                end
            end
            loclinpp =  quiver(mX, mY, lDX, lDY);
            loclinpp.Color = [51,153,255]*(1/255); 
        end
        
        flowp = plot(X(1,1),X(2,1), 'k', 'LineWidth', 1);
        xlim(xe(1)+[-s, s]);
        ylim(xe(2)+[-s, s]);
        xlabel(statevars(1), 'Interpreter', 'latex', 'Fontsize', 16);
        ylabel(statevars(2), 'Interpreter', 'latex', 'Fontsize', 16);
        %grid minor;
        
        if(disp_locallin)
            DX = zeros(size(X));
            %dx0 = [pi-100*0.01; 0]-xe;
            dx0 = [1;1];
            dx = dx0;
            DX(:,1) = dx;
            for k = 1:K
                % Señales de entrada
                du = control(dx, parameters);

                % Método RK4 para la aproximación numérica de la solución
                k1 = linearize_dynamics(dx, du, xe, ue, parameters);
                k2 = linearize_dynamics(dx+(dt/2)*k1, du, xe, ue, parameters);
                k3 = linearize_dynamics(dx+(dt/2)*k2, du, xe, ue, parameters);
                k4 = linearize_dynamics(dx+dt*k3, du, xe, ue, parameters);
                dx = dx + (dt/6)*(k1+2*k2+2*k3+k4);
    
                % Se guardan las trayectorias del estado
                DX(:, k+1) = dx;
            end
            
            LX = xe + DX(:,1);
            loclinflow = plot(LX(1),LX(2), 'Color', ...
                [51,153,255]*(0.5/255), 'LineWidth', 1);
        end
        
        if(disp_trajlin)
            
        end
        
        hold off;

        subplot(1,2,2);
    end
    
    s = 3;
    r = 0.1;
    xlim(s*[-1, 1]);
    ylim(s*[-1, 1]);
    grid off;
    hold on;
    
    q = X(1:2,1);
    r1 = p1(q); x1 = r1(1); y1 = r1(2);
    pendrod = plot([0,x1], [0,y1], 'Color', [0.5, 0.5, 0.5], 'LineWidth', 3);
    pendmass = circle(x1,y1,r);
    
    for k = 2:K+1
        q = X(1:2,k);
        r1 = p1(q); x1 = r1(1); y1 = r1(2);
        
        if(disp_phasep)
            flowp.XData = [flowp.XData, X(1,k)];
            flowp.YData = [flowp.YData, X(2,k)];
        end
        
        if(disp_locallin)
            LX = xe + DX(:,k);
            loclinflow.XData = [loclinflow.XData, LX(1)];
            loclinflow.YData = [loclinflow.YData, LX(2)];
        end
            
        pendrod.XData = [0,x1];
        pendrod.YData = [0,y1];
        pendmass.Position(1:2) = [x1-r, y1-r];
        drawnow limitrate
        pause(dt);
    end
end