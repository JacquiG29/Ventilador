load('variables_linloc')

% diametro_ambu = 0.1;
% max_angle = pi/3;
% min_angle = pi/4;
% ell = 0.29;
% pendrod = plot([0,0], [0,0], 'Color', [0.12, 0.12, 0.12], 'LineWidth', 5);
% xlim([0 0.3]);
% ylim([0 0.3]);
% pos = [0.1 0 0.1 0.1]; 
% rectangle('Position',pos,'Curvature',[1 1],'EdgeColor',[0 .5 .5],'FaceColor',[0 .5 .5])
% 
% for y = pos_rep'
%     x = map(0,1,min_angle,max_angle,y);
%     fprintf("sin=%f, cos=%f \n", sin(x), cos(x));
%     pendrod.XData = [ell*sin(x) 0];%[0,ell*sin(x)];
%     pendrod.YData = [ell*cos(x) 0];%[0,ell*cos(x)];
%     drawnow %limitrate
% end

t = t0:0.0001:tf1; %size(Ref',1)
waka = t0:0.01:tf1;
prueba2 = zeros(50002, 2);
valor = 1;
paquita = 1;
for waka = t0:0.01:tf1
    for ta = 1:100
        prueba2(paquita,:)=[t(paquita) ref_pos(valor,2)];
        paquita = paquita + 1;
    end
    valor = valor + 1;
end
prueba2 = prueba2(1:50001,:);
