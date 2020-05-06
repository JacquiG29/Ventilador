% Perfiles_AmboVent.m
% Perfil de posición y velocidad del AmboVent
% Sistemas de Control 2 - Proyecto Final
% Luis Alberto Rivera
% Modificado por Jacqueline Guarcax y Gabriela Iriarte
% 6/05/2020
% En este programa guardamos los perfiles que debemos de seguir en el
% control del ventilador mecánico.
%% Valor de los perfiles de posición y velocidad del brazo

pos = [0,  0,  1,  2,  4,  6,  8, 10, 13, 15, 18, 21, 25, 28, 31, 35, 38, 42, 46, 50,...
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

%% Normalización de los datos
dt = 0.01; % 10ms período de muestreo usada por los amiguis de AmboVent
pos_norm = pos/max(pos); % Posición normalizada
aprox_der = diff(pos_norm)./dt; % Para que la velocidad sea exacta vamos a 
% sacar la derivada de la posición normalizada. El período de muestreo es
% el mismo que el que usan los de AmboVent.

%% Mapeo de la velocidad vel para que esté normalizada según la derivada
% que encontramos anteriormente
A = min(vel);
B = max(vel);
C = min(aprox_der);
D = max(aprox_der);

vel_norm = ((vel-A)./(B-A))*(D-C)+C;

%% Graficamos los perfiles si plotear es 1
plotear = 0;

if (plotear == 1)
    figure(1); clf;
    subplot(2,1,1);
    plot(pos_norm,'r');
    xlabel('Index');
    ylabel('Promiles of full range');
    title('Position');
    grid on;
    subplot(2,1,2);
    hold on
    plot(aprox_der,'g');
    plot(vel_norm,'b');
    xlabel('Index');
    ylabel('position change');
    title('Velocity');
    grid on;
end

%% Convertimos estos perfiles a las trayectorias que seguiran las variables de estado:
% Nuestras variables de estado son 
% x1 = theta motor 
% x2 = w motor
% x3 = theta brazo 
% x4 = w brazo

% Preguntar qué onda con el xss :(
xss = [zeros(size(pos_norm)); zeros(size(pos_norm)); pos_norm; vel_norm];

%% Encontramos uss ????? no c



%% Guardamos el workspace para importarlo en el ventilador.m (el main)
% Descomentar para guardar las variables para utilizarlas en el main.
% save('variables_linloc','xss','uss') 





