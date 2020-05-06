% parametros.m
% Proyecto del ventilador mecánico
% Jacqueline Guarcax y Gabriela Iriarte
% 6/05/2020
% Descripción: En este programa calculamos y guardamos los parámetros
% necesarios para la estimación del sistema del proyecto final de Sistemas
% de Control 2.
%% Parámetros del sistema
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

%% -------- AMBÚ -------
k = 1478; %3462.75; % cte. del resorte del Ambú (experimental)
%k=3000;
D2o =0.5; % Damping ambú FALTA ESTIMAR N-m-s/rad
K2 = k*N1N2^2; % K reflejada
D2 = D2o*N1N2^2; % Damping ambú reflejado

%% ------- BRAZO ------- 
Dp = 100;% fricción de engranes FALTA ESTIMAR 
Dp2 = Dp*N1N2^2; % fricción de engranes reflejado

%% J del péndulo. Cambiar este para hacerlo no lineal, etc.
modelo_pendulo = 1;
if(modelo_pendulo == 1)
    Jb = 0.00362641*sf; % kg/m2
elseif(modelo_pendulo == 2)
    %Jb = ; 
elseif(modelo_pendulo == 3)
    %Jb = ; 
end
Jb2 = (J2+Jb)*N1N2^2+J1; % J del péndulo reflejado

%% ------- MOTOR -------
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
Kf = 5;% K fumada FALTA ESTIMAR
m = 0.148; % kg
g = 9.81; % m/s2
ell = 0.29; % metros

%% Guardamos el workspace para importarlo en el ventilador.m (el main)
% Comentar o descomentar esta linea para exportar params.mat
save('params')


