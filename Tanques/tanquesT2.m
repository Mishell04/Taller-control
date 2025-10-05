clearvars; close all; clc;

%% --- Planta identificada (primer orden) ---
K = 0.9;         % ganancia estacionaria
T = 11.6;        % constante de tiempo
s = tf('s');
G = K/(T*s + 1); % planta aproximada

%% --- Diseño del controlador PI (método de Ragazzini) ---
tau_c = 0.5*T;   % tiempo de cierre deseado
Kp = T/(K*tau_c);
Ki = 1/tau_c;
Kd = 0;

C = Kp + Ki/s;   % Controlador PI (Kd=0)

%% --- Mostrar ganancias calculadas ---
fprintf('\n--- Valores del controlador PI ---\n');
fprintf('Kp = %.4f\n', Kp);
fprintf('Ki = %.4f\n', Ki);


%% --- Lazo cerrado y abierto ---
sys_ol = feedback(G,1);      % lazo cerrado sin control
sys_cl = feedback(C*G,1);    % lazo cerrado con control

%% --- Simulación de respuestas ---
t = 0:0.1:100;
[y_ol, t_ol] = step(sys_ol, t);
[y_cl, t_cl] = step(sys_cl, t);

info_ol = stepinfo(sys_ol);
info_cl = stepinfo(sys_cl);

% Error estacionario
ess_ol = 1 - dcgain(sys_ol);
ess_cl = 1 - dcgain(sys_cl);

%% --- Resultados en consola ---
disp('--- Planta sin control ---');
disp(info_ol);
fprintf('Error estacionario ess = %.4f\n\n', ess_ol);

disp('--- Planta con PI (Ragazzini) ---');
disp(info_cl);
fprintf('Error estacionario ess = %.4f\n\n', ess_cl);

%% --- Gráficas comparativas ---
figure;
plot(t_ol, y_ol,'r','LineWidth',1.5); hold on;
plot(t_cl, y_cl,'b','LineWidth',1.5);
yline(1,'--k','Referencia');
xlabel('Tiempo [s]');
ylabel('Salida');
legend('Planta sin control','Con PI','Referencia');
grid on;
title('Respuesta a paso de la planta de tanques con y sin control PI');

%% --- Tabla de métricas de desempeño ---
metrics = {
    'Tiempo de subida T_r [s]', info_ol.RiseTime, info_cl.RiseTime;
    'Tiempo de establecimiento T_s [s]', info_ol.SettlingTime, info_cl.SettlingTime;
    'Sobrepico M_p [%]', info_ol.Overshoot, info_cl.Overshoot;
    'Error estacionario e_{ss}', ess_ol, ess_cl
    };

fprintf('\n--- Tabla de métricas de desempeño ---\n');
fprintf('%35s  %12s  %12s\n', 'Métrica', 'Sin control', 'Con PI');
fprintf('%35s  %12s  %12s\n', repmat('_',1,35), repmat('_',1,12), repmat('_',1,12));

for i = 1:size(metrics,1)
    fprintf('%35s  %12.4f  %12.4f\n', metrics{i,1}, metrics{i,2}, metrics{i,3});
end
