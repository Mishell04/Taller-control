clear all; close all; clc;

%% --- Planta identificada (desde los datos reales) ---
K     = 0.1267;      % ganancia estacionaria
zeta  = 0.1855;      % amortiguamiento calculado de la señal real
Ts    = 13;          % tiempo de establecimiento experimental
wn    = pi/(zeta*Ts); % frecuencia natural calculada

% Coeficientes de la planta
B  = K*wn^2;
a1 = 2*zeta*wn;
a0 = wn^2;

% Función de transferencia de la planta
G  = tf([B],[1 a1 a0]);

%% --- Controlador PID diseñado (basado en planta real) ---
Kp = 83.5;
Ki = 69.4;
Kd = 59.5;

% Filtro derivativo opcional (para reducir ruido)
wn_d = 1.143;        % frecuencia natural deseada
tau_f = 0.1/wn_d;    % tiempo de filtrado pequeño
s = tf('s');
C = Kp + Ki/s + (Kd*s)/(tau_f*s + 1);

%% --- polinomio deseado ---
fprintf('\n--- CÁLCULO DEL POLINOMIO DESEADO ---\n');

% Especificaciones deseadas
zeta_d = 0.7;
t_s_des = 5;
wn_d = 4/(zeta_d * t_s_des);

% Polinomio de 2º orden deseado
poly2 = [1, 2*zeta_d*wn_d, wn_d^2];

% Tercer polo elegido
p3 = -10*wn_d;

% Polinomio cúbico deseado
syms s_sym;
poly_sym = expand((s_sym^2 + 2*zeta_d*wn_d*s_sym + wn_d^2)*(s_sym - p3));
coeffs_all = double(fliplr(coeffs(poly_sym, s_sym)));
A0 = coeffs_all(1);
A1 = coeffs_all(2);
A2 = coeffs_all(3);

fprintf('Polinomio deseado: s^3 + %.3f s^2 + %.3f s + %.3f\n', A2, A1, A0);

% Polinomio real con las ganancias
poly_real = [a1 + B*Kd, a0 + B*Kp, B*Ki];
fprintf('Polinomio real obtenido: s^3 + %.3f s^2 + %.3f s + %.3f\n', ...
    poly_real(1), poly_real(2), poly_real(3));

% Polinomio real normalizado
poly_real_norm = poly_real / poly_real(1);
fprintf('Polinomio real normalizado: s^3 + %.3f s^2 + %.3f s + %.3f\n', ...
    poly_real_norm(1), poly_real_norm(2), poly_real_norm(3));

%% --- Lazo cerrado ---
sys_cl = feedback(C*G,1);

%% --- Simulación de respuestas ---
tFinal = 30;
[y_nc, t_nc] = step(G, tFinal);        % Planta sin control
[y_cl, t_cl] = step(sys_cl, tFinal);   % Planta con PID

%% --- Métricas de desempeño ---
info_nc = stepinfo(G);      % tiempos y sobrepico sin control
info_cl = stepinfo(sys_cl); % tiempos y sobrepico con PID

% Error estacionario (para escalón unitario)
ess_nc = abs(1 - y_nc(end));
ess_cl = abs(1 - y_cl(end));

% Sobrepico porcentual
Mp_nc = info_nc.Overshoot;
Mp_cl = info_cl.Overshoot;

%% --- Mostrar resultados en consola ---
disp('--- Planta sin control ---');
disp(info_nc);
fprintf('Mp = %.2f %%  |  ess = %.4f\n\n', Mp_nc, ess_nc);

disp('--- Planta con PID ---');
disp(info_cl);
fprintf('Mp = %.2f %%  |  ess = %.4f\n\n', Mp_cl, ess_cl);

%% --- Gráfica comparativa ---
figure;
plot(t_nc, y_nc, 'b', 'LineWidth', 1.5); hold on;
plot(t_cl, y_cl, 'r', 'LineWidth', 1.5);
yline(1,'--k','Referencia');
xlabel('Tiempo [s]');
ylabel('Salida');
legend('Planta sin control','Planta con PID','Referencia');
grid on;
title('Respuesta de la planta identificada con y sin PID');

%% --- Tabla de métricas de desempeño ---
Metrics = table({'Tiempo de subida T_r [s]'; 'Tiempo de establecimiento T_s [s]'; ...
                 'Sobrepico M_p [%]'; 'Error estacionario e_{ss}'}, ...
                [info_nc.RiseTime; info_nc.SettlingTime; Mp_nc; ess_nc], ...
                [info_cl.RiseTime; info_cl.SettlingTime; Mp_cl; ess_cl], ...
                'VariableNames',{'Métrica','Sin_control','Con_PID'});

disp('--- Tabla de métricas de desempeño ---');
disp(Metrics);
