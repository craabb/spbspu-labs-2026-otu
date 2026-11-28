clear; 
clc; 
close all;

Go = tf(1, [1 2 1]);
sigma_req = 5;
t_p_req = 0.5;

%% 1. Оценка исходной ЗАМКНУТОЙ системы
G_closed = feedback(Go,1);
S = stepinfo(G_closed);
fprintf('1. Исходная система\n');
fprintf('Полюса: '); fprintf('%.3f  ', pole(G_closed)); fprintf('\n');
fprintf('Устойчивость: %d\n', isstable(G_closed));
fprintf('t_п = %.4f c, sigma = %.2f %%, ошибка = %.4f\n\n', ...
    S.SettlingTime, S.Overshoot, dcgain(feedback(1,Go)));

%% 2. Настройка ПИД методом Зиглера-Николса
L = 3 - exp(1);
T = exp(1);

Kp_first_method = 1.2*T/L;   
Ti_first_method = 2*L;   
Td_first_method = 0.5*L;
fprintf('2. Метод Зиглера-Николса\n');
fprintf('Метод 1: L = %.4f c, T = %.4f c\n', L, T);
fprintf('         Kp = %.4f, Ti = %.4f c, Td = %.4f c\n\n', ...
    Kp_first_method, Ti_first_method, Td_first_method);

%% 3. Оценка системы с ПИД регулятором
Kp = Kp_first_method;
Ti = Ti_first_method;
Td = Td_first_method;

C = pid(Kp, Kp/Ti, Kp*Td);
G_closed_zn = feedback(C*Go,1);
Sp = stepinfo(G_closed_zn);
err_zn = dcgain(feedback(1,C*Go));

fprintf('3. Система с ПИД (Зиглера-Николса)\n');
fprintf('C(s) = %.4f + %.4f/s + %.4f*s\n', Kp, Kp/Ti, Kp*Td);
fprintf('t_п = %.4f c (требование <= %.1f: %d), sigma = %.2f %% (требование <= %.1f: %d), ошибка = %.4f\n\n', ...
    Sp.SettlingTime, t_p_req, Sp.SettlingTime <= t_p_req, ...
    Sp.Overshoot, sigma_req, Sp.Overshoot <= sigma_req, ...
    err_zn);

%% 4. Автоматическая настройка с помощью pidtune
Ca = pidtune(Go, 'pid');
Sa = stepinfo(feedback(Ca*Go,1));
fprintf('4. Автонастройка (pidtune) ===\n');
fprintf('Автоматическая настройка: Kp=%.4f Ki=%.4f Kd=%.4f | t_п=%.4f c, sigma=%.2f %%\n\n', ...
    Ca.Kp, Ca.Ki, Ca.Kd, Sa.SettlingTime, Sa.Overshoot);

wc_target = 10;
Ca_fast = pidtune(Go, 'pid', wc_target);
Sa_fast = stepinfo(feedback(Ca_fast*Go,1));
fprintf('Улучшенная автонастройка (wc=%.1f)\n', wc_target);
fprintf('  Kp=%.4f, Ki=%.4f, Kd=%.4f\n', Ca_fast.Kp, Ca_fast.Ki, Ca_fast.Kd);
fprintf('  t_п=%.4f c, sigma=%.2f %%\n\n', Sa_fast.SettlingTime, Sa_fast.Overshoot);

[GM_zn, PM_zn, ~, ~] = margin(C*Go);
[GM_auto, PM_auto, ~, ~] = margin(Ca_fast*Go);

fprintf('5. Запасы устойчивости \n');
fprintf('Зиглера-Николса:      Запас по амплитуде = %.2f дБ, Запас по фазе = %.2f град\n', 20*log10(GM_zn), PM_zn);
fprintf('Автонастройка (wc=10): Запас по амплитуде = %.2f дБ, Запас по фазе = %.2f град\n\n', 20*log10(GM_auto), PM_auto);

figure;
step(G_closed, 'b', G_closed_zn, 'r--', feedback(Ca_fast*Go,1), 'g-.');
legend('Без регулятора', 'Зиглера-Николса', ['Автонастройка (wc=' num2str(wc_target) ')']);
title('Переходные процессы');
ylabel('Amplitude');
grid on;
