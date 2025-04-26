close all; clc

% Last data
load('Lego_measurements_oppg_1j.mat') 
lys = Lys;  % Signal {u_k}
tid = Tid;  % Tid {t_k}

% Lag figur med 3x1 subplot
figure

% --- Plot rådata ---
subplot(3,1,1)
plot(tid, lys, 'b', 'LineWidth', 1.5)
grid on;
title('Målesignal \{u_k\} vs. tid t', 'FontSize', 12)
xlabel('Tid t [s]', 'FontSize', 10)
ylabel('Amplitude', 'FontSize', 10)
ylim([0 80])
xlim([0 30])

% --- Steg 1: Fjern konstant del i starten ---
tid_rang = 1:10;  % Indeks for konstant del
lys(tid_rang) = NaN;  % Sett konstant del til NaN

% --- Plot signal etter fjerning ---
subplot(3,1,2)
plot(tid, lys, 'b', 'LineWidth', 1.5)
grid on;
title('Fjernet konstant del og uteliggere', 'FontSize', 12)
xlabel('Tid t [s]', 'FontSize', 10)
ylabel('Amplitude', 'FontSize', 10)
ylim([0 80])
xlim([0 30])

% --- Interpoler NaN verdier ---
lys = fillmissing(lys, 'linear');  

% --- Juster tid slik at den starter fra t = 0 ---
tid = tid - tid(tid_rang(end) + 1);

% --- Steg 2: Erstatt uteliggere med NaN ---
mean_lys = mean(lys, 'omitnan');  
std_lys = std(lys, 'omitnan');  

out_thresh = 3 * std_lys;  
uteliggere = abs(lys - mean_lys) > out_thresh;  
lys(uteliggere) = NaN;  
lys = fillmissing(lys, 'linear');  

% --- Steg 3: Fjern likevektsverdi C ---
lys = lys - mean(lys, 'omitnan');  

% --- Ekstra: Reparer store dropp mellom 12-15s ---
% Oppdag dropp basert på differanse
diff_lys = diff(lys);
threshold_drop = -20;  % Terskel for plutselige dropp
drop_indices = find(diff_lys < threshold_drop);

% Utvid område rundt dropp
window_size = 20;
bad_idx = [];
for i = 1:length(drop_indices)
    bad_idx = [bad_idx, max(1, drop_indices(i)-window_size) : min(length(lys), drop_indices(i)+window_size)];
end
bad_idx = unique(bad_idx);

% Masker dropp med NaN og interpoler
lys(bad_idx) = NaN;
lys = fillmissing(lys, 'linear');

% --- Ekstra: Low-pass filter for glatting ---
Fs = 1 / mean(diff(tid));  
cutoff_hz = 2.0;  % Cutoff frekvens
Wn = cutoff_hz / (Fs/2);  

[b, a] = butter(4, Wn, 'low');  % 4. ordens Butterworth
%lys = filtfilt(b, a, lys);  % Filtrer signalet

% --- Estimer amplituden og frekvensen ---
N = length(lys);  
f = (0:N-1) * Fs / N;  

Y = fft(lys);
P2 = abs(Y / N);  
P1 = P2(1:N/2+1);  
f = f(1:N/2+1);  

[~, idx] = max(P1);  
omega = 2 * pi * f(idx);  
U = P1(idx);  

% Estimert signal u_est
u_est = U * cos(omega * tid);  

% --- Plot estimerte signal sammen med rengjort signal ---
subplot(3,1,3)
plot(tid, lys, 'b', 'LineWidth', 1.5)  
hold on;
plot(tid, u_est, 'r--', 'LineWidth', 1.5)  
grid on;
title('Rengjort signal og estimert u(t)', 'FontSize', 12)
xlabel('Tid t [s]', 'FontSize', 10)
ylabel('Amplitude', 'FontSize', 10)
ylim([-20 20])
xlim([0 30])
legend({'Rengjort signal', 'Estimert u(t)'}, 'Location', 'Best')

% --- Lagre rengjort signal ---
Lys = lys;  % nytt navn på signalet
Tid = tid;  % ny tid

save('P01_sinus_justert.mat', 'Lys', 'Tid')
