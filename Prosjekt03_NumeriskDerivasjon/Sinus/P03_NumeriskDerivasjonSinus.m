%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% P03_NumeriskDerivasjonStigningstall
%
% Hensikten med programmet er å numerisk derivere filtrert
% målesignal u_{f,k} som representere avstandsmåling [m]
% til å beregne v_{f,k} som fart [km/t]
%
% Følgende sensorer brukes:
% - Ultralydsensor
% - Bryter
%--------------------------------------------------------------------------


%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
%         EXPERIMENT SETUP, FILENAME AND FIGURE

clear; close all   % Alltid lurt å rydde workspace opp først
online = false;    % Online mot EV3 eller mot lagrede data?
plotting = true;   % Skal det plottes mens forsøket kjøres
filename = 'P03_minedata_sinus.mat';

if online

    % LEGO EV3 og styrestikke
    mylego = legoev3('USB');
    joystick = vrjoystick(1);
    [JoyAxes,JoyButtons] = HentJoystickVerdier(joystick);

    % sensorer
    mySonicSensor = sonicSensor(mylego);
    myTouchSensor = touchSensor(mylego);

else
    % Dersom online=false lastes datafil.
    load(filename)
end

fig1 = figure;
set(gcf, 'Position', [100, 100, 800, 600]); % Sett figurstørrelse
drawnow;

% setter skyteknapp til 0, og initialiserer tellevariabel k
JoyMainSwitch=0;
k=0;

% Definer initialverder
% Velg måling Tid, Avstand og Bryter
T_s = 0.01; % Initial verdi for tidssteg
% Spesifiser tau og beregn alfa
fc = 1.5; % Knekkfrekvens
Ts = T_s; % Samplingsperiode
tau = 0.05
alfa = 1-exp(-T_s/tau);

%----------------------------------------------------------------------


while ~JoyMainSwitch

    %+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    %                       GET TIME AND MEASUREMENT
    % Få tid og målinger fra sensorer, motorer og joystick

    % oppdater tellevariabel
    k=k+1;

    if online
        if k==1
            tic
            Tid(1) = 0;
        else
            Tid(k) = toc;
        end

        % sensorer
        Avstand(k) = double(readDistance(mySonicSensor));
        Bryter(k)  = double(readTouch(myTouchSensor));

        % Data fra styrestikke. Utvid selv med andre knapper og akser
        [JoyAxes,JoyButtons] = HentJoystickVerdier(joystick);
        JoyMainSwitch = JoyButtons(1);
    else
        % online=false
        % Naar k er like stor som antall elementer i datavektpren Tid,
        % simuleres det at bryter paa styrestikke trykkes inn.
        if k==numel(Tid)
            JoyMainSwitch=1;
        end

        if plotting
            % Simulerer tiden som EV3-Matlab bruker på kommunikasjon
            % når du har valgt "plotting=true" i offline
            pause(0.03)
        end

    end


    % +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    %             CONDITIONS, CALCULATIONS AND SET MOTOR POWER
    % Gjør matematiske beregninger og motorkraftberegninger.

    % Tilordne målinger til variabler
    u(k) = 100*Avstand(k);
    if k==1
        

        % Spesifisering av initialverdier og parametere
        T_s(1) = 0.01;  % nominell verdi
        u_f(k) = u(k);  % Sett initialverdi for filter
    else
        % Beregn filtrert avstand
        u_f(k) = (1 - alfa) * u_f(k-1) + alfa * u(k);

        % Beregn tidsskritt
        T_s(k) = Tid(k) - Tid(k-1);

        if Bryter(k) == 1
            % Beregn hastighet
            v(k) = ((u(k) - u(k-1)) / T_s(k));
            v_f(k) = ((u_f(k) - u_f(k-1)) / T_s(k));
        else
            v(k) = 0;
            v_f(k) = 0;
        end
    end

    %--------------------------------------------------------------


    %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    %                  PLOT DATA
    %
    % Husk at syntaksen plot(Tid(1:k),data(1:k))
    % for gir samme opplevelse i online=true og online=false siden
    % hele datasettet (1:end) eksisterer i den lagrede .mat fila

    % Plotter enten i sann tid eller når forsøk avsluttes
    if plotting || JoyMainSwitch
        if k > 1
          
            x_min = min(Tid(1:k));
            x_max = max(Tid(1:k));

            subplot(3,1,1)
            
            %plot(Tid(1:k), u(1:k), 'b-'); hold on;
            plot(Tid(1:k), u_f(1:k), 'r-'); hold off;
            grid on;
            %legend('$\{u_k\}$', '$\{u_f\}$', 'Interpreter', 'latex');
            title('Avstandsmålinger og IIR Filtrert avstandsmålinger', 'Interpreter', 'latex');
             ylabel('[ m ]');
            xlim([x_min, x_max]);
            ylim([min([u(1:k), u_f(1:k)]) - 5, max([u(1:k), u_f(1:k)]) + 5]); 

            subplot(3,1,2)
            hold off
            plot(Tid(1:k), v(1:k), 'b-'); hold on;
            plot(Tid(1:k), v_f(1:k), 'r-'); hold off;
            grid on;
            legend('$\{v_k\}$', '$\{v_f\}$', 'Interpreter', 'latex');
            title('Hastighet beregnet fra avstand og IIR filtrert avstand', 'Interpreter', 'latex');
            xlabel('Tid [sek]');
            ylabel('[ Km/t ]');
            xlim([x_min, x_max]);
            ylim([min([v(1:k), v_f(1:k)]) - 5, max([v(1:k), v_f(1:k)]) + 5]); 
        end

        drawnow;
    end

end


% +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
%           STOP MOTORS

if online
    stop(motorA);
    % Lagre data
    % save('P03_minedata_sinus.mat', 'Tid', 'Avstand','Bryter');
end


%------------------------------------------------------------------

% ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% VERIFY NUMERICAL DIFFERENTIATION USING PREDEFINED PARAMETERS

% Use your predefined values for amplitude and frequency
U = 3.4; % amplitude
w = 2*pi*(1/2.65); % angular frequency [rad/s] 

% Calculate offset (C) as mean of filtered signal
C = mean(u_f);

% Create fitted sine wave for distance (Equation 8.8)
u_f_est = U*sin(w*Tid)+(U/w + u_f(1))

% Theoretical derivative (velocity) (Equation 8.9)
V = U*w; % amplitude [cm/s]
phi = pi/2; % phase shift
v_f_est = V * sin(w*Tid+phi);

% ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% PLOT THE RESULTS FOR VERIFICATION

figure(fig1)
if k > 1
    x_min = min(Tid(1:k));
    x_max = max(Tid(1:k));
    
    subplot(3,1,1)
    %plot(Tid(1:k), u(1:k), 'b-'); 
    plot(Tid(1:k), u_f(1:k), 'b-');hold on;
    plot(Tid(1:k), u_f_est(1:k), 'r-', 'LineWidth', 1.5); hold off;
    grid on;
    legend('$\{u_k\}$', '$\{u_{f,k}\}$', '$u_f(t) = U\sin(\omega t) + C$', ...
           'Interpreter', 'latex', 'Location', 'best');
    title('Avstandsmålinger og tilpasset sinusfunksjon', 'Interpreter', 'latex');
    ylabel('[m]');
    xlim([x_min, x_max]);
    
    subplot(3,1,2)
    plot(Tid(1:k), v_f(1:k), 'b-'); hold on;
    plot(Tid(1:k), v_f_est(1:k), 'r-', 'LineWidth', 1.5); hold off;
    grid on;
    legend('$\{v_{f,k}\}$', '$v_f(t) = U\omega\sin(\omega t)$', ...
           'Interpreter', 'latex', 'Location', 'best');
    title('Verifisering av numerisk derivasjon', 'Interpreter', 'latex');
    ylabel('[m/s]');
    xlim([x_min, x_max]);
    
    % Add parameter information
    subplot(3,1,3)
    text(0.1, 0.8, sprintf('Brukte parametere:'), 'FontSize', 12, 'FontWeight', 'bold');
    text(0.1, 0.65, sprintf('Amplitude: U = %.2f cm', U), 'FontSize', 12);
    text(0.1, 0.5, sprintf('Vinkelfrekvens: ω = %.2f rad/s', w), 'FontSize', 12);
    text(0.1, 0.35, sprintf('Fart amplitude: V = Uω = %.2f cm/s', V), 'FontSize', 12);
    text(0.1, 0.2, sprintf('Konstantledd: C = %.2f cm', C), 'FontSize', 12);
    axis off;
    
    drawnow;
end