%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% P03_NumeriskDerivasjonStigningstall
%
% Hensikten med programmet er å numerisk derivere filtrert
% målesignal u_{f,k} som representere avstandsmåling [m]
% til å beregne v_{f,k} som fart [m/s]
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
filename = 'Prosjekt03_NumeriskDerivasjon/Stigningstall/P03_minedata_sinus.mat';

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
Ts_nom = 0.01; % Nominell verdi for tidssteg [s]
% Spesifiser tau og beregn alfa
fc = 1.5; % Knekkfrekvens [Hz]
tau = 1/(2*pi*fc); % Tidskonstant [s]

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

        % Data fra styrestikke
        [JoyAxes,JoyButtons] = HentJoystickVerdier(joystick);
        JoyMainSwitch = JoyButtons(1);
    else
        % online=false
        % Når k er like stor som antall elementer i datavektoren Tid,
        % simuleres det at bryter på styrestikke trykkes inn.
        if k==numel(Tid)
            JoyMainSwitch=1;
        end

        if plotting
            % Simulerer tiden som EV3-Matlab bruker på kommunikasjon
            pause(0.03)
        end
    end


    % +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    %             CONDITIONS, CALCULATIONS AND SET MOTOR POWER
    % Gjør matematiske beregninger og motorkraftberegninger.

    % Tilordne målinger til variabler (behold avstand i meter)
    u(k) = Avstand(k); % Avstand i meter
    
    if k==1
        % Initialverdier
        T_s(k) = Ts_nom;  % Initialt tidssteg
        u_f(k) = u(k);    % Initialverdi for filter
        v(k) = 0;        % Initial hastighet
        v_f(k) = 0;       % Initial filtrert hastighet
    else
        % Beregn tidssteg
        T_s(k) = Tid(k) - Tid(k-1);
        
        % Beregn filtrert avstand (IIR-filter)
        alfa = 1-exp(-T_s(k)/tau);
        u_f(k) = (1 - alfa) * u_f(k-1) + alfa * u(k);
        
        % Beregn hastighet (numerisk derivasjon) i m/s
        v(k) = (u(k) - u(k-1)) / T_s(k);
        v_f(k) = (u_f(k) - u_f(k-1)) / T_s(k);
    end

    %--------------------------------------------------------------


    %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    %                  PLOT DATA
    %
    % Plotter enten i sann tid eller når forsøk avsluttes

    if plotting || JoyMainSwitch
        if k > 1
            x_min = min(Tid(1:k));
            x_max = max(Tid(1:k));

            subplot(3,1,1)
            plot(Tid(1:k), u(1:k), 'b-'); hold on;
            plot(Tid(1:k), u_f(1:k), 'r-'); hold off;
            grid on;
            legend('$\{u_k\}$', '$\{u_f\}$', 'Interpreter', 'latex');
            title('Avstandsmålinger og IIR Filtrert avstandsmålinger', 'Interpreter', 'latex');
            xlabel('Tid [s]');
            ylabel('Avstand [m]');
            xlim([x_min, x_max]);
            ylim([min([u(1:k), u_f(1:k)])-0.1, max([u(1:k), u_f(1:k)])+0.1]);

            subplot(3,1,2)
            plot(Tid(1:k), Bryter(1:k), 'k');
            grid on;
            title('Bryter på Laserpistol', 'Interpreter', 'latex');
            xlabel('Tid [s]');
            ylim([-0.5, 1.5]);
            xlim([x_min, x_max]);

            subplot(3,1,3)
            plot(Tid(1:k), v(1:k), 'b-'); hold on;
            plot(Tid(1:k), v_f(1:k), 'r-'); hold off;
            grid on;
            legend('$\{v_k\}$', '$\{v_f\}$', 'Interpreter', 'latex');
            title('Hastighet beregnet fra avstand og filtrert avstand', 'Interpreter', 'latex');
            xlabel('Tid [s]');
            ylabel('Hastighet [m/s]');
            xlim([x_min, x_max]);
            ylim([min([v(1:k), v_f(1:k)])-0.5, max([v(1:k), v_f(1:k)])+0.5]);
        end
        drawnow;
    end
end

% +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
%           STOP MOTORS

% AVSLUTNING: STOPP MOTOR OG LAGRE DATA
if online
    save('P03_stigningstall.mat', 'Tid', 'Avstand', 'Bryter');
end
%------------------------------------------------------------------