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
filename = 'P03_minedata.mat';

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
tau = 0.02
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
            v(k) = (u(k) - u(k-1)) / T_s(k);
            v_f(k) = (u_f(k) - u_f(k-1)) / T_s(k);
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
            
            plot(Tid(1:k), u(1:k), 'b-'); hold on;
            plot(Tid(1:k), u_f(1:k), 'r-'); hold off;
            grid on;
            legend('$\{u_k\}$', '$\{u_f\}$', 'Interpreter', 'latex');
            title('Avstandsmålinger og IIR Filtrert avstandsmålinger', 'Interpreter', 'latex');
            xlim([x_min, x_max]);
            ylim([min([u(1:k), u_f(1:k)]) - 5, max([u(1:k), u_f(1:k)]) + 5]); 

            subplot(3,1,2)
            
            plot(Tid(1:k), Bryter(1:k), 'k'); hold off;
            grid on;
            title('Bryter p{\aa} Laserpistol', 'Interpreter', 'latex');
            xlim([x_min, x_max]);
            ylim([-0.5, 1.5]); % Binary signal (0 or 1)

            subplot(3,1,3)
            hold off
            plot(Tid(1:k), v(1:k), 'b-'); hold on;
            plot(Tid(1:k), v_f(1:k), 'r-'); hold off;
            grid on;
            legend('$\{v_k\}$', '$\{v_f\}$', 'Interpreter', 'latex');
            title('Hastighet beregnet fra avstand og IIR filtrert avstand', 'Interpreter', 'latex');
            xlabel('Tid [sek]');
            xlim([x_min, x_max]);
            ylim([min([v(1:k), v_f(1:k)]) - 5, max([v(1:k), v_f(1:k)]) + 5]); 
        end

        drawnow;
    end

end

% +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
%           STOP MOTORS

% AVSLUTNING: STOPP MOTOR
if online

    % Lagre data
    %save('P03_minedata.mat', 'Tid', 'Avstand','Bryter'); % RETTET: Lagre data
end
%------------------------------------------------------------------

