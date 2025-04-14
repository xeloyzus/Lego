%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% P02_FiltreringTemperatur
%
% Hensikten med programmet er å lavpassfiltrere målesignalet u_k som
% representerer temperaturmåling [C].
% 
% Følgende sensorer brukes:
% - Lyssensor
%--------------------------------------------------------------------------

clear; close all;   % Rydd opp i workspace
online = false;     % Online mot EV3 eller lastede data?
plotting = false;    % Skal det plottes i sanntid?
filename = 'P02_LysTid_1.mat'; 

if online
    % LEGO EV3 og styrestikke
    mylego = legoev3('USB');
    joystick = vrjoystick(1);
    [JoyAxes,JoyButtons] = HentJoystickVerdier(joystick);

    % Sensorer
    myColorSensor = colorSensor(mylego);
    
else
    % Dersom online=false lastes datafil.
    load(filename);
end

fig1 = figure;
set(gcf, 'Position', [100, 100, 800, 600]); % Sett figurstørrelse
drawnow;

% Initialisering
JoyMainSwitch = 0;
k = 0;

% Sett lavpassfilterets tidskonstant (må justeres etter systemet ditt)
tau = 1.8;  % Juster for raskere/langsommere filtrering
y = 0;  % Startverdi for filtrert signal

%----------------------------------------------------------------------

while ~JoyMainSwitch

    %+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    % HENT TID OG MÅLINGER

    k = k + 1; % Øk tellevariabel

    if online
        if k == 1
            tic;
            Tid(1) = 0;
        else
            Tid(k) = toc;
        end

        % Hent lysintensitet fra sensor
        Lys(k) = double(readLightIntensity(myColorSensor, 'reflected'));

        % Data fra joystick
        [JoyAxes, JoyButtons] = HentJoystickVerdier(joystick);
        JoyMainSwitch = JoyButtons(1);
        
    else
        if k == length(Tid)
            JoyMainSwitch = 1;
        end

        if plotting
            pause(0.03); % Simuler kommunikasjonsforsinkelse
        end
    end

    %--------------------------------------------------------------

    %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    % BEREGNINGER OG FILTRERING

    u(k) = Lys(k); % Målte verdier

    if k == 1
        % Initialiser tidssteg
        T_s(1) = 0.05;  % Nominell verdi
        y(k) = u(k);    % Sett initialverdi for filter
    else
        % Beregn tidssteg
        T_s(k) = Tid(k) - Tid(k-1); 

        % Beregn filterparameter α basert på eksponentiell demping
        alpha = 1 - exp(-T_s(k) / tau);

        % Lavpassfiltrering 
        y(k) = (1 - alpha) * y(k-1) + alpha * u(k) ;
    end

    %--------------------------------------------------------------

    %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    % PLOTTING AV DATA

    if plotting || JoyMainSwitch
        plot(Tid(1:k), u(1:k), 'r-', 'LineWidth', 1.5); 
        hold on;
        plot(Tid(1:k), y(1:k), 'b-', 'LineWidth', 1.5);
        hold off;

        title('Temperatur Filtrering');
        xlabel('Tid [s]');
        ylabel('Temperatur [C]');
        legend({'$\{u_k\}$', '$\{y_k\}$'}, 'Location', 'best','Interpreter', 'latex');
        grid on;

        drawnow;
    end
    %--------------------------------------------------------------

end


%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% AVSLUTNING: STOPP MOTOR OG LAGRE DATA


% Stopp motor før avslutning

% AVSLUTNING: STOPP MOTOR 
if online
    stop(mymotor); % Stopp motor før avslutning

    % Lagre data
    %save('P01_chirp_juster.mat', 'Tid', 'Lys', 'JoyPot'); % RETTET: Lagre data
end
