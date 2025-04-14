%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% P02_FiltreringTemperatur
%
% Hensikten med programmet er å filtrere temperaturmålesignalet u_k.
% Det benyttes lavpass- og høypassfiltre basert på Butterworth-filter.
% 
% Følgende sensorer brukes:
% - Lyssensor
%--------------------------------------------------------------------------

clear; close all;   % Rydd opp i workspace
online = false;     % Online mot EV3 eller lastede data?
plotting = false;    % Skal det plottes i sanntid?
filename = 'P01_stoy.mat'; % Fil med lagrede målinger

if online
    % LEGO EV3 og styrestikke
    mylego = legoev3('USB');
    joystick = vrjoystick(1);
    [JoyAxes, JoyButtons] = HentJoystickVerdier(joystick);

    % Sensorer
    myColorSensor = colorSensor(mylego);
else
    % Dersom online=false lastes datafil.
    load(filename);
end

% Opprett figur for plotting
fig1 = figure;
set(gcf, 'Position', [100, 100, 800, 600]); % Sett figurstørrelse
drawnow;

% Initialisering
JoyMainSwitch = 0;
k = 0;

% Definer filterparametere
Fs = 20;  % Sampling rate (juster etter behov)
Fc_lp = 1;  % Lavpass cutoff-frekvens (Hz)
Fc_hp = 1;  % Høypass cutoff-frekvens (Hz)
N = 2;  % Filterorden
T_s = Tid; % Initial verdi for tidssteg
% Spesifiser tau og beregn alfa
fc = 1.5; % Knekkfrekvens

tau = 0.05
alfa = 1-exp(-T_s/tau); % beregning av tilhørende tidskonstant \tau
% beregning av \alpha

B = [alfa];
A = [1 -(1-alfa)]; % parametervektor B
% parametervektor A
% Beregn normaliserte frekvenser for Butterworth-filteret
Wn_lp = Fc_lp / (Fs / 2);
Wn_hp = Fc_hp / (Fs / 2);

% Design lavpass- og høypassfilter (Butterworth)
[bl, al] = butter(N, Wn_lp, 'low');  % Lavpass
[bh, ah] = butter(N, Wn_hp, 'high'); % Høypass

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
        if length(JoyButtons) >= 1
            JoyMainSwitch = JoyButtons(1);
        end
    else
        if k >= length(Tid)
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
        T_s(1) = 0.01;  % nominell verdi
        y_lp(k) = u(k); % Sett initialverdi for lavpass
        y_hp(k) = 0;    % Høypass startverdi
    else
        % Beregn tidssteg
        T_s(k) = Tid(k) - Tid(k-1);

        % Bruk Butterworth-filter (filtfilt for null faseforskyvning)
        y_lp = filter(bl, al, u); % Lavpassfiltering
        y_hp = filter(bh, ah, u); % Høypassfiltering
       
%GenereltIIRFilter(u,y_hp,bh,ah)
        
    end

    %--------------------------------------------------------------

    %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    % PLOTTING AV DATA

 if plotting || JoyMainSwitch
    
    clf; % Fjern forrige plot
    hold on;
    
    % Plot all signals
    plot(Tid(1:k), u(1:k), 'r-', 'LineWidth', 1.5); % Rådata
    plot(Tid(1:k), y_lp(1:k), 'g-', 'LineWidth', 1.5); % Lavpass
    plot(Tid(1:k), y_hp(1:k), 'b-', 'LineWidth', 1.5); % Høypass
    
    title('Temperatur Filtrering');
    xlabel('Tid [s]');
    ylabel('Temperatur [C]');
    legend({'R{\aa}data $\{u_k\}$', 'Lavpass $\{y_{lp}\}$', 'H{\r a}ypass $\{y_{hp}\}$'}, ...
        'Location', 'best', 'Interpreter', 'latex');
    grid on;

    x_min = min(Tid(1:k)); 
    x_max = Tid(k);  
    xlim([x_min, x_max + 1]); 

    min_val = min([u(1:k), y_lp(1:k), y_hp(1:k)]);
    max_val = max([u(1:k), y_lp(1:k), y_hp(1:k)]);
    
    y_range = max_val - min_val;
    y_margin = max(y_range * 0.1, 1);
    
    ylim([min_val - y_margin, max_val + y_margin]);  

    drawnow;
    hold off;
   
    

end
    %--------------------------------------------------------------

end

%PlotAltOmFilter(u(1:k),y_lp(1:k-1),Tid(k),B,A,fc)
GenereltIIRFilter(u(1:k),y_lp(1:k-1),B,A)
%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

% AVSLUTNING: 
if online
    % Lagre data
    save('P01_stoy.mat', 'Tid', 'Lys'); % RETTET: Lagre data
end