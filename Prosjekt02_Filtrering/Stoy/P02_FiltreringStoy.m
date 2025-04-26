%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% P02_FiltreringTemperatur
%
% Hensikten med programmet er å filtrere temperaturmålesignalet u_k.
% Det benyttes lavpass- og høypassfiltre basert på Butterworth-filter.
% 
% Følgende sensorer brukes:
% - Lyssensor
%--------------------------------------------------------------------------

clear; close all;
online = false;
plotting = true;
filename = 'P01_stoy.mat';

if online
    mylego = legoev3('USB');
    joystick = vrjoystick(1);
    [JoyAxes, JoyButtons] = HentJoystickVerdier(joystick);
    myColorSensor = colorSensor(mylego);
else
    load(filename);
end

JoyMainSwitch = 0;
k = 0;

% Filterparametere
Fs = 20;
Fc_lp = 1;
Fc_hp = 1;
N = 2;
T_s = 1/Fs;  % Nominell tidssteg

fc = 0.5;
tau = 1/2*pi*fc;
alfa = 1 - exp(-T_s / tau);

B = [alfa];
A = [1 -(1-alfa)];

Wn_lp = Fc_lp / (Fs / 2);
Wn_hp = Fc_hp / (Fs / 2);

[bl, al] = butter(N, Wn_lp, 'low');
[bh, ah] = butter(N, Wn_hp, 'high');

while ~JoyMainSwitch
    k = k + 1;

    if online
        if k == 1
            tic;
            Tid(1) = 0;
        else
            Tid(k) = toc;
        end
        Lys(k) = double(readLightIntensity(myColorSensor, 'reflected'));
        [JoyAxes, JoyButtons] = HentJoystickVerdier(joystick);
        if length(JoyButtons) >= 1
            JoyMainSwitch = JoyButtons(1);
        end
    else
        if k >= length(Tid)
            JoyMainSwitch = 1;
        end
        if plotting
            pause(0.03);
        end
    end

    u(k) = Lys(k);

    if k == 1
        T_s(k) = T_s;
        y_lp(k) = u(k);
        y_hp(k) = 0;
    else
        T_s(k) = Tid(k) - Tid(k-1);

        % Egenimplementert lavpass og høypass
        y_lp(k) = GenereltIIRFilter(u(1:k), y_lp(1:k-1), bl, al);  % Butterworth low-pass
        y_hp(k) = GenereltIIRFilter(u(1:k), y_hp(1:k-1), bh, ah);  % Butterworth high-pass
                %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    end

    %--------------------------------------------------------------

    if plotting || JoyMainSwitch
        % Bruk det generelle IIR-filteret til slutt
       

    end
end
%PlotAltOmFilter(u(1:k),y_lp(1:k),Tid(1:k),bl,al,fc)

PlotAltOmFilter(u(1:k),y_hp(1:k),Tid(1:k),bh,ah,fc)

%FrekvensSpekterSignal(u(1:k), Tid(1:k))
% Lagre hvis online
if online
    % save('P01_stoy.mat', 'Tid', 'Lys');
end
