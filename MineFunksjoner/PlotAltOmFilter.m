function PlotAltOmFilter(u,y,t,B,A,fc,varargin)
%PlotAltOmFilter  Lager en figur med 6 delfigurer med
% signal- og filterinformasjon. Figuren er strukturet med:
% - to delfigurer til venstre som viser inngangssignalet u 
%   med tilhørede frekvensspekter 
% - to delfigurer i midten som viser sprangresponsen og 
%   amplitudeforsterkning til filteret 
% - to delfigurer til venstre som viser filtrert utgangssignal y
%   med tilhørende frekvensspekter
%
% Syntaks:
% PlotAltOmFilter(u,y,t,B,A,fc)
% PlotAltOmFilter(u,y,t,B,A,fc,'no hold')
%
% Innganger:
%     u - alle målingene
%     y - alle filtrerte målinger  
%     t - tidsvektoren
%     B - filterparametere, [b0 b1 b2 ..]
%     A - filterparametere, [ 1 a1 a2 ..]
%     fc - knekkfrekvensen som ble brukt i filtreringen av {u_k}
%     no hold - valgfritt argument slik at y-aksen til utsignal {y_k} 
%               ikke låses til å være lik y-aksen for {u_k}
%
%
% See also: FrekvensSpekterSignal, freqz, GenereltIIRFilter

% Author: Drengstig Tormod
% Work address: UiS
% Last revision: 20-Desember-2024


Ts = mean(diff(t));
fs = 1/Ts;
fN = fs/2;
orden = numel(A)-1;

figure
set(gcf,'position',[100 300 1100 650]);  

% ------------------------------------------------------
% subplot(2,3,1)
% ------------------------------------------------------
% inngangssignalet {u_k} plottet som funksjon av tid
subplot(2,3,1)
plot(t,u)
grid
title('Inngangssignal $\{u_k\}$','Interpreter','latex')
xlabel('tid [s]')
legend(['$\{T_s\}$=',num2str(Ts),' s'],'Interpreter','latex')
axis_u = axis;   
axis(axis_u); 
ylim_u = ylim;   % tar vare på y-aksegrenser til bruk for {y_k}

% ------------------------------------------------------
% subplot(2,3,2)
% ------------------------------------------------------
% sprangresponsen til filteret
subplot(2,3,2)
tau = 1/(2*pi*fc);     % 
if orden == 1
    t_step = 0:Ts:10*tau;
else
    t_step = 0:Ts:20*tau;
end    
u_step = ones(1,numel(t_step));
u_step(1) = 0;
y_step(1) = u_step(1);
for k = 2:length(t_step)
    y_step(k) = GenereltIIRFilter(u_step(1:k), y_step(1:k-1), B,A);
end
plot(t_step,y_step,'b')
hold on
plot(t_step,u_step,'r')
axis 'padded'
grid
title('Sprangrespons filter')
if orden == 1
    legend(['orden=',num2str(orden),...
    ', $\{tau\}$=',num2str(tau,3),' s'],...
                'location','best','Interpreter','latex')
else
    legend(['orden=',num2str(orden)],...
        'location','best','Interpreter','latex')
end
xlabel('tid [s]')


% ------------------------------------------------------
% subplot(2,3,3)
% ------------------------------------------------------
% det filtrert signalet {y_k} plottet som funksjon av tid
subplot(2,3,3)
plot(t,y)
grid
title('Utgangssignal $\{y_k\}$','Interpreter','latex')
xlabel('tid [s]')
if isempty(varargin)
    ylim(ylim_u)
end


% ------------------------------------------------------
% subplot(2,3,4)
% ------------------------------------------------------
% frekvensspekteret til inngangssignalet {u_k}
subplot(2,3,4)
[frekvenser_u, spekter_u] = FrekvensSpekterSignal(u,t);
plot(frekvenser_u, spekter_u,'linewidth',1.5) 
grid
title('Frekvensspekteret til $\{u_k\}$','Interpreter','latex')
xlabel('frekvenser [Hz] i inngangssignalet')
ylabel('amplitude')
ax_u_spekter = axis;
axis(ax_u_spekter)
hold on
plot([fc fc],ax_u_spekter(3:4),'r--')
legend(['$\{f_N\}$=',num2str(fs/2),' Hz'],...
    ['Spesifisert $\{f_c\}$=',num2str(fc),' Hz'],...
    'location','best','Interpreter','latex')



% ------------------------------------------------------
% subplot(2,3,5)
% ------------------------------------------------------
% Frekvensresponsen (amplitudeforsterkningen og faseforskyvning)
% til filteret med de valgte koeffisientene A og B
subplot(2,3,5)
numOfFreq = 1000;
[amplitude,frekvenser]=freqz(B,A,numOfFreq,fs);
plot(frekvenser,abs(amplitude))       % lineær x-akse
%semilogx(frekvenser,abs(amplitude))
grid
title('Amplitudeforsterkning filter')
ylabel('$\{A\}$','Interpreter','latex')
ax_freqz = axis;
axis(ax_freqz)
hold on
plot([fc fc],ax_freqz(3:4),'r--')
xlabel('frekvens [Hz]')


% ------------------------------------------------------
% subplot(2,3,6)
% ------------------------------------------------------
% frekvensspekteret til utgangssignal y
subplot(2,3,6)
[frekvenser_y, spekter_y] = FrekvensSpekterSignal(y,t);
plot(frekvenser_y,spekter_y) 
grid
title('Frekvensspekteret til $\{y_k\}$','Interpreter','latex')
xlabel('frekvenser [Hz] i utgangssignalet')
ylabel('amplitude')
axis(ax_u_spekter);
hold on
plot([fc fc],ax_u_spekter(3:4),'r--')

