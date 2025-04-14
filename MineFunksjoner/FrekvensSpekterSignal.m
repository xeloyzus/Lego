function [varargout] = FrekvensSpekterSignalSignal(x,t,varargin)
%FrekvensSpekterSignal  Beregner frekvenspekteret til et signal basert
% på fft-funksjonen.
%
% Syntaks:
% FrekvensSpekterSignal(x,t) plotter signal og frekvensspekteret
%
% FrekvensSpekterSignal(x,t,S) plotter signal og frekvensspekteret 
% med strengen S i tittel-feltet.
%
% [f, s] = FrekvensSpekterSignal(x,t) plotter ikke, men 
% returner frekvensene f og spekteret s.
%
% Innganger:
%     x - signal som det beregnes frekvensspekter til
%     t - tidsvektoren til signalet
%     S - tekststreng med signalnavnet
%
% Utganger:
%     f - beregnet frekvenser
%     s - beregnet spekter
%
% Eksempel:
%     t = 0:0.01:10;
%     x = 0.8*cos(2*pi*3*t);
%     FrekvensSpekterSignal(x,t,'$\{x_k\}$')
% 
% See also: fft

% Author: Drengstig Tormod
% Work address: UiS
% Last revision: 12-May-2024


%------------- BEGIN CODE --------------
if isempty(varargin)
    SignalName = 'Signal';
else
    if isstr(varargin{1})
        SignalName = varargin{1};
    else
        error('Siste argument i kallet til funksjonen er ikke en streng')
        return
    end
end

Ts = mean(diff(t));
fs = 1/Ts;
fN = fs/2;     % Nyquistfrekvensen     
N = numel(x);  % Lengden av signalet
      
% Fouriertransformasjon
X = fft(x,N)/N; 

% Frekvenser langs x-aksen for plotting fra 0 til fN (Nyquistfrekvensen).
% Lengde på frekvensvektoren er halvparten av N siden
% vi skal bare plotte halvparten av frekvensspekteret.
frekvens = fN*linspace(0,1,round(N/2)); 

% Frekvenskomponentene i X er komplekse størrelser, og vi skal
% plotte kun lengden (amplituden) på disse. Bruker derfor abs().
spekter = abs(X(1:round(N/2)));

% Vi multipliser spekteret i de positive frekvensene med 2. 
% Vi trenger ikke multipliser amplitude(1) og amplitude(end) 
% med 2 fordi disse amplitudene tilsvarer henholdsvis 
% null- og Nyquistfrekvensene, og de har ikke kompleks
% konjugerte par i negative frekvenser.
spekter(2:end-1) = 2*spekter(2:end-1);

if nargout == 2
       varargout{1} = frekvens;
       varargout{2} = spekter;
end

if nargout == 0
    figure;
    subplot(2,1,1)
    plot(t,x)
    grid
    title(SignalName)
    xlabel('tid [s]')
    legend(['$T_s$=',num2str(Ts),' sekund'],'Interpreter','latex')

    % Plotter halvparten av fourierspekteret.
    subplot(2,1,2)
    plot(frekvens,spekter)
    grid
    title('Frekvensspekteret')
    legend(['$f_N$=',num2str(fN),' Hz'],'Interpreter','latex')
    xlabel('frekvenser [Hz] i signalet')
    ylabel('amplitude')
end

