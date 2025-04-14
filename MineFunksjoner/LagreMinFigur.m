function [] = LagreMinFigur(fig_handle,figurfilnavn)
%LagreMinFigur  Lagrer .fig og .pdf av gjeldende figur.
%
% Syntaks:  
% Klikk først i figurvinduet og kall funksjonen slik:
% LagreMinFigur(gcf,'figurfilnavn')
%
% Se også savefig, print

% Forfatter: Tormod Drengstig
% Siste revisjon: 6. november 2024


% Ved lagring til vektorgrafikk pdf
% må figurstørrelsen settes. 
fig_handle.PaperPositionMode = 'auto';
figure_pos = fig_handle.PaperPosition;
fig_handle.PaperSize = [figure_pos(3) figure_pos(4)];

% Lagrer 2 figurtyper, en *.fig slik at du kan 
% endre på figuren senere, og en *.pdf for rapport
figurename_1 = [figurfilnavn,'.fig'];
figurename_2 = [figurfilnavn,'.pdf'];

if ~exist(figurename_2)
    savefig(figurename_1)
    print('-dpdf','-painters','-bestfit',figurename_2)
else
    TekstStreng = ['Filen ''',figurename_2,...
        ''' finnes fra før. Overskrive?'];
    svar=questdlg(TekstStreng,'Advarsel','Ja','Nei','Nei');
    switch svar
        case 'Ja'
        savefig(figurename_1)
        print('-dpdf','-painters','-bestfit',figurename_2)
        case 'Nei'
            return
    end
end

end

