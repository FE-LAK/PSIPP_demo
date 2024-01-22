clear all; close all; %format

mapChoice=3;   %map choices: 1 = Mwrh1, 2=Mwrh2, 3= Mgame


M={};

switch mapChoice
    case 1  % simple do 10agv 33 ponovitev v vec lokacij v grafu (kot case 2, le da je jsno mapa z rugimi oznakami cest)
        Mfin=5; % stevilo metod
        M{1}=load('Results/Single/Result_Mwrh1_M0.txt','-ascii'); % Astar  
        M{2}=load('Results/Single/Result_Mwrh1_M1.txt','-ascii');  % pSIPP
        M{3}=load('Results/Single/Result_Mwrh1_M2.txt','-ascii');  % pSIPPc
        M{4}=load('Results/Single/Result_Mwrh1_M4.txt','-ascii');  % pSIPPl
        M{5}=load('Results/Single/Result_Mwrh1_M5.txt','-ascii');  % CCBS
        
        Mlabel={'A*','PSIPP','PSIPPc','PSIPPl','CCBS'};
    
   %================================================    
    case 2  % Digani do 40agv 33 ponovitev iz slepih ulic
        Mfin=5; % stevilo metod
        M{1}=load('Results/Single/Result_Mwrh2_M0.txt','-ascii'); % Astar 
        M{2}=load('Results/Single/Result_Mwrh2_M1.txt','-ascii');  % pSIPP
        M{3}=load('Results/Single/Result_Mwrh2_M2.txt','-ascii');  % pSIPPc
        M{4}=load('Results/Single/Result_Mwrh2_M4.txt','-ascii');  % pSIPPl
        M{5}=load('Results/Single/Result_Mwrh2_M5.txt','-ascii');  % CCBS
        
        Mlabel={'A*','PSIPP','PSIPPc','PSIPPl','CCBS'};
        
   %================================================
    case 3  % den520A do 35agv do 43 ponovitev iz slepih ulic
        Mfin=5; % stevilo metod
        M{1}=load('Results/Single/Result_Mgame_M0.txt','-ascii'); % Astar 
        M{2}=load('Results/Single/Result_Mgame_M1.txt','-ascii');  % pSIPP
        M{3}=load('Results/Single/Result_Mgame_M2.txt','-ascii');  % pSIPPc
        M{4}=load('Results/Single/Result_Mgame_M4.txt','-ascii');  % pSIPPl
        M{5}=load('Results/Single/Result_Mgame_M5.txt','-ascii');  % CCBS
        
        Mlabel={'A*','PSIPP','PSIPPc','PSIPPl','CCBS'};
   %================================================
        
end

%Mcolor={'b-.','r-.','k:','g.-','m-'};
Mcolor={'b-.','b-.','k:','g.-','r-'};

NagvMax=M{1}(end,2);
T=[];
Eta={};
ZetaTSoc={};
ZetaTMks={};
ZetaNit={};
AvTSoc={};
AvZetaTMks={};
AvZetaNit={};


Thr={};

Nagv=(1:NagvMax)';

for i=1:NagvMax
    
    for m=1:Mfin
        filt1= M{m}(:,2)==i;        % filter isto st. AGV
        filt2=~isnan(M{m}(:,3));    % filter veljavna resitev
        filt= filt1  &  filt2;
        
        % da delam povprecje istih
        ff= (M{5}(:,2)==i) & ~isnan(M{5}(:,3)) & (M{m}(:,2)==i) & ~isnan(M{m}(:,3)) ;
        
        Eta{m}(i)=sum(filt)/sum(filt1);
        
        
        Thr{m}(i)= i*sum(filt)/sum(M{m}(filt,4))*3600;    % ? to je le za uspesne primere zato pri CCBS vedno narasca
        %  Thr{m}(i)= mean(i*(1)./(M{m}(filt,4))*3600);
        
        AvTSoc{m}(i)=mean(M{m}(filt,3));   % povprecnih casov tSoc
        AvTMks{m}(i)=mean(M{m}(filt,4));   %  povprecnih casov tMks
        AvNit{m}(i) =mean(M{m}(filt,5));
        
        % delam povprecje le tistih,ki imajo oboji resitev
        ZetaTSoc{m}(i)=mean(M{m}(ff,3))/mean(M{5}(ff,3));   % razmerje povprecnih casov tSoc
        ZetaTMks{m}(i)=mean(M{m}(ff,4))/mean(M{5}(ff,4));   % razmerje povprecnih casov tMks
        ZetaNit{m}(i) =mean(M{m}(ff,5))/mean(M{5}(ff,5));
        
        
    end
    
end



for m=1:Mfin
    figure(1), hold on, plot(Nagv,Eta{m}*100,Mcolor{m})%, xlabel('Nagv [1]'), ylabel('Eta [%]')
    figure(2), hold on, plot(Nagv,ZetaTSoc{m}(:),Mcolor{m}), xlabel('Nagv [1]'), ylabel('ZetaTSoc [1]')
    figure(3), hold on, plot(Nagv,ZetaTMks{m}(:),Mcolor{m}), xlabel('Nagv [1]'), ylabel('ZetaTMks [1]')
    figure(4), hold on, plot(Nagv,ZetaNit{m}(:), Mcolor{m}), xlabel('Nagv [1]'), ylabel('ZetaNit [1]')
    figure(5), hold on, plot(Nagv,Thr{m}(:), Mcolor{m}), xlabel('Nagv [1]'), ylabel('Throughput [1/h]')
    
    figure(6), hold on, plot(Nagv,AvTSoc{m}(:),Mcolor{m}), xlabel('Nagv [1]'), ylabel('ZetaTMks [1]')
    figure(7), hold on, plot(Nagv,AvTMks{m}(:), Mcolor{m}), xlabel('Nagv [1]'), ylabel('ZetaNit [1]')
    figure(8), hold on, plot(Nagv,AvNit{m}(:), Mcolor{m}), xlabel('Nagv [1]'), ylabel('Throughput [1/h]')
end



figure(1),legend(Mlabel);
xlabel('$$N_{AGV}$$ [1]','interpreter','latex','FontSize',12), ylabel('$$\eta$$ [$$\%$$]','interpreter','latex','FontSize',12),
hold off

figure(2),legend(Mlabel);
xlabel('$$N_{AGV}$$ [1]','interpreter','latex','FontSize',12), ylabel('$$\zeta_{t_{SoC}}$$ [1]','interpreter','latex','FontSize',12),
hold off

figure(3),legend(Mlabel);hold off
xlabel('$$N_{AGV}$$ [1]','interpreter','latex','FontSize',12), ylabel('$$\zeta_{t_{Mks}}$$ [1]','interpreter','latex','FontSize',12),
hold off

figure(4),legend(Mlabel);
xlabel('$$N_{AGV}$$ [1]','interpreter','latex','FontSize',12), ylabel('$$\zeta_{N_{it}}$$ [1]','interpreter','latex','FontSize',12),
hold off

figure(5),legend(Mlabel);
xlabel('$$N_{AGV}$$ [1]','interpreter','latex','FontSize',12), ylabel('$$T$$ [1/h]','interpreter','latex','FontSize',12),
hold off


if 1
    figure(6),legend(Mlabel);
    xlabel('$$N_{AGV}$$ [1]','interpreter','latex','FontSize',12), ylabel('$$t_{SoC}$$ [1]','interpreter','latex','FontSize',12),
    hold off
    
    figure(7),legend(Mlabel);hold off
    xlabel('$$N_{AGV}$$ [1]','interpreter','latex','FontSize',12), ylabel('$$t_{Mks}$$ [1]','interpreter','latex','FontSize',12),
    hold off
    
    figure(8),legend(Mlabel);
    xlabel('$$N_{AGV}$$ [1]','interpreter','latex','FontSize',12), ylabel('$${N_{it}}$$ [1]','interpreter','latex','FontSize',12),
    hold off
end



%====================================================
% generate image for article
figure(11),
for m=2:Mfin
    subplot(4,1,1),hold on
    plot(Nagv,ZetaTSoc{m}(:),Mcolor{m})
    set(gca,'xlim',[Nagv(1),Nagv(end)])
    
    subplot(4,1,2),hold on
    plot(Nagv,ZetaTMks{m}(:),Mcolor{m})
    set(gca,'xlim',[Nagv(1),Nagv(end)])
    
    %      subplot(4,1,3),hold on
    %      plot(Nagv,Thr{m}(:), Mcolor{m})
    %      set(gca,'xlim',[Nagv(1),Nagv(end)])
    
    subplot(4,1,3),hold on
    %  semilogy(Nagv,ZetaNit{m}(:), Mcolor{m})
    semilogy(Nagv,AvNit{m}(:), Mcolor{m})
    set(gca, 'YScale', 'log') %  you can explicitly force it to be logarithmic
    
    set(gca,'xlim',[Nagv(1),Nagv(end)])
    
    subplot(4,1,4),hold on
    plot(Nagv,Eta{m}*100,Mcolor{m})
    set(gca,'xlim',[Nagv(1),Nagv(end)])
end
figure(11);
subplot(4,1,1)
ylabel('$$\zeta_{t_{SoC}}$$ [1]','interpreter','latex','FontSize',12)
subplot(4,1,2)
ylabel('$$\zeta_{t_{Mks}}$$ [1]','interpreter','latex','FontSize',12)
subplot(4,1,3)
%ylabel('$$T$$ [1/h]','interpreter','latex','FontSize',12),
ylabel('$$\log_{10}{N_{itAv}}$$ [1]','interpreter','latex','FontSize',12),

subplot(4,1,4)
ylabel('$$\eta$$ [$$\%$$]','interpreter','latex','FontSize',12),
xlabel('$$N_{AGV}$$ [1]','interpreter','latex','FontSize',12)
legend(Mlabel(2:end),'location','west')

if 0 % shranim slike za clanek
    % set(gcf,'PaperPosition',[1 1 4 5]), print -depsc  ResultMwrh1Single;
    % set(gcf,'PaperPosition',[1 1 4 5]), print -depsc  ResultMwrh2Single;
    % set(gcf,'PaperPosition',[1 1 4 5]), print -depsc  ResultMgameSingle;
end







if 0   % generiraj tabelo za v clanek
    alg={'ASTAR','SIPP','PSIPPc','PSIPPl','CCBS'};
    ee='\\';
    % for r=2:1:size(TABELA_SoC,1)
    for r=[2 4 6 7 8 9 10]     % wrh1
        % for r=[2 20:3:36]   % wrh2
        % for r=[2 15:3:31 31]     % game
        niz=strcat('\parbox[t]{2mm}{\multirow{4}{*}{\rotatebox[origin=c]{0}{',num2str(r),'}}}');
        disp(niz);
        for m=2:5
            fprintf('& %8s & $%10.3f$ & $%10.3f$ & $%12.1f$ & $%10.1f$ %s \n',alg{m},ZetaTSoc{m}(r) , ZetaTMks{m}(r), AvNit{m}(r) , Eta{m}(r)*100,ee);
        end
        disp('\midrule');
    end
    
end








