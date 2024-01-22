close all, clear all,  %format bank

mapChoice=3;   %map choices: 1 = Mwrh1, 2=Mwrh2, 3= Mgame



NbatcOff=1; % koliko zadnji batchov ignoriram

switch mapChoice
    
    case 1 % simple mapa 10agv 52 ponovitev
        load('Results/Chained/ResultLLC_Mwrh1_M4.mat');  % PSIPPL  % 
        M=load('Results/Chained/ResultLLC_Mwrh1_M5.txt','-ascii');  % CCBS LL  
        
        Ntask = size(taskSol,2); % skupno stevilo single taskkov
        Nbatch =size(M,1);      % stevilo batch klicev CCBS vsak po Nagv agentov
        
        Ntask = Ntask-Nagv*NbatcOff; % skupno stevilo single taskkov
        Nbatch =Nbatch-NbatcOff;      % stevilo batch klicev CCBS vsak po Nagv agentov
        
        %NitCompare =2042.00    4154110.00
        %tMksCompare =1788.04       1703.79
        
    %===================================    
    case 2      % Diganni 20 AGV 52 ponovitev iz slepih ulic
        load('Results/Chained/ResultLLC_Mwrh2_M4.mat');  % PSIPPL   
        M=load('Results/Chained/ResultLLC_Mwrh2_M5.txt','-ascii');  % CCBS LL  
        
        Ntask = size(taskSol,2); % skupno stevilo single taskkov
        Nbatch =size(M,1);      % stevilo batch klicev CCBS vsak po Nagv agentov
        
        Ntask = Ntask-Nagv*NbatcOff; % skupno stevilo single taskkov
        Nbatch =Nbatch-NbatcOff;      % stevilo batch klicev CCBS vsak po Nagv agentov
        
        %NitCompare =2040.00     595250.00
        %tMksCompare =3561.36       6856.49
        
    %===================================    
    case 3 % den520d 20 agv 52 SINGLE task ponovitev iz slepih ulic
        load('Results/Chained/ResultLLC_Mgame_M4.mat');  % PSIPPL  
        M=load('Results/Chained/ResultLLC_Mgame_M5.txt','-ascii');  % CCBS LL  
        
        Ntask= size(taskSol,2); % skupno stevilo single taskkov
        Nbatch =size(M,1);      % stevilo batch klicev CCBS vsak po Nagv agentov
        
        Ntask = Ntask-Nagv*NbatcOff; % skupno stevilo single taskkov
        Nbatch =Nbatch-NbatcOff;      % stevilo batch klicev CCBS vsak po Nagv agentov
        
        %       NitCompare =2616.00    3612328.00
        %       tMksCompare =14105.74      20568.55
        
        % zanimivo:  na Fifure 3 se proti koncu blizu 1000 taskov T_mks zacne povecevat bolj. To je morda zato, ker gre vec vozil v SL in ocitno rata tam ozko grlo in ovira druge. ? cudno
        
end


%=======================================================








if 1  % izris in primerjava PSIPLL s CCBS
    
    % cas izvedbe za stevilo opravljenih taskov (niso po vrsti)
    [tmksSort]=sort([taskSol(1:Ntask).tDrop])';
    figure(3),plot(1:Ntask,tmksSort)
    xlabel('$$N_{task}$$ [1]','interpreter','latex','FontSize',12), ylabel('$$t_{Mks}$$ [s]','interpreter','latex','FontSize',12),
    
    % task sekvenca, cas potreben da se izvedejo vsi taski do trenutnega
    TMKS=[];
    for n=1:Ntask %size(Tasks,1)
        TMKS=[TMKS; max( [taskSol(1:n).tDrop] )];
    end
    figure(4),plot(1:Ntask,TMKS)
    xlabel('$$N_{taskSequnce}$$ [1]','interpreter','latex','FontSize',12), ylabel('$$t_{Mks}$$ [1]','interpreter','latex','FontSize',12),
    
    
    % taski po vrsti in njihovi casi izvedbe
    figure(5),plot(1:Ntask,[taskSol(1:Ntask).tDrop])
    xlabel('$$Task_{ID}$$ [1]','interpreter','latex','FontSize',12), ylabel('$$t_{fin}$$ [1]','interpreter','latex','FontSize',12),
    
    
    
    
    
    %===== primerjava s CCBS
    times=[]; % drop times
    for i=1:Nbatch %size(M,1)
        times=[times;M(i,8:end)'];
    end
    
    tmksSortCCBS=sort([times]);
    figure(3),hold on,plot((1:Nbatch*Nagv),tmksSortCCBS,'r'),hold off
    legend('PSIPPl','CCBS')
    
    TMKScbs=[];
    for n=1:Ntask %size(Tasks,1)
        TMKScbs=[TMKScbs; max( [times(1:n)] )];
    end
    figure(4),hold on,plot(1:Nbatch*Nagv,TMKScbs,'r'), hold off
    legend('PSIPPl','CCBS')
    
    
    figure(5),hold on,plot(1:Nbatch*Nagv,times,'r'), hold off
    legend('PSIPPl','CCBS')
    
    
    % disp('Avarage Time per Task');
    % TtaskPSIPP=diff([taskSol(1:Ntask).tDrop])
    
    %     TtaskPSIPP=mean(diff(tmksSort))
    
    %     TtaskCCBS=mean(diff(timesSort))
    
    
    figure(6), plot(1:Ntask,tmksSort./tmksSortCCBS)
    %   xlabel('$$N_{taskDone}$$ [1]','interpreter','latex','FontSize',12), ylabel('$$t_{PSIPPl}/t_{CCBS}$$ [1]','interpreter','latex','FontSize',12),
    %  title('Ratio of delivery times')
    xlabel('$$N_{task}$$ [1]','interpreter','latex','FontSize',12), ylabel('$$t_{Mks,PIPPl}/t_{Mks,CCBS}$$ [1]','interpreter','latex','FontSize',12),
    
    povprecjeRazmerij=mean(tmksSort./tmksSortCCBS)
    
    %    mean(diff(tmksSort))
    %    mean(diff(tmksSortCCBS))
    %    mean(diff(tmksSort))/mean(diff(tmksSortCCBS))
    
    
    tAvPSIPP=tmksSort(end)/Ntask
    tAvCCBS=tmksSortCCBS(end)/Ntask
    
    razmerjeCasovDostav=tmksSort(end)/tmksSortCCBS(end)
    
    %tAvPSIPP/tAvCCBS
    
    
    izboljsavaGledeCCBS_Procent=(tmksSortCCBS(end)-tmksSort(end))/tmksSortCCBS(end)*100
    
    
    NitCompare= [ PSIPPL(2)-NbatcOff*Nagv*2 M(end-NbatcOff,7)]
    tMksCompare=[ tmksSort(end) tmksSortCCBS(end)]
    
end






if 0 % shranim slike za clanek
    figure(7),
    subplot(2,1,1)
    plot(1:Ntask,tmksSort, (1:Nbatch*Nagv),tmksSortCCBS,'r' )
    axis([0 510 0 1800])
    ylabel('$$t_{Mks}$$ [s]','interpreter','latex','FontSize',12),
    legend('PSIPPl','CCBS')
    
    set(gca,'xticklabel','')
    
    subplot(2,1,2)
    plot(1:Ntask,tmksSort./tmksSortCCBS)
    axis([0 510 0.6 1.2])
    ylabel('$${t_{Mks_{PIPPl}}}/{t_{Mks_{CCBS}}}$$ [1]','interpreter','latex','FontSize',12),
    xlabel('$$N_{task}$$ [1]','interpreter','latex','FontSize',12),
    
    % set(gcf,'PaperPosition',[1 1 4 3]), print -depsc  ResultMwrh1LLchained;
end






