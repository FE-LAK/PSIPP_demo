    coor=nan(2,7);  % nodes coordinates
    coor(:,1)=[0;0]*2;
    coor(:,2)=[0;2]*2;
    coor(:,3)=[2;1]*2;
    coor(:,4)=[2;2]*2;
    coor(:,5)=[3;4]*2;
    coor(:,6)=[4;1]*2;
    coor(:,7)=[4;2]*2;
    
    nID1=1; nID2=2; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    agvSim.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    
    nID1=2; nID2=4; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    agvSim.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )

    nID1=3; nID2=4; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    agvSim.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    
    nID1=4; nID2=7; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    agvSim.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    
    nID1=6; nID2=7; p1=coor(:,nID1);p2=coor(:,nID2); rID12=nID1*100+nID2; twoway=1; rID21=nID2*100+nID1;
    secShape='L';
    secParam=[ p1' p2' norm(p1'-p2')]; % last parameter is sector length
    agvSim.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 )
    
    nID1=1; nID2=2; p1=[0;0]; p2=[0;4]; rID12=1200; twoway=0; rID21=nan;
    secShape=['A']; secParam=[0 0 -pi -pi pi*2];
    agvSim.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 );

    
    nID1=1; nID2=3; p1=coor(:,nID1); p2=coor(:,nID2); rID12=13; twoway=0; rID21=31;
    secShape=['L','A']; 
    secParam=[0 0 2 0 2;...      %L=[x1,y1,x2,y2,d], 
              2 0 0 pi/2 pi ]; %A=[x0,y0,f0,df,dd] 
    agvSim.addTwoNodesAndConnection(nID1, nID2, p1,p2, rID12,secShape,secParam, twoway, rID21 );
    
      
    agvSim.precomputeMap();
    agvSim.drawMap();
