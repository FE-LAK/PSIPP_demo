function clearOccupancies(obj)
    roads=obj.roadID.keys;
    nodes=obj.nodeID.keys;
    for i=1:obj.roadID.Count
        st=obj.roadID(roads{i});
        st.occupancy=[];
        st.occupancy2=[];
        obj.roadID(roads{i})=st;
    end
    for j=1:obj.nodeID.Count
        st=obj.nodeID(nodes{j});
        st.occupancy=[];
        obj.nodeID(nodes{j})=st;
    end
end