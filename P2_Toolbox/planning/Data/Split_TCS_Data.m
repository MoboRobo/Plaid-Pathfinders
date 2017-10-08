% Splits Up a TCS Data File of the Given Scale into Smaller Sub-Files.
function Split_TCS_Data(scale)
    load(strcat('cubicSpirals_Data_',num2str(scale)),'a1Tab','a2Tab','b1Tab','b2Tab','r1Tab','r2Tab');
    
    save(strcat('cubicSpirals_Data_',num2str(scale),'_A'),'a1Tab','a2Tab');
    save(strcat('cubicSpirals_Data_',num2str(scale),'_B'),'b1Tab','b2Tab');
    save(strcat('cubicSpirals_Data_',num2str(scale),'_r'),'r1Tab','r2Tab');
end