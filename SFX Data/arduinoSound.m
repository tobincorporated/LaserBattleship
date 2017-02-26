[y, Fs]= ...
    audioread('C:\Users\Cody Prestwood\Google Drive\PEF Personal\SFX\laserPew.wav');

Fnew=8000;
ynew = zeros(1,9000);



for i = 1:length(ynew)
    if(floor(Fs/Fnew*i) <= length(y))
        ynew(i)=y(floor(i*Fs/Fnew));
    end
    
end

yfin=round((ynew+1)*64);

dlmwrite('laserPew.txt',yfin,',');

