% Crea un video da una serie di frame. Vuole in input la serie di frame (F)
% il framerate (framerate) e il nome/indirizzo del file da salvare
% sottoforma di stringa (name). Esempio di utilizzo

% MakeVideo_frames(FF,length(FF)/video_time,'C:\MyFolder\Myfile')

function [] = MakeVideo_frames(F,framerate,name)
writerObj = VideoWriter(name);
writerObj.FrameRate = framerate;
open(writerObj);
% write the frames to the video
for i=1:length(F)
    % convert the image to a frame
    i
    frame = F(i) ;    
    writeVideo(writerObj, frame);
end
% close the writer object
close(writerObj);
end

