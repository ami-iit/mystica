function p = getMysticaFullPath()
    initial_path = pwd;
    cd(fullfile(fileparts(mfilename('fullpath')),'..','..'));
    p = pwd;
    cd(initial_path);
end
