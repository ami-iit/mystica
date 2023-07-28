function s = getCommitHash
    initialPath = pwd;
    cd(fileparts(mfilename('fullpath')))
    [~,msg]=system('git status');
    if ~contains(msg,'nothing to commit, working tree clean')
        warning('Local modification!')
    end
    [~,hash] = system('git rev-parse HEAD');
    s = ['mystica: https://github.com/ami-iit/mystica/commit/',hash(1:end-1)];
    cd(initialPath)
end
