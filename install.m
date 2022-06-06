function install(input)
    arguments
        input.mambaforge_prefix char = ''
        input.env_name          char = 'mystica'
    end
    % function created inspired by https://github.com/robotology/robotology-superbuild/blob/master/scripts/install_robotology_packages.m

    mystica_fullpath = fileparts(mfilename('fullpath'));
    install_prefix   = fullfile(mystica_fullpath,'deps');
    setup_script     = fullfile(mystica_fullpath,'deps','setup.m');

    matlab_path_env = '';

    if exist(install_prefix,'dir')
        fprintf('Directory %s already exists.\n', install_prefix);
        fprintf('Please use it or delete to proceed with the install.\n');
        return;
    end

    if ispc
        env_sep = ";";
    else
        env_sep = ":";
    end

    %% Installing Binaries via conda

    fprintf('Installing MATLAB/Simulink binaries in %s\n', install_prefix);

    [mamba_full_path,env_full_path] = configure_mambaforge(input.mambaforge_prefix,install_prefix,input.env_name);

    % Install all the packages via conda
    fprintf('Installing packages\n');
    system(sprintf('"%s" create -n "%s" -y -c conda-forge -c robotology casadi-matlab-bindings "libblas=*=*openblas"\n', mamba_full_path,input.env_name));
    % see discussion https://github.com/ami-iit/element_morphing-cover-design/issues/215#issuecomment-1081515249 to understand why we added "libblas=*=*openblas"
    fprintf('Installation of packages completed\n');

    if ispc
        pckgs_install_prefix = fullfile(env_full_path,'Library');
    elseif isunix
        pckgs_install_prefix = fullfile(env_full_path);
    end

    %% Configure GitHub repositories

    fprintf('Installing GitHub repositories\n');
    clone_git_repository('https://github.com/ewiger/yamlmatlab.git','deps')
    matlab_path_env = strcat( fullfile(install_prefix,'yamlmatlab') , env_sep , matlab_path_env );
    system(sprintf('"%s" env config vars set MATLABPATH="%s" -p "%s"',mamba_full_path,matlab_path_env,env_full_path));
    fprintf('Installation of GitHub repositories completed\n');

    %% Addpath mystica directory

    fprintf('Installing mystica root folder\n')
    matlab_path_env = strcat( mystica_fullpath , env_sep , matlab_path_env );
    system(sprintf('"%s" env config vars set MATLABPATH="%s" -p "%s"',mamba_full_path,matlab_path_env,env_full_path));
    fprintf('Installing mystica root folder completed\n')

    %% Addpath mystica/meshes directory

    fprintf('Installing mystica meshes folder\n')
    mystica_meshes_fullpath = fullfile(mystica_fullpath,'meshes');
    matlab_path_env = strcat( mystica_meshes_fullpath , env_sep , matlab_path_env );
    system(sprintf('"%s" env config vars set MATLABPATH="%s" -p "%s"',mamba_full_path,matlab_path_env,env_full_path));
    fprintf('Installing mystica meshes folder completed\n')

    %% Creation of setup.m

    fprintf('Creating setup script in %s\n', setup_script);
    setupID = fopen(setup_script,'w');
    fprintf(setupID,'%% Specify OS-specific locations\n');
    fprintf(setupID,'if ispc\n');
    fprintf(setupID,'    env_sep = ";";\n');
    fprintf(setupID,'else\n');
    fprintf(setupID,'    env_sep = ":";\n');
    fprintf(setupID,'end\n');
    fprintf(setupID,'\n');
    fprintf(setupID,'%% Install prefix (hardcoded at generation time)\n');
    fprintf(setupID,'pckgs_install_prefix    = "%s";\n', pckgs_install_prefix);
    fprintf(setupID,'install_prefix          = "%s";\n', install_prefix);
    fprintf(setupID,'mystica_fullpath        = "%s";\n', mystica_fullpath);
    fprintf(setupID,'mystica_meshes_fullpath = "%s";\n', mystica_meshes_fullpath);
    fprintf(setupID,'\n');
    fprintf(setupID,'%% AddPath packages installed with conda\n');
    fprintf(setupID,'addpath(fullfile(pckgs_install_prefix,"mex"));\n');
    fprintf(setupID,'\n');
    fprintf(setupID,'%% AddPath github repositories\n');
    fprintf(setupID,'addpath(fullfile(install_prefix,"yamlmatlab"));\n');
    fprintf(setupID,'\n');
    fprintf(setupID,'%% AddPath mystica\n');
    fprintf(setupID,'addpath(mystica_fullpath);\n');
    fprintf(setupID,'\n');
    fprintf(setupID,'%% AddPath mystica meshes\n');
    fprintf(setupID,'addpath(mystica_meshes_fullpath);\n');
    fprintf(setupID,'\n');
    fprintf(setupID,'%% Add to the env:"PATH" the directory with the packages installed with conda\n');
    fprintf(setupID,'setenv("PATH",strcat(fullfile(pckgs_install_prefix,"bin"), env_sep, getenv("PATH")));\n');
    fclose( setupID);

    fprintf('packages are successfully installed!\n');
    fprintf('Please run %s before using the packages,\n',setup_script)
    fprintf('or activate the conda enviroment %s and open matlab from that terminal.\n',input.env_name);
    fprintf('To uninstall these packages, just delete the folders %s and %s .\n', install_prefix,env_full_path);

end

function clone_git_repository(repository_url,install_prefix)
    arguments
        repository_url
        install_prefix
    end

    [~,nameRepo] = fileparts(repository_url);
    disp([nameRepo,':'])
    if ~isfolder([install_prefix,filesep,nameRepo])
        system(['git clone ',repository_url,' ',install_prefix,filesep,nameRepo]);
    else
        disp('repository exists')
        system(['git -C ',install_prefix,filesep,nameRepo,' pull']);
    end
end

function [mamba_full_path,env_full_path] = configure_mambaforge(mambaforge_prefix,install_prefix,env_name)

    if ispc
        mamba_full_path = fullfile(mambaforge_prefix,'mambaforge','condabin','mamba.bat');
    elseif isunix
        mamba_full_path = fullfile(mambaforge_prefix,'mambaforge','bin','mamba');
    end

    if exist(mamba_full_path,'file')
        fprintf('mambaforge already installed\n');
        env_full_path = fullfile(mambaforge_prefix,'mambaforge','envs',env_name);
    else
        % The install url is created following
        mambaforge_url_prefix = 'https://github.com/conda-forge/miniforge/releases/latest/download/';
        if ispc
            mambaforge_installer_name = 'Mambaforge-Windows-x86_64.exe';
        elseif ismac
            [~, uname_m] = system('uname -m');
            % Remove newline
            uname_m = strip(uname_m);
            mambaforge_installer_name = sprintf('Mambaforge-MacOSX-%s.sh', uname_m);
        elseif isunix
            [~, uname] = system('uname');
            % Remove newline
            uname = strip(uname);
            [~, uname_m] = system('uname -m');
            % Remove newline
            uname_m = strip(uname_m);
            mambaforge_installer_name = sprintf('Mambaforge-%s-%s.sh', uname, uname_m);
        end

        fprintf('Downloading mambaforge installer \n');
        mambaforge_installer_url = strcat(mambaforge_url_prefix, mambaforge_installer_name);
        websave(mambaforge_installer_name, mambaforge_installer_url);
        fprintf('Download of mambaforge installer completed\n');

        % See https://github.com/conda-forge/miniforge#non-interactive-install
        fprintf('Installing mambaforge\n');
        if ispc
            system(sprintf('start /wait "" %s /InstallationType=JustMe /RegisterPython=0 /S /D=%s', mambaforge_installer_name, install_prefix));
            mamba_full_path = fullfile(install_prefix,'condabin','mamba.bat');
            % On Windows, the files in conda are installed in the Library
            % subdirectory of the prefix
        elseif isunix
            system(sprintf('sh %s -b -p "%s"', mambaforge_installer_name, install_prefix));
            assert(length(['#!',install_prefix,'/bin python'])<=127,'install_prefix path is too long! Shebangs cannot be longer than 127 characters (see https://github.com/robotology/robotology-superbuild/pull/1145)')
            mamba_full_path = fullfile(install_prefix,'bin','mamba');
        end
        fprintf('Installation of mambaforge completed\n');

        if ~exist(install_prefix, 'dir')
            fprintf('Installation in %s failed for unknown reason.\n', install_prefix);
            fprintf('Please open an issue at https://github.com/ami-iit/element_morphing-cover-design/issues/new .\n');
            return;
        end

        fprintf('Deleting mambaforge installer\n');
        delete(mambaforge_installer_name);

        env_full_path = fullfile(install_prefix,'envs',env_name);

    end

end
