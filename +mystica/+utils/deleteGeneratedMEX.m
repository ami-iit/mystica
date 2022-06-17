function deleteGeneratedMEX()
    delFnc('mystica_stateKin')
end
function delFnc(name)
    d = dir([name,'.mex*']);
    name_full = d.name;
    if exist(name_full,'file')==3
        delete(name_full)
        fprintf('%s deleted\n',name_full)
    end
end

