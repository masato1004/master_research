function func__progress(title, i, imax)
    persistent strlen
    if isempty(strlen)
        strlen = 0;
    end
    Tmp = {': %3d/%d\n', i, imax};
    Tmp{1} = [ repmat(sprintf('\b'),[1 strlen]),  Tmp{1} ];

    Txt = sprintf(Tmp{1:3});
    fprintf(title+Txt);
    strlen = length(char(title))+length(Txt) - strlen;
end