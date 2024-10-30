function varargout = pyListUnpack(inputStructs)
%PYLISTUNPACK Unpacks a list of tensors into separate outputs

import model_best_epoch_99_pth_tar.ops.*

varargout = cell(1, numel(inputStructs));
for i=1:numel(inputStructs)
    varargout{i} = inputStructs(i);
end
end

