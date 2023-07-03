function params = getParamsFromTable(paramsTable)
    % GETPARAMSFROMTABLE - Get the parameters from a table to a cell array
    % with labels that looks like this:
    %   {'var1', value1, 'var2', value2, ...}
    %
    % Inputs:
    %   paramsTable: Table containing the parameters
    %
    % Outputs:
    %   params: Cell array containing the parameters with labels
    fields = paramsTable.Properties.VariableNames;
    numParams = numel(fields);
    paramsNoLabel = table2cell(paramsTable);

    params = cell(1, 2*numParams); 
    for p = 1:numParams
        params{2*p-1} = fields{p};
        params{2*p} = paramsNoLabel{p};
    end
end