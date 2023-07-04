%% Script to test the function logic.pointcloud.scalingFactorFromRoomHeight
%

% Create multiple test cases with a floor, ceiling plane, height and expected output
% then loop through the test cases and check the output of the function
% against the expected output

% Test cases {floor, ceiling, height, expected output}
testCases = {
    {planeModel([0, 0, 1, 0]), planeModel([0, 0, 1, 1]), 2.8, 2.8}, ...
    {planeModel([0, 1, 1, 0]), planeModel([0, 1, 1, 2]), 2.8, 1.979898987}, ...
    {planeModel([0.1, 1, 1, 1]), planeModel([0, 1, 1, 3]), 2.8, 1.97}, ... % Slightly tilted floor plane
};
eps = 1e-2; % Tolerance for the test

% Loop through the test cases
for i = 1:length(testCases)
    % Get the test case
    testCase = testCases{i};
    
    % Get the floor plane
    floorPlane = testCase{1};
    
    % Get the ceiling plane
    ceilingPlane = testCase{2};
    
    % Get the height
    height = testCase{3};
    
    % Get the expected output
    expectedOutput = testCase{4};
    
    % Get the output of the function
    output = logic.pointcloud.scalingFactorFromRoomHeight(floorPlane, ceilingPlane, height);
    disp("Output of test case " + string(i) + ": " + string(output) + " Expected output: " + string(expectedOutput) + ".")
    
    % Check the output against the expected output
    assert(abs(output - expectedOutput) / expectedOutput < eps);
end