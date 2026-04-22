function [THROTTLE_TEST, THRUST_TEST_N, CURRENT_TEST_A] = parse_test_table(filename)

    printf('Parsing motor properties from file: %s\n', filename);

    THROTTLE_TEST = [];
    THRUST_TEST_N = [];
    CURRENT_TEST_A = [];

    raw_data = dlmread(filename, ';', 1, 0); % Skip the header row (1) and start from the first column (0)

    printf('Parsed %d rows of test data.\n', size(raw_data, 1));

    if size(raw_data, 2) < 2
        error('Expected at least 3 columns in the test data CSV file.');
    end

    THROTTLE_TEST = raw_data(:, 1);
    THRUST_TEST_N = raw_data(:, 2);
    CURRENT_TEST_A = raw_data(:, 3);

end
