function [] = generate_validation_sum(filename_validation_sum, ...
                                      filename_text_sum, ...
                                      filename_motor_properties, ...                        
                                      THROTTLE_TEST, err_thrust_pct, err_current_pct)

    % open finale protocol file for writing
    fid = fopen(filename_validation_sum, 'w');
    if fid == -1
        error('Datei konnte nicht geöffnet werden: %s', filename_validation_sum);
    end

    % open the test summary text file for reading
    test_text_fid = fopen(filename_text_sum, 'r');
    if test_text_fid == -1
        warning('Test summary file not found: %s', filename_text_sum);
        test_content_available = false;
    else
        test_content_available = true;
    end

    % generate the current date and time for the protocol
    gen_time = datestr(now, 'yyyy-mm-dd HH:MM:SS');
    
    fprintf(fid, '# Validation Protocol & Test Results\n\n');
    fprintf(fid, '**Generated on:** %s\n', gen_time);
    fprintf(fid, '\n## 1. HIL Test Summary (POSIX/STM32)\n\n'); 
    fprintf(fid, ' ### Motor-Model Test Bench: BrotherHobby 1404 KV4600+T4030 (2S)\n\n');   

    % add csv table from motor_properties to fid    
    % 'Delimiter', ';'
    raw_text = fileread(filename_motor_properties);
    clean_text = strrep(raw_text, '"', '');
    lines = strsplit(clean_text, '\n');

    fprintf(fid, '| ID | Parameter | Value | Unit |\n');
    fprintf(fid, '|----|-----------|-------|------|\n');

        for i = 2:length(lines) % Начинаем со 2-й строки, чтобы пропустить шапку
            line = strtrim(lines{i});
            if isempty(line), continue; end
    
            parts = strsplit(line, ';');
            if length(parts) >= 4
                fprintf(fid, '| %s | %s | %s | %s |\n', ...
                strtrim(parts{1}), strtrim(parts{2}), strtrim(parts{3}), strtrim(parts{4}));
            end
        end

    fprintf(fid, '\n\n');
    
    % add text from test_text to fid
    while ~feof(test_text_fid)
        line = fgetl(test_text_fid);
        if ischar(line) % Check if line is valid
            fprintf(fid, '%s\n', line);
        end
    end
    fclose(test_text_fid);    
    fprintf(fid, '\n\n');

    fprintf(fid, '## 2. Accuracy Analysis (Octave vs POSIX/STM32)\n\n');
    fprintf(fid, '**Max Thrust Error:** %.2f%%\n', max(err_thrust_pct));
    fprintf(fid, '**Max Current Error:** %.2f%%\n\n', max(err_current_pct));

    fprintf(fid, '| Throttle | Thrust Error %% | Current Error %% |\n');
    fprintf(fid, '|----------|----------------|----------------|\n');
    for i = 1:length(THROTTLE_TEST)
        fprintf(fid, '| %.2f | %.2f | %.2f|\n', THROTTLE_TEST(i), err_thrust_pct(i), err_current_pct(i));
    end
    fprintf(fid, '\n');
    
    % show plots of errors
    fprintf(fid, '![Discrepancy between real stand data and Motor-Model](%s)\n', './octave/plots/plot4_errors.png');
    fprintf(fid, '\n');

    fprintf('Report saved to: %s\n', filename_validation_sum);
    fprintf(fid, '*Document Version: v.0.3.0 | Part of LIGHT-MBSE-PIPELINE-SKELETON*\n');  
    
    
    fclose(fid);

end