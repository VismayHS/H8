%% RUN_ALL - HACKSIMUL8 2026 complete solution driver
%  Runs Tasks 1-4 in order and reports timing.
%
%  Put every file from this folder on the MATLAB path (or cd into it), then
%  just type:  run_all
%
%  Expected runtime on an i5-12450H:
%     Task 1   ~2 s
%     Task 2   ~30-60 s   (Method D optimisation)
%     Task 3   ~5-12 min  (300 optimisations + NN training)  <-- the long one
%     Task 4   ~1-2 min
%
%  IF YOU ARE SHORT ON TIME: open task3 and drop N from 300 to 120. Accuracy
%  falls a little, the story does not change, and you save several minutes.

clear; clc; close all;
here = fileparts(mfilename('fullpath'));
if ~isempty(here), cd(here); end

fprintf('\n');
fprintf('==========================================================\n');
fprintf('  HACKSIMUL8 2026 - 6-DOF UAV Quadcopter\n');
fprintf('  PID altitude control with ML-based self-tuning\n');
fprintf('==========================================================\n\n');

T = tic;

fprintf('>>> TASK 1: dynamic model + state space\n');
t1 = tic; task1_statespace; fprintf('    [%.1f s]\n\n', toc(t1));

fprintf('>>> TASK 2: altitude PID design and tuning comparison\n');
t2 = tic; task2_altitude_pid; fprintf('    [%.1f s]\n\n', toc(t2));

fprintf('>>> TASK 3: data generation + ML training\n');
t3 = tic; task3_generate_data_train_ml; fprintf('    [%.1f s]\n\n', toc(t3));

fprintf('>>> TASK 4: ML self-tuner test cases\n');
t4 = tic; task4_test_ml_selftuning; fprintf('    [%.1f s]\n\n', toc(t4));

fprintf('==========================================================\n');
fprintf('  ALL TASKS COMPLETE in %.1f s\n', toc(T));
fprintf('  Figures are open; .mat files are saved in this folder.\n');
fprintf('==========================================================\n');
