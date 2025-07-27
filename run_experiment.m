% 运行显式折扣方法实验

clear all; close all; clc;

fprintf('显式折扣方法实验\n');
fprintf('========================================\n\n');

% 设置随机种子
rng(42);

% ========== 运行算法 ==========
fprintf('开始运行算法...\n\n');

tic;
[K_final, gamma_history, cost_history, iteration_results] = explicit_discount_method();
elapsed_time = toc;

fprintf('\n算法完成! 运行时间: %.2f 秒\n', elapsed_time);
fprintf('========================================\n\n');

% ========== 结果分析 ==========
fprintf('结果分析:\n\n');

% 最终控制器
fprintf('学到的控制器: K = [%.6f, %.6f]\n', K_final(1,1), K_final(1,2));

% 验证稳定性
A = [4, 3; 3, 1.5];
B = [2; 2];
A_cl = A - B * K_final;
rho_cl = max(abs(eig(A_cl)));
fprintf('闭环系统谱半径: %.6f\n', rho_cl);

if rho_cl < 1
    fprintf('系统稳定 ✓\n');
else
    fprintf('系统不稳定 ✗\n');
end

% 与理论最优解比较
try
    Q = eye(2);
    R = 2;
    [K_opt, ~, ~] = dlqr(A, B, Q, R);
    
    fprintf('\n理论最优控制器: K_opt = [%.6f, %.6f]\n', K_opt(1,1), K_opt(1,2));
    
    error_norm = norm(K_final - K_opt, 'fro');
    relative_error = error_norm / norm(K_opt, 'fro');
    fprintf('相对误差: %.4f%%\n', relative_error * 100);
    
catch
    fprintf('\n无法计算理论最优解\n');
end

% 算法统计
fprintf('\n收敛统计:\n');
fprintf('外循环次数: %d\n', length(gamma_history));
fprintf('初始折扣因子: %.6f\n', gamma_history(1));
fprintf('最终折扣因子: %.6f\n', gamma_history(end));
fprintf('初始成本: %.4f\n', cost_history(1));
fprintf('最终成本: %.4f\n', cost_history(end));

% ========== 可视化 ==========
fprintf('\n生成图表...\n');

figure('Position', [100, 100, 1200, 800]);

% 折扣因子演化
subplot(2, 3, 1);
plot(0:(length(gamma_history)-1), gamma_history, 'b-o', 'LineWidth', 2);
grid on;
xlabel('迭代次数');
ylabel('折扣因子');
title('折扣因子演化');
set(gca, 'YScale', 'log');
hold on;
plot([0, length(gamma_history)-1], [1, 1], 'r--');
legend('算法轨迹', '\gamma = 1');

% 成本演化
subplot(2, 3, 2);
plot(0:(length(cost_history)-1), cost_history, 'g-s', 'LineWidth', 2);
grid on;
xlabel('迭代次数');
ylabel('成本估计');
title('成本演化');

% 更新率演化
subplot(2, 3, 3);
alphas = [iteration_results.alpha];
plot(1:length(alphas), alphas, 'm-^', 'LineWidth', 2);
grid on;
xlabel('迭代次数');
ylabel('更新率');
title('更新率演化');

% 控制器增益演化
subplot(2, 3, 4);
K_history = cat(3, iteration_results.K);
K1_history = squeeze(K_history(1, 1, :));
K2_history = squeeze(K_history(1, 2, :));

plot(1:length(K1_history), K1_history, 'r-o', 'LineWidth', 2);
hold on;
plot(1:length(K2_history), K2_history, 'b-s', 'LineWidth', 2);
grid on;
xlabel('迭代次数');
ylabel('控制器增益');
title('控制器参数演化');
legend('K(1,1)', 'K(1,2)');

% sigma_min演化
subplot(2, 3, 5);
sigma_mins = [iteration_results.sigma_min];
plot(1:length(sigma_mins), sigma_mins, 'c-d', 'LineWidth', 2);
grid on;
xlabel('迭代次数');
ylabel('\sigma_{min}');
title('\sigma_{min} 演化');

% 特征值分布
subplot(2, 3, 6);
if size(K_final, 2) == 2
    % 单位圆
    theta = linspace(0, 2*pi, 100);
    plot(cos(theta), sin(theta), 'k--', 'LineWidth', 1);
    
    hold on;
    eigs_open = eig(A);
    eigs_closed = eig(A - B * K_final);
    
    plot(real(eigs_open), imag(eigs_open), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    plot(real(eigs_closed), imag(eigs_closed), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    
    grid on;
    axis equal;
    xlabel('实部');
    ylabel('虚部');
    title('特征值分布');
    legend('单位圆', '开环', '闭环');
end

sgtitle('显式折扣方法实验结果', 'FontSize', 16);

% ========== 详细数据 ==========
fprintf('\n详细迭代数据:\n');
fprintf('迭代 | 折扣因子 | 成本 | 更新率 | K(1,1) | K(1,2)\n');
fprintf('----|---------|------|-------|-------|-------\n');

for i = 1:length(iteration_results)
    fprintf('%3d | %7.5f | %6.3f | %5.3f | %6.3f | %6.3f\n', ...
            iteration_results(i).outer_iter, ...
            iteration_results(i).gamma, ...
            iteration_results(i).cost, ...
            iteration_results(i).alpha, ...
            iteration_results(i).K(1,1), ...
            iteration_results(i).K(1,2));
end

% 保存结果
save('experiment_results.mat', 'K_final', 'gamma_history', 'cost_history', ...
     'iteration_results', 'A', 'B', 'elapsed_time');

fprintf('\n实验完成! 结果已保存到 experiment_results.mat\n'); 