function [K_final, gamma_history, cost_history, iteration_results] = explicit_discount_method()
    % 显式折扣方法 - 学习稳定控制器
    
    fprintf('=== 显式折扣方法实现 ===\n');
    fprintf('目标: 在不知道系统模型的情况下学习稳定控制器\n\n');
    
    % ========== 1. 系统参数设置 ==========
    fprintf('1. 设置系统参数...\n');
    
    % 系统动力学矩阵
    A = [4, 3; 3, 1.5];
    B = [2; 2];
    
    % 验证系统不稳定性
    rho_A = max(abs(eig(A)));
    fprintf('   系统矩阵A的谱半径: %.4f (>1, 系统不稳定)\n', rho_A);
    
    % LQR成本矩阵
    Q = eye(2);
    R = 2;
    
    % 系统维度
    n = size(A, 1);
    m = size(B, 2);
    
    fprintf('   状态维度 n = %d, 输入维度 m = %d\n', n, m);
    fprintf('   A = [%.1f, %.1f; %.1f, %.1f]\n', A(1,1), A(1,2), A(2,1), A(2,2));
    fprintf('   B = [%.1f; %.1f]\n', B(1), B(2));
    fprintf('   Q = I_%d, R = %.1f\n\n', n, R);
    
    % ========== 2. 算法参数设置 ==========
    fprintf('2. 设置算法参数...\n');
    
    % 初始参数
    gamma_0 = 1e-3;              % 初始折扣因子
    xi = 0.9;                    % 安全系数
    K_0 = zeros(m, n);           % 初始控制器
    
    % 内循环参数
    M = 1;                       % 每次外循环的PG迭代次数
    eta = 1e-2;                  % 学习率
    r = 1e-3;                    % 扰动半径
    N_e = 200;                    % 梯度估计的rollout次数
    
    % 外循环参数
    N = 200;                      % 成本估计的rollout次数
    T = 150;                     % 模拟时域长度
    
    % 终止条件
    max_outer_iter = 200;         % 最大外循环次数
    
    fprintf('   初始折扣因子 gamma_0 = %.1e\n', gamma_0);
    fprintf('   安全系数 xi = %.2f\n', xi);
    fprintf('   内循环迭代次数 M = %d\n', M);
    fprintf('   学习率 eta = %.1e\n', eta);
    fprintf('   扰动半径 r = %.1e\n', r);
    fprintf('   梯度估计rollout次数 N_e = %d\n', N_e);
    fprintf('   成本估计rollout次数 N = %d\n', N);
    fprintf('   模拟时域 T = %d\n\n', T);
    
    % ========== 3. 初始化存储变量 ==========
    gamma_history = [];
    cost_history = [];
    iteration_results = [];
    
    % 当前变量
    gamma_k = gamma_0;
    K_k = K_0;
    
    fprintf('3. 开始算法主循环...\n\n');
    
    % ========== 4. 主算法循环 ==========
    for k = 1:max_outer_iter
        fprintf('=== 外循环第 %d 次迭代 ===\n', k);
        fprintf('当前折扣因子 gamma^%d = %.6f\n', k-1, gamma_k);
        
        % ========== 4.1 内循环: 策略优化 ==========
        fprintf('\n4.1 内循环: 策略优化...\n');
        
        K_j = K_k;  % 内循环初始策略
        
        for j = 1:M
            fprintf('  内循环第 %d 次迭代:\n', j);
            
            % 计算梯度估计
            grad_estimate = compute_gradient_estimate(A, B, Q, R, K_j, gamma_k, ...
                                                    r, N_e, T, n, m);
            
            % 策略梯度更新
            K_j = K_j - eta * grad_estimate;
            
            % 评估当前策略成本
            current_cost = estimate_cost(A, B, Q, R, K_j, gamma_k, N, T);
            fprintf('    更新后策略成本估计: %.4f\n', current_cost);
        end
        
        K_k_plus_1 = K_j;
        
        % ========== 4.2 外循环: 折扣因子更新 ==========
        fprintf('\n4.2 外循环: 折扣因子更新...\n');
        
        % 估计新策略的成本
        J_hat = estimate_cost(A, B, Q, R, K_k_plus_1, gamma_k, N, T);
        fprintf('  新策略成本估计 J_hat = %.6f\n', J_hat);
        
        % 计算更新率
        sigma_min_val = compute_sigma_min(Q, R, K_k_plus_1);
        fprintf('  sigma_min(Q + K^T*R*K) = %.6f\n', sigma_min_val);
        
        denominator = 2 * J_hat - sigma_min_val;
        if denominator <= 0
            fprintf('  警告: 分母 <= 0, 使用保守更新\n');
            alpha_k = 0.1;
        else
            alpha_k = sigma_min_val / denominator;
        end
        
        fprintf('  更新率 alpha^%d = %.6f\n', k-1, alpha_k);
        
        % 更新折扣因子
        gamma_k_plus_1 = (1 + xi * alpha_k) * gamma_k;
        fprintf('  新折扣因子 gamma^%d = %.6f\n', k, gamma_k_plus_1);
        
        % ========== 4.3 记录结果 ==========
        gamma_history = [gamma_history, gamma_k];
        cost_history = [cost_history, J_hat];
        
        iteration_results(k).outer_iter = k;
        iteration_results(k).gamma = gamma_k;
        iteration_results(k).K = K_k_plus_1;
        iteration_results(k).cost = J_hat;
        iteration_results(k).alpha = alpha_k;
        iteration_results(k).sigma_min = sigma_min_val;
        
        % ========== 4.4 检查终止条件 ==========
        if gamma_k_plus_1 >= 1.0
            fprintf('\n*** 算法成功终止! ***\n');
            fprintf('折扣因子达到 1: gamma^%d = %.6f >= 1\n', k, gamma_k_plus_1);
            fprintf('找到稳定控制器!\n');
            
            % 验证稳定性
            A_cl = A - B * K_k_plus_1;
            rho_cl = max(abs(eig(A_cl)));
            fprintf('闭环系统谱半径: %.6f (<1 表示稳定)\n', rho_cl);
            
            K_final = K_k_plus_1;
            gamma_history = [gamma_history, gamma_k_plus_1];
            return;
        end
        
        % 更新变量为下次迭代准备
        gamma_k = gamma_k_plus_1;
        K_k = K_k_plus_1;
        
        fprintf('\n');
    end
    
    % 如果达到最大迭代次数
    fprintf('达到最大迭代次数 %d, 算法停止\n', max_outer_iter);
    K_final = K_k;
end

function grad_estimate = compute_gradient_estimate(A, B, Q, R, K, gamma, r, N_e, T, n, m)
    % 计算策略梯度的零阶估计
    
    grad_sum = zeros(m, n);
    
    for i = 1:N_e
        % 生成随机扰动方向
        U_i = randn(m, n);
        U_i = U_i / norm(U_i, 'fro');
        
        % 生成扰动策略
        K_i1 = K + r * sqrt(m*n) * U_i;
        K_i2 = K - r * sqrt(m*n) * U_i;
        
        % 采样初始状态
        x0 = mvnrnd(zeros(n,1), eye(n))';
        
        % 计算两个扰动策略的有限时域成本
        V_T_1 = compute_finite_horizon_cost(A, B, Q, R, K_i1, gamma, x0, T);
        V_T_2 = compute_finite_horizon_cost(A, B, Q, R, K_i2, gamma, x0, T);
        
        % 计算梯度分量
        grad_comp = (V_T_1 - V_T_2) * U_i;
        grad_sum = grad_sum + grad_comp;
    end
    
    % 平均并归一化
    grad_estimate = grad_sum / (2 * r * sqrt(m*n) * N_e);
end

function V_T = compute_finite_horizon_cost(A, B, Q, R, K, gamma, x0, T)
    % 计算有限时域成本
    
    A_cl = A - B * K;
    
    x = x0;
    total_cost = 0;
    
    for t = 0:(T-1)
        % 计算控制输入
        u = -K * x;
        
        % 计算当前步的成本
        cost_t = x' * Q * x + u' * R * u;
        
        % 累加成本，乘以折扣因子
        total_cost = total_cost + (gamma^t) * cost_t;
        
        % 状态更新
        x = A_cl * x;
    end
    
    V_T = total_cost;
end

function J_hat = estimate_cost(A, B, Q, R, K, gamma, N, T)
    % 估计策略成本
    
    cost_sum = 0;
    
    for i = 1:N
        % 采样初始状态
        x0 = mvnrnd(zeros(size(A,1),1), eye(size(A,1)))';
        
        % 计算有限时域成本
        V_T = compute_finite_horizon_cost(A, B, Q, R, K, gamma, x0, T);
        cost_sum = cost_sum + V_T;
    end
    
    J_hat = cost_sum / N;
end

function sigma_min_val = compute_sigma_min(Q, R, K)
    % 计算 sigma_min(Q + K^T * R * K)
    
    matrix = Q + K' * R * K;
    eigenvals = eig(matrix);
    sigma_min_val = min(eigenvals);
    
    % 确保非负
    sigma_min_val = max(0, sigma_min_val);
end 