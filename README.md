# 显式折扣方法 - MATLAB实现

基于论文 "Learning to Control Linear Systems without Prior Knowledge" 实现的无模型控制器学习算法。

## 文件说明

- `explicit_discount_method.m` - 主算法实现
- `run_experiment.m` - 运行实验并查看结果

## 使用方法

直接运行：
```matlab
run_experiment
```

算法会自动运行并显示结果，包括：
- 学习到的控制器增益
- 系统稳定性验证
- 收敛过程可视化

## 算法简介

显式折扣方法通过逐步增加折扣因子来学习稳定的线性控制器：

1. 从一个小的折扣因子开始
2. 在每次迭代中优化折扣LQR问题
3. 根据成本函数显式更新折扣因子
4. 重复直到折扣因子接近1（原问题）

主要优势：
- 不需要系统模型
- 不需要初始稳定控制器
- 理论收敛保证

## 测试系统

默认使用2维不稳定系统：
```matlab
A = [4, 3; 3, 1.5]
B = [2; 2]
```

可以修改 `explicit_discount_method.m` 中的系统参数来测试其他系统。

## 依赖

- MATLAB (建议R2018b及以上)
- Statistics and Machine Learning Toolbox 