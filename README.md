# Policy Gradient for Stabilizing Linear Systems: A MATLAB Implementation

This repository contains a MATLAB implementation of the **Explicit Discount Method** proposed in the paper:

**"Convergence and Sample Complexity of Policy Gradient Methods for Stabilizing Linear Systems"**  
by Feiran Zhao, Xingyun Fu, and Keyou You.  
*Published in: IEEE Transactions on Automatic Control, VOL. 70, NO. 3, MARCH 2025*

The primary goal of this algorithm is to learn a stabilizing feedback controller `K` for a linear time-invariant (LTI) system `x_{t+1} = Ax_t + Bu_t` **without prior knowledge of the system matrices `A` and `B`**, and without requiring an initial stabilizing policy.

---

## The Core Idea: The Explicit Discount Method

The challenge in stabilizing an unknown system is that standard policy gradient (PG) methods require a finite cost, which is only guaranteed if the system is already stable. This algorithm cleverly bypasses this "chicken-and-egg" problem using a **curriculum learning** approach.

> The core idea is to start with a very easy problem and gradually increase the difficulty until the original, hard problem is solved.

### How It Works

The method is built upon a dual-loop structure that iteratively refines the controller `K` and a discount factor `γ`.

1.  **Starting Simple with a Discounted Cost:**
    The algorithm begins by optimizing a **discounted LQR cost function**: `J_γ(K) = Σ γ^t * (x_t'Qx_t + u_t'Ru_t)`. By choosing a very small initial discount factor `γ_0`, the problem becomes easy enough that even a zero controller (`K=0`) yields a finite cost. This provides a safe and valid starting point for optimization.

2.  **Inner Loop: Policy Optimization:**
    For a fixed `γ`, the inner loop improves the current controller `K`. Since the model is unknown, it uses a **Zeroth-Order (or derivative-free) Policy Gradient** method:
    *   It randomly perturbs the current policy `K` in multiple directions.
    *   It collects cost samples by running simulations (rollouts) with these perturbed policies.
    *   It estimates the gradient `∇J_γ(K)` based on the difference in costs and updates the policy `K`.

3.  **Outer Loop: The Explicit Discount Factor Update:**
    This is the key innovation of the paper. After improving the policy `K` in the inner loop, the algorithm needs to increase `γ` to make the problem harder. Instead of using an inefficient search, it uses an **explicit update rule**:
    *   It estimates the cost `J_γ(K)` of the newly found policy.
    *   It calculates an update rate `α` based on this cost. The intuition is that a lower cost implies a more stable policy, which can tolerate a larger increase in `γ`.
    *   It updates the discount factor: `γ_{new} = (1 + ξα) * γ_{old}`.

4.  **Termination:**
    The algorithm repeats this process, gradually increasing `γ` towards 1. When `γ` finally reaches or exceeds 1, the algorithm terminates. The resulting controller `K_final` is guaranteed (with high probability) to stabilize the original, undiscounted system, i.e., `ρ(A - BK) < 1`.

---

## Simulation Setup

This implementation replicates the linear system example from the paper's simulation section to validate the algorithm's performance.

### System Dynamics

The code simulates a 2-D unstable linear system:
*   **State Matrix (A):**
    ```
    A = [4.0, 3.0; 
         3.0, 1.5]
    ```
    (This system is unstable, with `ρ(A) ≈ 5.5`)

*   **Input Matrix (B):**
    ```
    B = [2.0; 
         2.0]
    ```

### LQR Cost
*   **State Cost (Q):** `eye(2)`
*   **Input Cost (R):** `2`

### Algorithm Parameters
The key hyperparameters used in the `explicit_discount_method.m` script are:
*   `gamma_0`: `1e-3` (Initial discount factor)
*   `xi`: `0.9` (Safety factor for `γ` update)
*   `eta`: `1e-3` (or `0.01`, etc., depending on tuning) (Learning rate for policy gradient)
*   `M`: `1` (Number of inner-loop PG steps)
*   `N_e`: `20` (or `200`, etc.) (Number of rollouts for gradient estimation)
*   `N`: `20` (or `200`, etc.) (Number of rollouts for cost estimation)
*   `T`: `100` (Simulation horizon for each rollout)

---

## Results

After running the main script (`run_experiment.m`), the algorithm successfully learns a stabilizing controller.

### Convergence Plots

*(This figure shows the evolution of key metrics throughout the learning process, demonstrating successful convergence.)*

<!-- 
    INSERT YOUR 6-PANEL FIGURE HERE. 
    You can generate this by running the code and saving the figure.
-->
![Convergence Plots](path/to/your/results_figure.png)

### Final Performance & Data

<img width="1664" height="934" alt="b5c9b957b06de23c957de2e1def3125" src="https://github.com/user-attachments/assets/d28eec4b-61a6-49a4-84c6-95578e568968" />
<img width="381" height="186" alt="675c426f42c8836a87ed4ae04f54ead" src="https://github.com/user-attachments/assets/f61087e1-61f4-4ac0-9b74-5f97c04f46f0" />


```

---




