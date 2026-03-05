**行生成**

动机：

$$
min_{x,y} \quad c^Tx + f^Ty\\
\text{s.t. }
\begin{cases}
Ax+By=b\\
x\ge0\\
y\in \mathbb{Z}
\end{cases}
$$

直接求解困难，考虑分解，但是因为$y$是整数变量，在做对偶时操作不了，所以只能放在主问题。

将问题**按变量类型**拆分为$y$相关的MP和$x$相关的SP，当SP验证解不可行时再向MP加入Cut（用于指导下一次生成MP的解）。

optimality cut：用于加速计算
feasibility cut：用于让主问题求到真正可行解

MP & SP：

$$
\begin{array}{c|c}

\begin{aligned}
    \min_y \quad   & q(\bar y) + f^T y \\
    \text{s.t.} & y \in Y
\end{aligned}

&

\begin{aligned}
    \min_x \quad   & c^T x + f^T \bar{y} \\
    \text{s.t.} & Ax = b - B\bar{y} \\
                & x \ge 0
\end{aligned}

\end{array}
$$

$$
min_{y\in Y}\left[f^Ty+min\{c^Tx|Ax=b-By\}\right]
$$

* 这里每次主问题给y，子问题的可行域都会变化，不够理想$\Rightarrow$子问题做对偶把$b-By$移动到目标函数里，现在可行域与$y$无关

D-SP：

$$
max_{\lambda} \quad \lambda^T(b-B\bar{y}) + f^T\bar{y}\\
\text{s.t. }
\begin{cases}
\lambda A \le c^T\\
\lambda \quad \text{free}
\end{cases}
$$

---

### 子问题不同解对应主问题的情况

Benders分解中，MP给定$\bar y$后，D-SP有如下情况：

1. D-SP有界最优 $\Rightarrow$ SP有界最优 $\Rightarrow$ MP加最优性割
2. D-SP有无界解 $\Rightarrow$ SP无可行解 ⇒ MP加可行性割
3. D-SP无可行解 $\Rightarrow$ SP无可行解/无界解 ⇒ MP无效

只讨论前两种情况，假设D-SP可行域有极点$\lambda_i^p$，当可行域无界时另有极方向$\lambda_j^r$

1. **D-SP有界最优**

   此时，D-SP和SP最优值相同为$q(y)$，MP用该$q(y)$更新目标，D-SP可以写成如下形式：

   $$
   q(y) = min \quad q + f^T\bar{y} \\
   \text{s.t. }
   \begin{cases}
   \lambda^T(b-B\bar{y})\le q\\
   \lambda A \le c^T\\
   \lambda \quad \text{free}
   \end{cases}
   $$

   可以生成**optimality cut**使得结果不差于已有值（每次降低UB）：

   $$
   \lambda_i^p(b-B\bar{y})\le q, \quad i \in I
   $$

   对于任意y，D-SP可行域都固定，最优割加到主问题里相当于在可行域上施加一个约束。
2. **D-SP有无界解**

   此时，D-SP可以无限增大，一定有$b-B\bar{y}$和某个$\lambda_j^r$同方向，有：

   $$
   \lambda_j^r(b-B\bar{y}) \gt 0
   $$

   即当前的$\bar y$ 不合适， 需要向主问题添加**feasibility cut**（每次提高LB），排除该$y$及类似不可行情形：

   $$
   \lambda_j^r(b-B\bar{y}) \leq 0, \quad j\in J
   $$

加完割之后，主问题形如（每个$\bar y$只产生一个割）：

$$
\min_y \quad  q + f^T y \\
\text{s.t.}
\begin{cases}
\lambda_i^p(b-By)^T\le q & i\in I\\
\lambda_j^r(b-By)^T\le 0 & j\in J\\
y \in Y
\end{cases}
$$
