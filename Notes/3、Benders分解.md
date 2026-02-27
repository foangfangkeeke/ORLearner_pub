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

$$
\begin{array}{c|c}

\begin{aligned}
max \ & 15\pi_1+20pi_2\\
\text{s.t.}
\ & \begin{cases}
5\pi_1\le1\\
2\pi_2\le1\\
\pi_1,\pi_2\ge0
\end{cases}
\end{aligned}

\|

\begin{aligned}
max\ & 15\pi_1+20pi_2\\
\text{s.t.}
\ & \begin{cases}
1-5\pi_1\ge0\\
1-2\pi_2\ge0\\
\pi_1,\pi_2\ge0
\end{cases}
\end{aligned}

\end{array}
$$

$$
min_{y\in Y}\left[f^Ty+min\{c^Tx|Ax=b-By\}\right]
$$
