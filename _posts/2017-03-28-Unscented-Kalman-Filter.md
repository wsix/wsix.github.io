---
title: "【自动驾驶】- Unscented Kalman Filter"
categories:
  - technology
tags:
  - self-driving car
---

在自动驾驶中，对环境车辆速度和位置的判断主要依赖于非线性滤波器。传统的非线性滤波方法主要是**扩展的卡尔曼滤波器**（EKF），但是这种方法在使用中存在着精度不高，稳定性差，对目标机动反应差等缺点。在自动驾驶过程中，环境往往比较复杂，不时会有弯道等复杂路段，因此在应用中我们使用 UKF 代替 EKF 对车辆的速度和位置进行估计。

## Constant Turn Rate and Velocity Magnitude Model (CTRV)

相对于假设车辆的速度与方向恒定，在这里我们假设车辆能够保持恒定的角速度和速度在弯道行驶。这个假设下的模型也被称为 CTRV，如下图所示：

<div align="center">
    <img src="http://wsix.site/assets/images/UKF/CTRV.png" width = "500" alt="CTRV 模型图示" />
</div>

### 状态向量

对应于上图，CTRV 模型定义的状态向量为：

$$
x = \left[
        \begin{matrix}
            p_x \\
            p_y \\
            v \\
            \psi \\
            \dot \psi
        \end{matrix}
    \right]
$$

其中

* **$ \{p_x, p_y\} $** 表示二维平面下目标物体的横坐标和纵坐标；
* **$ \{v\} $** 表示目标物体的速度；
* **$ \{\psi\} $** 表示目标物体的偏航角 (yaw angle)；
* **$ \{\dot \psi\} $** 表示目标物体的角速度；

### 状态微分方程

上图中列出了 k 时刻车子的位置和 k+1 时刻车子的位置，这里假设状态变化模型为 $ \{x_{k+1} = f(x_k, \nu_k)\} $，此时的目的就是寻找出 $ \{x_k\} $ 与$ \{x_{k+1}\} $之间的递推关系 $ \{f\} $。根据 CTRV 的恒定角速度和速度的假设，我们可以得出状态方程中的各个变量变化的微分方程：

$$
\left[
    \begin{matrix}
        \dot{p_x} \\
        \dot{p_y} \\
        \dot{v} \\
        \dot{\psi} \\
        \ddot{\psi}
    \end{matrix}
\right] = \left[
    \begin{matrix}
        v \cdot \cos{(\psi)} \\
        v \cdot \sin{(\psi)} \\
        0 \\
        \dot{\psi} \\
        0
    \end{matrix}
\right]
$$

此时，如果我们获得了 $ \{t_k\} $ 和 $ \{t_{k+1}\} $的时间，我们就可以通过积分得到状态向量的递推关系：

$$
x_{k+1} = x_{k} + \int_{t_k}^{t_{k+1}}{
        \left[
            \begin{matrix}
                \dot{p_x}(t) \\
                \dot{p_y}(t) \\
                \dot{v}(t) \\
                \dot{\psi}(t) \\
                \ddot{\psi}(t)
            \end{matrix}
        \right]
    } dt = x_{k} + \left[
        \begin{matrix}
            \frac{v_k}{\dot{\psi_k}} \left( \sin{(\psi_k + \dot{\psi_k} \Delta t)} - \sin{(\psi_k)} \right) \\
            \frac{v_k}{\dot{\psi_k}} \left(-\cos{(\psi_k + \dot{\psi_k} \Delta t)} + \cos{(\psi_k)} \right) \\
            0 \\
            \dot{\psi} \Delta t \\
            0
        \end{matrix}
    \right]
$$

### Process Noise Vector

在之前的分析中，我们并没有考虑一些影响车辆状态变化的噪声，假设车辆的速度和角速度都是恒定的。这种情况在现实生活中并不常见，事实上，车辆的速度和角速度几乎每时每刻都会有细微的变化。

所以我们在这里引入了噪声 $ \{\nu_k\} $，它包含两个相互独立的变量，分别代表了影响速度的噪声（加速度）和影响角速度的噪声（角加速度）。

$$
\nu_k = \left[
        \begin{matrix}
            \nu_{a,k} \\
            \nu_{\ddot{\psi},k}
        \end{matrix}
    \right], \begin{matrix}
        \nu_{a,k} \sim N(0, \sigma_a^2) \\
        \nu_{\ddot{\psi},k} \sim N(0, \sigma_\ddot{\psi}^2)
    \end{matrix}
$$

接下来，我们便可以根据以上的噪声对之前得到的递推方程进行修改，这里假设噪声在任意两个时间段之间是恒定的，那么根据中学学过的加速度的知识，便可以得到如下的递推公式：

$$
x_{k+1} = x_{k} + \left[
        \begin{matrix}
            \frac{v_k}{\dot{\psi_k}} \left( \sin{(\psi_k + \dot{\psi_k} \Delta t)} - \sin{(\psi_k)} \right) \\
            \frac{v_k}{\dot{\psi_k}} \left(-\cos{(\psi_k + \dot{\psi_k} \Delta t)} + \cos{(\psi_k)} \right) \\
            0 \\
            \dot{\psi} \Delta t \\
            0
        \end{matrix}
    \right] + \left[
        \begin{matrix}
            \frac{1}{2} (\Delta t)^2 \cos{(\psi_k)} \cdot \nu_{a,k} \\
            \frac{1}{2} (\Delta t)^2 \sin{(\psi_k)} \cdot \nu_{a,k} \\
            \Delta t \cdot \nu_{a,k} \\
            \frac{1}{2} (\Delta t)^2 \nu_{\ddot{\psi}, k} \\
            \Delta t \nu_{\ddot{\psi}, k}
        \end{matrix}
    \right]
$$

以上就构建好了我们将要应用到 UKF 中的模型。

## Unscented Kalman Filter

对于状态变化的递推公式是线性的情况，此时状态向量的分布在预测前后仍然服从近似的正态分布，我们可以直接使用 Kalman Filter 来对目标物体的状态进行预测。

<div align="center">
    <img src="http://wsix.site/assets/images/UKF/linear.png" height="400" alt="Linear" />
</div>

但是对于状态变换的递推公式是非线性的时候，状态向量在经过预测之后得到的新的分布不再服从正态分布，甚至一般情况下我们很难计算得出此时的分布，如刚刚我们建立的 CTRV 模型，普通的 Kalman Filter 就无法解决了。此时我们就可以尝试使用 EKF 或者 UKF 解决这类问题。

### Basic Unscented Transformation

UKF 首先假设得到的分布仍然服从正态分布，通过采样一些点来尽可能地逼近新的分布。

<div align="center">
    <img src="http://wsix.site/assets/images/UKF/nonlinear.png" height="400" alt="nonlinear" />
</div>

其大致步骤如下：

1. 我们在原始的分布上采样一些点，使用这些点来表示原来的状态向量的分布（因为这个原因，所以这些点也被称为 Sigma Points）；
2. 将这些点代表的状态向量代入到非线性的递推公式中进行转换，从而得到经过转换之后的新的状态点；
3. 使用这些经过转换之后的 Sigma Points 估计新的分布（按照正态分布估计），求出它们的均值；

### 实施 UKF 的步骤

现在我们可以总览一下 UKF 的具体使用步骤，如下图所示：

<div align="center">
    <img src="http://wsix.site/assets/images/UKF/UKF_roadmap.png" height="400" alt="UKF_roadmap" />
</div>

下面是对这几个步骤的具体说明。

#### 选择 Sigma Points

Sigma Points 的数量取决于构建的模型的维度： $ \{n_\sigma = 2n_x + 1\} $ 。在 CTRV 模型中，我们的状态向量一共有 $ \{n_x = 5\} $ 个维度，所以此时 $ \{n_\sigma = 2 \times 5 + 1 = 11\} $。

接下来就是如何选出这 11 个 Sigma Points 了。在 Udacity 的课程上直接给出了计算的公式：

$$
X_{\sigma,k} = \left[
        X_{k|k} \quad
        X_{k|k} + \sqrt{(\lambda + nx)P_{k|k}} \quad
        X_{k|k} - \sqrt{(\lambda + nx)P_{k|k}}
    \right], \lambda=3-n_x
$$

注意

* **$ \{X_{k\|k}\} $**始终是 $ \{X_{\sigma,k}\} $的第一个元素；
* **$ \{X_{k\|k} + \sqrt{(\lambda + nx)P_{k\|k}}\} $**是 $ \{X_{\sigma,k}\} $的第2个到第$ \{n_k + 1\} $个元素；
* **$ \{X_{k\|k} - \sqrt{(\lambda + nx)P_{k\|k}}\} $**是 $ \{X_{\sigma,k}\} $的第$ \{n_k + 2\} $个到第$ \{2n_k + 1\} $个元素；

如此一来，我们好像就可以进行下一步对这些 Sigma Points 进行预测了，但是需要注意的是，我们的递推公式 $ \{x_{k+1} = f(x_k, \nu_k)\} $ 不仅仅与状态向量相关而且与噪声向量也相关，所以我们不能仅仅用状态向量计算 Sigma Points，还需要考虑噪声向量的影响。这一步并没有想象中的复杂，我们可以按照下图的方式对原来的状态向量进行扩充，同时更改相信的 P 矩阵的值，重新计算得到扩展状态向量的 Sigma Points。

<div align="center">
    <img src="http://wsix.site/assets/images/UKF/aug.png" height="400" alt="aug" />
</div>

#### 预测 Sigma Points

预测 Sigma Points 的过程比较简单，将之前的得到的扩展状态向量的对应部分带入到我们之前计算得出的递推公式就可以得到相应的结果了：


<div align="center">
    <img src="http://wsix.site/assets/images/UKF/predict.png" height="400" alt="predict" />
</div>


这里需要注意的是，预测得到的新的状态向量的维度应为原始状态向量的维度，同时还需要考虑 $ \{\dot{\psi}\} $ 为零的情况，此时：

$$
x_{k+1} = x_k + \left[
        \begin{matrix}
            v_k \cos{(\psi_k)} \Delta t \\
            v_k \sin{(\psi_k)} \Delta t \\
            0 \\
            \dot{\psi} \Delta t \\
            0
        \end{matrix}
    \right] + \left[
        \begin{matrix}
            \frac{1}{2} (\Delta t)^2 \cos{(\psi_k)} \cdot \nu_{a,k} \\
            \frac{1}{2} (\Delta t)^2 \sin{(\psi_k)} \cdot \nu_{a,k} \\
            \Delta t \cdot \nu_{a,k} \\
            \frac{1}{2} (\Delta t)^2 \nu_{\ddot{\psi}, k} \\
            \Delta t \nu_{\ddot{\psi}, k}
        \end{matrix}
    \right]
$$

#### 计算状态向量预测结果的平均值和协方差矩阵

在上一步中，我们已经计算得到了经过预测之后的 Sigma Points，接下来就是计算这些 Sigma Points 加权平均值作为此次的预测结果。

定义各个 Sigma Points 的权重，如下：

$$
\begin{split}
    \omega_i = \frac{\lambda}{\lambda + n_a}, i = 1 \\
    \omega_i = \frac{1}{2(\lambda + n_a)}, i = 1
\end{split}
$$

此时，可以按照以上权重计算平均值和协方差矩阵：

$$
x_{k+1|k} = \sum_{i=1}^{n_\sigma}\omega_i \mathcal{X}_{k+1|k,i}
$$

$$
P_{k+1|k} = \sum_{i=1}^{n_\sigma}\omega_i (\mathcal{X}_{k+1|k,i} - x_{k+1|k})(\mathcal{X}_{k+1|k,i} - x_{k+1|k})^T
$$

#### Predict Measurement

通过上一步得到的各个 Sigma Points，我们可以预测测量值的大小，在这一步要注意 Radar 和 Lidar 的数据的区别，接下来以雷达为例介绍这一过程。

对于每一个 Sigma Point，我们可以做如下变换

$$
Z_{k+1|k,i} = \left[
        \begin{matrix}
            \rho \\
            \phi \\
            \dot \rho \\
        \end{matrix}
    \right] = \left[
        \begin{matrix}
            \sqrt{p_{x,i}^2 - p_{y,i}^2} \\
            \arctan(\frac{p_{y,i}}{p_{x,i}}) \\
            \frac{p_{x,i}\cos{(\phi)}v + p_{y,i}\sin{(\phi)}v}{\sqrt{p_{x,i}^2 - p_{y,i}^2}}
        \end{matrix}
    \right]
$$

计算对应的加权平均数可得：

$$
z_{k+1|k} = \sum_{i=1}^{n_\sigma}\omega_i Z_{k+1|k,i}
$$

协方差矩阵可以通过如下公式计算：

$$
S_{k+1|k} = \sum_{i=1}^{n_\sigma}\omega_i (Z_{k+1|k,i} - z_{k+1|k})(Z_{k+1|k,i} - z_{k+1|k})^T + \left[
        \begin{matrix}
            \sigma_{\rho}^2 & 0 & 0 \\
            0 & \sigma_{\phi}^2 & 0 \\
            0 & 0 & \sigma_{\dot{\rho}}^2
        \end{matrix}
    \right]
$$

#### Update state

这一步与标准的 Kalman Filter 相差不大，通过之前得到的状态预测值和观测预测值与实际观测值的比较来更新系统的状态和协方差矩阵。

首先是计算观测预测值与状态预测值得互协方差矩阵：

$$
T_{k+1|k} = \sum_{i=1}^{n_\sigma}\omega_i (\mathcal{X}_{k+1|k,i} - x_{k+1|k})(Z_{k+1|k,i} - z_{k+1|k})^T
$$

接着，计算 Kalman 增益：

$$
K_{k+1|k} = T_{k+1|k} S_{k+1|k}^{-1}
$$

更新状态向量：

$$
x_{k+1|k+1} = x_{k+1|k} + K_{k+1|k} (z_{k+1} - z_{k+1|k})
$$

更新协方差矩阵：

$$
P_{k+1}{k+1} = P_{k+1}{k+1} - K_{k+1|k} S_{k+1|k} K_{k+1|k}^T
$$




声明：本页所涉及到图片和公式均来自于 Udacity.com。




