---
title: "舒尔补"
description: 《机器人学中的状态估计》系列
date: 2024-09-02T23:51:06+08:00
image: book.png
slug: 2_02
hidden: false
comments: true
categories:
- Math
tags:
- 机器人学中的状态估计
---
## 舒尔补定义
给定任意的矩阵块 $M$ ，如下所示：

$$M=\begin{bmatrix} A & B\\  C &D\\ \end{bmatrix}$$
- 如果，矩阵块 $D$ 是可逆的，则 $A − B D^{-1}  C$称之为 $D$ 关于 $M$的舒尔补。
- 如果，矩阵块 $A$ 是可逆的，则 $D − CA^{-1}  B$称之为 $A$ 关于 $M$的舒尔补。
 
## 舒尔补的定理推导
将$M$矩阵分别变成上三角或者下三角：
$$\begin{bmatrix}I & 0\\ -CA^{-1} &I\\\end{bmatrix}
\begin{bmatrix}A & B\\ C & D\end{bmatrix}=\begin{bmatrix}A & B\\ 0 & \Delta _{A}\end{bmatrix}$$

$$\begin{bmatrix}A & B\\ C&D\end{bmatrix}
\begin{bmatrix}I & -A^{-1}B\\  0&I\\\end{bmatrix}=
\begin{bmatrix}A & 0\\ C & \Delta _{A}\end{bmatrix}$$

其中${\Delta _{A} =D-CA^{-1}B }$。联合起来，将${M}$变形为对角形：
$$\begin{bmatrix}I & 0\\ -CA^{-1} &I\\\end{bmatrix}
\begin{bmatrix}A & B\\ C & D\end{bmatrix}
\begin{bmatrix}I & -A^{-1}B\\  0&I\\\end{bmatrix}=
\begin{bmatrix}A & 0\\ 0 & \Delta _{A}\end{bmatrix}
$$
反过来亦可从对角矩阵恢复${M}$:
$$\begin{bmatrix}I & 0\\ CA^{-1} &I\\\end{bmatrix}
\begin{bmatrix}A & 0\\ 0& \Delta _{A}\end{bmatrix}
\begin{bmatrix}I & A^{-1}B\\  0&I\\\end{bmatrix}=
\begin{bmatrix}A & B\\ C & D\ \end{bmatrix}
$$
##  用途
### 快速求矩阵的逆
矩阵${M}$可以改写为：
$$M=\begin{bmatrix}A & B\\ C&D\end{bmatrix}=\begin{bmatrix}I & 0\\ CA^{-1} &I\\\end{bmatrix}
\begin{bmatrix}A & 0\\ 0& \Delta _{A}\end{bmatrix}
\begin{bmatrix}I & A^{-1}B\\  0&I\\\end{bmatrix}$$
$$M^{-1}=\begin{bmatrix}A & B\\ C&D\end{bmatrix}^{-1}=\begin{bmatrix}I & 0\\ CA^{-1} &I\\\end{bmatrix}^{-1}\begin{bmatrix}A^{-1} & 0\\ 0& \Delta _{A}^{-1}\end{bmatrix}\begin{bmatrix}I & A^{-1}B\\  0&I\\\end{bmatrix}^{-1}\\  =\begin{bmatrix}I & 0\\ -CA^{-1} &I\\\end{bmatrix}\begin{bmatrix}A^{-1} & 0\\ 0& \Delta _{A}^{-1}\end{bmatrix}\begin{bmatrix}I & -A^{-1}B\\  0&I\\\end{bmatrix}\\=\begin{bmatrix}A^{-1}+A^{-1}B{\Delta _A}^{-1}CA^{-1}&-A^{-1}B{\Delta _A}^{-1}\\-{\Delta _A}^{-1}CA^{-1}&{\Delta _A}^{-1} \end{bmatrix}
$$
### 舒尔补在信息矩阵求解中的使用

协方差矩阵$$\sum=\begin{bmatrix}A & C^{T}\\C&D\end{bmatrix}$$,
则信息矩阵$$\sum^{-1}=\begin{bmatrix}A&C^{T}\\C&D\end{bmatrix}^{-1}\\=\begin{bmatrix}A^{-1}+A^{-1}C^{T}{\Delta _A}^{-1}CA^{-1}&-A^{-1}C^{T}{\Delta _A}^{-1}\\-{\Delta _A}^{-1}CA^{-1}&{\Delta _A}^{-1} \end{bmatrix}\\\stackrel{\triangle}{=}\begin{bmatrix}\Lambda_{aa}&\Lambda _{ab}\\\Lambda _{ba}&\Lambda _{bb} \end{bmatrix}$$
其中，由上式可推导得$A^{-1}=\Lambda _{aa}-\Lambda _{ab}\Lambda _{bb}^{-1}\Lambda _{ba}$, 以及$D^{-1}=\Lambda _{bb}-\Lambda _{ba}\Lambda _{aa}^{-1}\Lambda _{ab}$，它们即为下次优化会使用的先验信息矩阵（边际概率的信息矩阵）。
### 通过舒尔补分解多元高斯分布
 假设多元变量$M$服从高斯分布，且由两部分组成：$$ x=\begin{bmatrix}a\\b\end{bmatrix} $$,变量构成的协方差矩阵等于$$\sum=\begin{bmatrix} A & C^{T} \\C&D \end{bmatrix}$$,其中$A=cov(a,a)$,$C=cov(a,b)$,$D=cov(b,b)$。
则$x$的概率分布为：
$$P(a,b)=P(a)P(a|b)\propto exp(-\frac{1}{2} \begin{bmatrix}a\\b\end{bmatrix}^{T}\begin{bmatrix}A&C^{T}\\C&D\end{bmatrix}^{-1}\begin{bmatrix}a&b\end{bmatrix})$$。使用上一节内容将矩阵转化为对角矩阵
$$ \begin{align}
P(a,b) \\ 
\propto  exp\left (  -\frac{1}{2}\begin{bmatrix}a\\b\end{bmatrix}^{T}\begin{bmatrix}A&C^{T}\\C&D\end{bmatrix}^{-1}\begin{bmatrix}a&b\end{bmatrix}\right) \\
  \propto exp \left( -\frac{1}{2}\begin{bmatrix}a\\b\end{bmatrix}^{T}\begin{bmatrix}I & 0\\ -CA^{-1} &I\\\end{bmatrix}\begin{bmatrix}A^{-1} & 0\\ 0& \Delta _{A}^{-1}\end{bmatrix}\begin{bmatrix}I & -A^{-1}B\\  0&I\\\end{bmatrix}\begin{bmatrix}a&b\end{bmatrix})\right )\\
  \propto exp\left( -\frac{1}{2}\begin{bmatrix}a^{T}&(b-CA^{-1}a)^{T}\end{bmatrix}\begin{bmatrix}A^{-1}&0\\0&{\Delta _A^{-1}}\end{bmatrix}\begin{bmatrix}a\\b-CA^{-1}a\end{bmatrix}\right)\\
  \propto exp \left( -\frac{1}{2}(a^TA^{-1}a)+(b-CA^{-1}a)^{T} \Delta _A^{-1}(b-CA^{-1}a) \right)\\
  \propto exp \left( -\frac{1}{2}a^{T}A^{-1}a\right)exp \left ( -\frac{1}{2}(b-CA^{-1}a)^{T}\Delta _A^{-1}(b-CA^{-1}a)\right)\\
  \propto P(a)P(b|a)
  \\\end{align}$$
  在《机器人学中的状态估计》2.2.3章节"联合概率密度函数，分解与推断"可见相似内容,其实就是高斯推断，套用相关模型$P(a)$是观测（边际概率），$P(b|a)$是后验概率，$P(a|b)$是传感器模型。