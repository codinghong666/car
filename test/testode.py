import torch
from torchdiffeq import odeint
import math
# 定义常微分方程（dy/dt = -y）
def func(t, y):
    return -y

# 初始条件
y0 = torch.tensor([1.0])  # y(0) = 1

# 时间点
t = torch.linspace(0., 10., 100)

# 求解ODE
solution = odeint(func, y0, t)

# 打印结果
print((solution-torch.exp(-t)).sum())
# print()
