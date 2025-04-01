# import torch
# import torch.nn as nn
# from torch.utils.data import DataLoader, TensorDataset

# # 修改后的LNN模型
# class LNN(nn.Module):
#     def __init__(self, input_size, hidden_size, output_size, T, alpha, beta, eta, device, deltatime=0.01):
#         super(LNN, self).__init__()
#         self.device = device
#         self.input_to_hidden = nn.Linear(input_size, hidden_size).to(device)
#         self.hidden_to_output = nn.Linear(hidden_size, output_size).to(device)
#         self.W = nn.Parameter(torch.randn(hidden_size, hidden_size) * 0.1).to(device)
#         self.U = nn.Parameter(torch.randn(input_size, hidden_size) * 0.1).to(device)
#         self.b = nn.Parameter(torch.randn(hidden_size) * 0.1).to(device)
#         self.deltatime = deltatime
#         self.T = T
#         self.alpha = alpha    
#         self.beta = beta
#         self.eta = eta
#         self.hidden_size = hidden_size

#     def forward(self, x, train=True):
#         batch_size = x.shape[0]
#         hidden = torch.zeros(batch_size, self.hidden_size).to(self.device)

#         for t in range(self.T):
#             hidden = hidden + self.deltatime * (
#                 -self.alpha * hidden + 
#                 torch.tanh(torch.matmul(hidden, self.W) + torch.matmul(x, self.U) + self.b)
#             )

#             if train:
#                 # 使用矩阵运算更新W
#                 delta_W = (self.eta * torch.matmul(hidden.t(), hidden) / batch_size) - (self.beta * self.W)
#                 self.W.data += self.deltatime * delta_W

#         output = self.hidden_to_output(hidden)
#         return output

# device = torch.device('cpu')

# # 生成测试数据
# input_size = 4
# hidden_size = 16
# output_size = 1
# num_samples = 10000

# X = torch.randn(num_samples, input_size, device=device)  # 输入数据
# y = X[:, 0] ** 2 + torch.sin(X[:, 1])  # 非线性目标
# y = y.view(-1, output_size).float()
# X = X.float()

# # 创建数据集和数据加载器
# dataset = TensorDataset(X, y)
# train_size = int(0.8 * num_samples)
# test_size = num_samples - train_size
# train_dataset, test_dataset = torch.utils.data.random_split(dataset, [train_size, test_size])

# batch_size = 32
# train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True)
# test_loader = DataLoader(test_dataset, batch_size=batch_size, shuffle=False)

# # 初始化模型和优化器（排除W参数）
# model = LNN(input_size, hidden_size, output_size, T=5, alpha=0.1, beta=0.1, eta=0.1, device=device, deltatime=0.01)
# criterion = nn.MSELoss()
# params = [p for name, p in model.named_parameters() if name != 'W']
# optimizer = torch.optim.Adam(params, lr=0.005)

# # 训练循环
# num_epochs = 500
# for epoch in range(num_epochs):
#     model.train()
#     total_loss = 0
#     for batch_X, batch_y in train_loader:
#         batch_X, batch_y = batch_X.to(device), batch_y.to(device)  # Move data to the selected device
#         optimizer.zero_grad()
#         outputs = model(batch_X, train=True)  # 训练模式会更新W
#         loss = criterion(outputs, batch_y)
#         loss.backward()
#         optimizer.step()
#         total_loss += loss.item()
    
#     avg_loss = total_loss / len(train_loader)
#     print(f'Epoch [{epoch+1}/{num_epochs}], Loss: {avg_loss:.4f}')

# # 测试循环
# model.eval()
# total_test_loss = 0
# with torch.no_grad():
#     for batch_X, batch_y in test_loader:
#         batch_X, batch_y = batch_X.to(device), batch_y.to(device)  # Move data to the selected device
#         outputs = model(batch_X, train=False)  # 测试模式不更新W
#         loss = criterion(outputs, batch_y)
#         total_test_loss += loss.item()

# avg_test_loss = total_test_loss / len(test_loader)
# print(f'Test Loss: {avg_test_loss:.4f}')



# import torch
# import torch.nn as nn
# import torch.optim as optim
# from torch.utils.data import DataLoader, TensorDataset
# from torchdiffeq import odeint

# class LiquidBlock(nn.Module):
#     """液态时间常数网络层"""
#     def __init__(self, hidden_size, device):
#         super().__init__()
#         self.hidden_size = hidden_size
#         self.device = device
        
#         # 可学习参数c
#         self.W = nn.Parameter(torch.randn(hidden_size, hidden_size) * 0.1)
#         self.tau = nn.Parameter(torch.ones(hidden_size))  # 时间常数
#         self.input_gain = nn.Parameter(torch.ones(hidden_size))
        
#         # 初始化
#         nn.init.xavier_uniform_(self.W)
#         self.reset_parameters()
    
#     def reset_parameters(self):
#         nn.init.uniform_(self.tau, 0.5, 1.5)  # 初始化时间常数
    
#     def forward(self, t, h):
#         # 连续时间动力学方程: dh/dt = (-h + σ(Wh + I)) / τ
#         dhdt = (-h + torch.sigmoid(h @ self.W)) / self.tau.abs()  # τ取绝对值保证正数
#         return dhdt

# class LNN(nn.Module):
#     """液态神经网络"""
#     def __init__(self, input_size, hidden_size, output_size, device):
#         super().__init__()
#         self.device = device
#         self.hidden_size = hidden_size
        
#         # 网络结构
#         self.input_proj = nn.Linear(input_size, hidden_size)
#         self.liquid = LiquidBlock(hidden_size, device)
#         self.output_layer = nn.Linear(hidden_size, output_size)
        
#         # ODE求解参数
#         self.integration_time = torch.linspace(0, 1, 10).to(device)  # 1秒内采样10个点
    
#     def forward(self, x, train=True):
#         # 输入投影
#         h0 = torch.tanh(self.input_proj(x))
        
#         # 求解ODE (形状: [time_steps, batch_size, hidden_size])
#         h = odeint(
#             self.liquid, 
#             h0, 
#             self.integration_time,
#             method='dopri5',  # 自适应步长
#             rtol=1e-3,
#             atol=1e-4
#         )
        
#         # 取最后时间步的输出
#         return self.output_layer(h[-1])

# def generate_data(num_samples=5000, input_size=4):
#     """生成非线性时序数据"""
#     X = torch.randn(num_samples, input_size)
#     # 目标函数: 非线性组合
#     y = X[:, 0] * X[:, 1] + torch.sin(X[:, 2]) + 0.5 * X[:, 3]**2
#     return X.float(), y.view(-1, 1).float()

# def train_model():
#     # 配置参数
#     device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
#     input_size = 4
#     hidden_size = 32
#     output_size = 1
#     batch_size = 64
#     epochs = 20
    
#     # 生成数据
#     X, y = generate_data()
#     dataset = TensorDataset(X, y)
#     train_size = int(0.8 * len(dataset))
#     train_set, test_set = torch.utils.data.random_split(dataset, [train_size, len(dataset)-train_size])
#     train_loader = DataLoader(train_set, batch_size=batch_size, shuffle=True)
#     test_loader = DataLoader(test_set, batch_size=batch_size)
    
#     # 初始化模型
#     model = LNN(input_size, hidden_size, output_size, device).to(device)
#     optimizer = optim.Adam(model.parameters(), lr=0.005)
#     criterion = nn.MSELoss()
    
#     # 训练循环
#     for epoch in range(epochs):
#         model.train()
#         train_loss = 0
#         for x_batch, y_batch in train_loader:
#             x_batch, y_batch = x_batch.to(device), y_batch.to(device)
            
#             optimizer.zero_grad()
#             outputs = model(x_batch)
#             loss = criterion(outputs, y_batch)
#             loss.backward()
#             optimizer.step()
            
#             train_loss += loss.item()
        
#         # 验证
#         model.eval()
#         test_loss = 0
#         with torch.no_grad():
#             for x_batch, y_batch in test_loader:
#                 x_batch, y_batch = x_batch.to(device), y_batch.to(device)
#                 outputs = model(x_batch)
#                 test_loss += criterion(outputs, y_batch).item()
        
#         print(f'Epoch {epoch+1}/{epochs} | Train Loss: {train_loss/len(train_loader):.4f} | Test Loss: {test_loss/len(test_loader):.4f}')
    
#     # 动态响应测试
#     print("\nTesting dynamic response:")
#     test_input = torch.randn(1, input_size).to(device)
#     with torch.no_grad():
#         print(f"Input: {test_input.cpu().numpy()}")
#         print(f"Output: {model(test_input).item():.4f}")
#         print(f"Perturbed Output (+0.1): {model(test_input+0.1).item():.4f}")

# if __name__ == "__main__":
#     train_model()


import torch
import torch.nn as nn
import torch.optim as optim
from torchdiffeq import odeint

class PlasticityRule(nn.Module):
    """可塑性规则 (STDP-inspired)"""
    def __init__(self, hidden_size):
        super().__init__()
        # 可学习参数
        self.eta = nn.Parameter(torch.tensor(0.1))  # 学习率
        self.tau_pre = nn.Parameter(torch.tensor(20.0))  # 突触前时间常数
        self.tau_post = nn.Parameter(torch.tensor(20.0))  # 突触后时间常数
        
    def forward(self, pre, post, W):
        # STDP-like规则: ΔW = η * (post^T @ pre - pre^T @ post)
        stdp_update = self.eta * (torch.outer(post, pre) - torch.outer(pre, post))
        return W + stdp_update

class LiquidNeuron(nn.Module):
    """真正的液态神经元层"""
    def __init__(self, input_size, hidden_size, device):
        super().__init__()
        self.hidden_size = hidden_size
        self.device = device
        
        # 动态参数（始终更新）
        self.W = nn.Parameter(torch.randn(hidden_size, hidden_size) * 0.1)
        self.U = nn.Parameter(torch.randn(input_size, hidden_size) * 0.1)
        self.tau = nn.Parameter(torch.ones(hidden_size))  # 时间常数
        
        # 可塑性规则
        self.plasticity = PlasticityRule(hidden_size)
        
        # 活动追踪
        self.register_buffer('last_pre', torch.zeros(hidden_size))
        self.register_buffer('last_post', torch.zeros(hidden_size))

    def forward(self, t, h):
        # 连续时间动力学
        dhdt = (-h + torch.tanh(h @ self.W)) / self.tau.abs()
        
        # 更新可塑性（实时更新，无论训练/测试模式）
        self.last_post = h  # 记录当前活动
        self.W.data = self.plasticity(self.last_pre, self.last_post, self.W)
        self.last_pre = h   # 更新历史活动
        
        return dhdt

class LNN(nn.Module):
    """完整的液态神经网络"""
    def __init__(self, input_size, hidden_size, output_size, device):
        super().__init__()
        self.device = device
        self.hidden_size = hidden_size
        
        # 网络结构
        self.input_layer = nn.Linear(input_size, hidden_size)
        self.liquid = LiquidNeuron(input_size, hidden_size, device)
        self.output_layer = nn.Linear(hidden_size, output_size)
        
        # ODE求解时间点
        self.integration_time = torch.linspace(0, 1, 10).to(device)

    def forward(self, x):
        # 输入投影
        h0 = torch.tanh(self.input_layer(x))
        
        # 求解ODE（自动包含实时权重更新）
        h = odeint(
            self.liquid,
            h0,
            self.integration_time,
            method='dopri5',
            rtol=1e-3,
            atol=1e-4
        )
        
        return self.output_layer(h[-1])

# 测试动态权重更新
def test_dynamics():
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model = LNN(4, 16, 1, device).to(device)
    
    # 模拟连续输入
    x = torch.randn(1, 4).to(device)
    print("Initial W:", model.liquid.W.data[:2, :2])
    
    # 第一次前向传播
    y1 = model(x)
    print("After first pass W:", model.liquid.W.data[:2, :2])
    
    # 第二次前向传播（相同输入）
    y2 = model(x)
    print("After second pass W:", model.liquid.W.data[:2, :2])
    
    # 验证权重变化
    assert not torch.allclose(y1, y2), "Output should differ due to plastic weights!"

if __name__ == "__main__":
    test_dynamics()