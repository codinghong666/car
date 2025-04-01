# import torch
# import torch.nn as nn

# class LNN(nn.Module):
#     def __init__(self, input_size, hidden_size, output_size, T, alpha, beta, eta, deltatime=0.01):
#         super(LNN, self).__init__()
#         self.input_to_hidden = nn.Linear(input_size, hidden_size)
#         self.hidden_to_output = nn.Linear(hidden_size, output_size)
#         self.W= nn.Parameter(torch.randn(hidden_size, hidden_size)*0.1)
#         self.U= nn.Parameter(torch.randn(input_size, hidden_size)*0.1)
#         self.b= nn.Parameter(torch.randn(hidden_size)*0.1)
#         self.deltatime=deltatime
#         self.T=T
#         self.alpha=alpha    
#         self.beta=beta
#         self.eta=eta
#         self.hideen_size=hidden_size

#     def forward(self, x):
#         # x: (batch_size, input_size)
#         # hidden: (batch_size, hidden_size)
#         # output: (batch_size, output_size)
#         batch_size = x.shape[0]
#         hidden = torch.zeros(batch_size, self.W.size(0)).to(x.device)
#         last_hidden = torch.zeros(batch_size, self.W.size(0)).to(x.device)
#         output = torch.zeros(batch_size, self.hidden_to_output.out_features).to(x.device)

#         for t in range(self.T):
#             hidden = hidden+self.deltatime*(-self.alpha*hidden+torch.tanh(torch.matmul(hidden, self.W) + torch.matmul(x, self.U) + self.b))
#             for i in range(self.hideen_size):
#                 for j in range(self.hideen_size):
#                     self.W[i][j]=self.W[i][j]+self.deltatime*(self.eta*hidden[i]*hidden[j]-self.beta*self.W[i][j])

#         output = self.hidden_to_output(hidden)
#         return output


# import torch
# import torch.nn as nn
# import torch.optim as optim
# import numpy as np
# import matplotlib.pyplot as plt

# # Define the model
# class LNN(nn.Module):
#     def __init__(self, input_size, hidden_size, output_size, T, alpha, beta, eta, deltatime=0.01):
#         super(LNN, self).__init__()
#         self.input_to_hidden = nn.Linear(input_size, hidden_size)
#         self.hidden_to_output = nn.Linear(hidden_size, output_size)
#         self.W = nn.Parameter(torch.randn(hidden_size, hidden_size) * 0.1)
#         self.U = nn.Parameter(torch.randn(input_size, hidden_size) * 0.1)
#         self.b = nn.Parameter(torch.randn(hidden_size) * 0.1)
#         self.deltatime = deltatime
#         self.T = T
#         self.alpha = alpha    
#         self.beta = beta
#         self.eta = eta
#         self.hidden_size = hidden_size

#     def forward(self, x):
#         batch_size = x.shape[0]
#         hidden = torch.zeros(batch_size, self.W.size(0)).to(x.device)
#         output = torch.zeros(batch_size, self.hidden_to_output.out_features).to(x.device)

#         for t in range(self.T):
#             hidden = hidden + self.deltatime * (-self.alpha * hidden + torch.tanh(torch.matmul(hidden, self.W) + torch.matmul(x, self.U) + self.b))
#             for k in range(batch_size):
#                 for i in range(self.hidden_size):
#                     for j in range(self.hidden_size):
#                         self.W[i][j] = self.W[i][j] + self.deltatime * (self.eta * hidden[k][i] * hidden[k][j] - self.beta * self.W[i][j])

#         output = self.hidden_to_output(hidden)
#         return output

# # Generate sine wave data
# def generate_sine_wave_data(num_samples, input_size, freq=1.0, noise_factor=0.1):
#     t = np.linspace(0, 2 * np.pi * freq, num_samples)
#     sine_wave = np.sin(t)
    
#     # Add noise for realism
#     noise = noise_factor * np.random.randn(num_samples)
#     sine_wave += noise

#     # Create input and target data
#     X = sine_wave[:-1].reshape(-1, 1)  # Use previous sine value as input
#     y = sine_wave[1:].reshape(-1, 1)  # Next sine value as target
    
#     # Pad to match input_size
#     X = np.repeat(X, input_size, axis=1)  # Repeat the input to match the input size
#     return torch.tensor(X, dtype=torch.float32), torch.tensor(y, dtype=torch.float32)

# # Set hyperparameters
# input_size = 10
# hidden_size = 5
# output_size = 1  # Predict the next sine value
# batch_size = 16
# T = 10
# alpha = 0.1
# beta = 0.1
# eta = 0.1
# deltatime = 0.01
# num_samples = 1000

# # Generate sine wave data
# X, y = generate_sine_wave_data(num_samples, input_size)

# # Split the data into training and test sets
# train_size = int(0.8 * num_samples)
# X_train, X_test = X[:train_size], X[train_size:]
# y_train, y_test = y[:train_size], y[train_size:]

# # Instantiate the model
# model = LNN(input_size, hidden_size, output_size, T, alpha, beta, eta)

# # Loss function and optimizer
# criterion = nn.MSELoss()
# optimizer = optim.Adam(model.parameters(), lr=0.001)

# # Training function
# def train(model, X_train, y_train, criterion, optimizer, num_epochs=100):
#     for epoch in range(num_epochs):
#         model.train()  # Set model to training mode
        
#         optimizer.zero_grad()  # Zero the gradients
        
#         # Forward pass
#         output = model(X_train)
        
#         # Compute the loss
#         loss = criterion(output, y_train)
        
#         # Backward pass
#         loss.backward()
        
#         # Optimize the weights
#         optimizer.step()
        
#         if (epoch + 1) % 10 == 0:
#             print(f"Epoch [{epoch+1}/{num_epochs}], Loss: {loss.item():.4f}")

# # Train the model
# train(model, X_train, y_train, criterion, optimizer)

# # Testing function
# def test(model, X_test, y_test):
#     model.eval()  # Set model to evaluation mode
#     with torch.no_grad():
#         output = model(X_test)
#         loss = criterion(output, y_test)
#         print(f"Test Loss: {loss.item():.4f}")
#         return output

# # Test the model
# predicted = test(model, X_test, y_test)

# # Plotting the results
# plt.plot(y_test.numpy(), label='True sine wave')
# plt.plot(predicted.numpy(), label='Predicted sine wave', linestyle='dashed')
# plt.legend()
# plt.show()



import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset

# 修改后的LNN模型
class LNN(nn.Module):
    def __init__(self, input_size, hidden_size, output_size, T, alpha, beta, eta, device, deltatime=0.01):
        super(LNN, self).__init__()
        self.input_to_hidden = nn.Linear(input_size, hidden_size,device=device)
        self.hidden_to_output = nn.Linear(hidden_size, output_size,device=device)
        self.W = nn.Parameter(torch.randn(hidden_size, hidden_size) * 0.1,device=device)
        self.U = nn.Parameter(torch.randn(input_size, hidden_size)* 0.1)
        self.b = nn.Parameter(torch.randn(hidden_size) * 0.1)
        self.deltatime = deltatime
        self.T = T
        self.alpha = alpha    
        self.beta = beta
        self.eta = eta
        self.hidden_size = hidden_size

    def forward(self, x, train=True):
        batch_size = x.shape[0]
        hidden = torch.zeros(batch_size, self.hidden_size).to(x.device)

        for t in range(self.T):
            hidden = hidden + self.deltatime * (
                -self.alpha * hidden + 
                torch.tanh(torch.matmul(hidden, self.W) + torch.matmul(x, self.U) + self.b)
            )

            if train:
                # 使用矩阵运算更新W
                delta_W = (self.eta * torch.matmul(hidden.t(), hidden) / batch_size) - (self.beta * self.W)
                self.W.data += self.deltatime * delta_W

        output = self.hidden_to_output(hidden)
        return output

# 生成测试数据
input_size = 2
hidden_size = 16
output_size = 1
num_samples = 10000

X = torch.randn(num_samples, input_size,device='mps')  # 输入数据
y = X[:, 0] ** 2 + torch.sin(X[:, 1])  # 非线性目标
y = y.view(-1, output_size).float()
X = X.float()

# 创建数据集和数据加载器
dataset = TensorDataset(X, y)
train_size = int(0.8 * num_samples)
test_size = num_samples - train_size
train_dataset, test_dataset = torch.utils.data.random_split(dataset, [train_size, test_size])

batch_size = 32
train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True)
test_loader = DataLoader(test_dataset, batch_size=batch_size, shuffle=False)

# 初始化模型和优化器（排除W参数）
model = LNN(input_size, hidden_size, output_size, T=5, alpha=0.1, beta=0.1, eta=0.1, deltatime=0.01)
criterion = nn.MSELoss()
params = [p for name, p in model.named_parameters() if name != 'W']
optimizer = torch.optim.Adam(params, lr=0.005)

# 训练循环
num_epochs = 500
for epoch in range(num_epochs):
    model.train()
    total_loss = 0
    for batch_X, batch_y in train_loader:
        optimizer.zero_grad()
        outputs = model(batch_X, train=True)  # 训练模式会更新W
        loss = criterion(outputs, batch_y)
        loss.backward()
        optimizer.step()
        total_loss += loss.item()
    
    avg_loss = total_loss / len(train_loader)
    print(f'Epoch [{epoch+1}/{num_epochs}], Loss: {avg_loss:.4f}')

# 测试循环
model.eval()
total_test_loss = 0
with torch.no_grad():
    for batch_X, batch_y in test_loader:
        outputs = model(batch_X, train=False)  # 测试模式不更新W
        loss = criterion(outputs, batch_y)
        total_test_loss += loss.item()

avg_test_loss = total_test_loss / len(test_loader)
print(f'Test Loss: {avg_test_loss:.4f}')