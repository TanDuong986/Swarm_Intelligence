import pandas as pd
import matplotlib.pyplot as plt

# Đọc file CSV
file_path = 'loop_trial_1.csv'  # Cập nhật đường dẫn tới file CSV của bạn
data = pd.read_csv(file_path)

# Chuyển đổi cột 'Time (s)' thành Time Step
time_step = data['Time (s)']

# Tính toán mean và variance cho Order và Entropy
order_mean = data['Order'].mean()
order_variance = data['Order'].var()

entropy_mean = data['Entropy'].mean()
entropy_variance = data['Entropy'].var()

# Vẽ biểu đồ Order
plt.figure(figsize=(10, 5))
plt.plot(time_step, data['Order'], label='Order', color='blue')
plt.axhline(y=order_mean, color='blue', linestyle='--', label=f'Order Mean ({order_mean:.5f})')
plt.axhline(y=order_mean + order_variance**0.5, color='blue', linestyle=':', label=f'Order Variance ({order_variance:.5f})')
plt.axhline(y=order_mean - order_variance**0.5, color='blue', linestyle=':', label=f'Order Variance ({order_variance:.5f})')

plt.xlabel('Bước thời gian (step)')
plt.ylabel('Order')
plt.title('Order của bầy đàn khi bay tuần hoàn có vật cản')
plt.grid(True)
plt.legend()
plt.savefig("oneTime_trial_1_order.png", dpi=300)
plt.show()

# Vẽ biểu đồ Entropy
plt.figure(figsize=(10, 5))
plt.plot(time_step, data['Entropy'], label='Entropy', color='red')
plt.axhline(y=entropy_mean, color='red', linestyle='--', label=f'Entropy Mean ({entropy_mean:.5f})')
plt.axhline(y=entropy_mean + entropy_variance**0.5, color='red', linestyle=':', label=f'Entropy Variance ({entropy_variance:.5f})')
plt.axhline(y=entropy_mean - entropy_variance**0.5, color='red', linestyle=':', label=f'Entropy Variance ({entropy_variance:.5f})')

plt.xlabel('Bước thời gian (step)')
plt.ylabel('Entropy')
plt.title('Entropy của bầy đàn khi bay tuần hoàn có vật cản')
plt.grid(True)
plt.legend()
plt.savefig("oneTime_trial_1_entropy.png", dpi=300)
plt.show()
