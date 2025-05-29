import numpy as np
import matplotlib.pyplot as plt
from sklearn.linear_model import Ridge, Lasso
from sklearn.datasets import make_regression

# Tạo dữ liệu giả: 100 mẫu, 10 đặc trưng, chỉ 3 đặc trưng là quan trọng
X, y, coef = make_regression(n_samples=100, n_features=10, 
                             n_informative=3, noise=5, coef=True, random_state=42)

# Huấn luyện Ridge (L2)
ridge = Ridge(alpha=1.0)
ridge.fit(X, y)

# Huấn luyện Lasso (L1)
lasso = Lasso(alpha=0.1)
lasso.fit(X, y)

# So sánh hệ số (trọng số theta_j)
plt.figure(figsize=(10, 5))
plt.plot(coef, 'o-', label='True coefficients', color='black')
plt.plot(ridge.coef_, 's--', label='Ridge coefficients (L2)', color='blue')
plt.plot(lasso.coef_, 'x--', label='Lasso coefficients (L1)', color='red')
plt.axhline(0, color='gray', linestyle='--')
plt.legend()
plt.title('So sánh trọng số giữa Ridge và Lasso')
plt.xlabel('Feature index')
plt.ylabel('Coefficient value')
plt.grid(True)
plt.tight_layout()
plt.show()
