import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from skimage import data, color, img_as_float
from skimage.metrics import peak_signal_noise_ratio as psnr
import cv2

# Helper function to add Gaussian noise
def add_noise(image, snr_db):
    noise = np.random.normal(0, 1, image.shape)
    noise_power = np.var(image) / (10 ** (snr_db / 10))
    noise *= np.sqrt(noise_power)
    return image + noise

# Helper function for Wiener Filter
def wiener_filter(observed_image, kernel, noise_variance):
    kernel_ft = np.fft.fft2(kernel, s=observed_image.shape)
    observed_image_ft = np.fft.fft2(observed_image)
    numerator = np.conj(kernel_ft) * observed_image_ft
    denominator = np.abs(kernel_ft) ** 2 + noise_variance
    restored_ft = numerator / denominator
    return np.abs(np.fft.ifft2(restored_ft))

# Helper function for Tikhonov Regularization
def tikhonov_regularization(A, g, mu):
    AtA = np.dot(A.T, A) + mu * np.eye(A.shape[1])
    Atg = np.dot(A.T, g)
    return np.linalg.solve(AtA, Atg)

# Helper function for Richardson-Lucy
def richardson_lucy(observed_image, kernel, iterations):
    estimate = observed_image.copy()
    for _ in range(iterations):
        estimate_ft = np.fft.fft2(estimate)
        kernel_ft = np.fft.fft2(kernel, s=observed_image.shape)
        ratio = np.fft.fft2(observed_image) / np.fft.fft2(signal.convolve2d(estimate, kernel, mode='same'))
        estimate = np.abs(np.fft.ifft2(estimate_ft * ratio))
    return estimate

# Load the original image
original = img_as_float(data.camera())

# Simulate the blurred image with Gaussian kernel and add noise
kernel = np.ones((5, 5)) / 25  # Simple box filter as blur kernel
blurred_image = signal.convolve2d(original, kernel, mode='same', boundary='wrap')
noisy_image = add_noise(blurred_image, snr_db=20)

# Apply Wiener filter
restored_wiener = wiener_filter(noisy_image, kernel, noise_variance=0.1)

# Apply Tikhonov regularization (using a simple identity matrix as A)
A = np.identity(original.size)
g = noisy_image.flatten()
restored_tikhonov = tikhonov_regularization(A, g, mu=0.01).reshape(noisy_image.shape)

# Apply Richardson-Lucy deconvolution
restored_richardson = richardson_lucy(noisy_image, kernel, iterations=10)

# Calculate PSNR
psnr_wiener = psnr(original, restored_wiener)
psnr_tikhonov = psnr(original, restored_tikhonov)
psnr_richardson = psnr(original, restored_richardson)

# Display the results
fig, axes = plt.subplots(2, 3, figsize=(12, 8))
axes[0, 0].imshow(original, cmap='gray')
axes[0, 0].set_title('Original')
axes[0, 1].imshow(noisy_image, cmap='gray')
axes[0, 1].set_title('Noisy Image')
axes[0, 2].imshow(restored_wiener, cmap='gray')
axes[0, 2].set_title(f'Wiener Filter (PSNR: {psnr_wiener:.2f} dB)')
axes[1, 0].imshow(restored_tikhonov, cmap='gray')
axes[1, 0].set_title(f'Tikhonov (PSNR: {psnr_tikhonov:.2f} dB)')
axes[1, 1].imshow(restored_richardson, cmap='gray')
axes[1, 1].set_title(f'Richardson-Lucy (PSNR: {psnr_richardson:.2f} dB)')

for ax in axes.flat:
    ax.axis('off')

plt.tight_layout()
plt.show()

# Report Results
print(f"PSNR for Wiener Filter: {psnr_wiener:.2f} dB")
print(f"PSNR for Tikhonov Regularization: {psnr_tikhonov:.2f} dB")
print(f"PSNR for Richardson-Lucy: {psnr_richardson:.2f} dB")
