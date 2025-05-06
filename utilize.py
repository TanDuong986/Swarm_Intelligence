
def log_metrics_to_csv(boids, frame, file_name="metrics.csv"):
    order, entropy = compute_metrics(boids)
    time_normalized = frame / FPS  # Tính thời gian tính theo giây

    # Ghi vào file CSV
    with open(file_name, mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([time_normalized, order, entropy])