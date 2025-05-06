# Swarm_Intelligence

Cấu trúc folder flock

Mô phỏng thuật toán flocking

Có 3 folder chính là code chứa các file code python, data chứa các dữ liệu chạy để vẽ hình và figure chứa các hình được vẽ ra trong các case

[Code](flock\code) có 5 file

* [ver2_bounded.py](flock\code\ver2_bounded.py) : File gốc chứa hành động flock đơn giản
* [dynamic](flock\code\dynamic_obs.py) : chứa file có vật cản có thể di chuyển được nhưng chỉ để xem hiện tượng
* [image](flock\code\image.py) : là file dynamic nhưng thêm các yếu tố về lặp lại file để thu dữ liệu cho vẽ hình báo cáo
* [plot ](flock\code\plot.py): vẽ biểu đồ error bar hoặc biểu đồ đường từ data đã thu

Data có 3 loại, 2 folder là khi có vật cản hoặc không, 2 file là đo vòng loop

Figure sẽ được thêm vào báo cáo
