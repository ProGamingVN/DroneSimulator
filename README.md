# Drone Simulator - Mô phỏng bay drone 3D thực tế

Một mô phỏng drone 3D thực tế được xây dựng bằng Three.js và vật lý Newtonian thực tế.

## Tính năng

- **Vật lý thực tế**: Mô hình lực và mô men quay chính xác dựa trên mô hình động lực học của quadcopter
- **Điều khiển thực tế**: 
  - W/S: Tiến/lùi (pitch)
  - A/D: Trái/phải (roll)
  - Space/Shift: Lên/xuống (thrust)
  - Q/E: Yaw trái/phải
  - Click chuột + di chuyển: Xoay view camera
- **Mô hình 3D chi tiết**: Động cơ, cánh quạt và корпуp hoàn chỉnh
- **Hiển thị thông tin**: Độ cao, vận tốc, góc nghiêng, tốc độ motore
- **Môi trường thực tế**: Trần đất xanh, nền trời xanh, ánh sáng đ realistic

## Cách chạy

1. Saope或 tải xuống repo này
2. Mở file `index.html` trong trình duyệt web hiện đại
3. Click vào màn hình để khóa con trỏ chuột (để điều khiển camera)
4. Sử dụng phím W/A/S/D/Space/Shift/Q/E để điều khiển drone
5. Nhấn 'R' để reset drone về vị trí ban đầu

## Nguyên lý vật lý

Mô phỏng sử dụng mô hình dynamique quadcopter chuẩn:

- **Lực levantar**: Đào động cơ tạo ra lực thẳng đứngיחס_exp vào bình phương của tốc độ quay (RPM²)
- **Mô men phản lực**: Động cơ tạo ra mô men quay ngược lại do phản lực Newton
- **Cấu trúc X**: 配置 của 4動機で、前後左右およびヨー制御が可能
- **Quán tính**: Mô hình tensors quán tính thực tế cho vai, pitch và yaw
- **Đón lực và mô men đọng**: Mô hình cản lực tuyến tính với tốc độ và góc tốc độ

## Thành phần

- `index.html`: Cấu trúc HTML và stylesheet
- `script.js`: Logic chính của mô phỏng (vật lý, render, điều khiển)
- Thư viện Three.js được tải từ CDN

## Các thông số kỹ thuật của drone (mô phỏng)

- Khối lượng: 0.8 kg
- Độ dài cánh: 0.22 m
- Hằng số тяга động cơ: 2.8e-6 N/(rad/s)²
- Hằng số mô men động cơ: 1.1e-7 Nm/(rad/s)²
- Moments of inertia: Ixx=0.0014, Iyy=0.0014, Izz=0.0023 kg·m²

## Phát triển tiếp theo

- Thêm pin mô phỏng thực tế
- Thêm hiệu ứng gió và hiện tượng rung
- Thêm mô hình điệu bay (waypoint navigation)
- Thêm hiệu ứng va chạm với vật cản
- Cải thiện mô hình khí động lực học