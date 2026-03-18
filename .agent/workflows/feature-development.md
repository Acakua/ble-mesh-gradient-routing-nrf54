---
description: Quy trình thiết kế, lập trình (TDD) và đóng gói tính năng mới
---

### Bước 1: Phác thảo và Thiết kế (Design Phase)
1. **Kích hoạt kỹ năng**: `@brainstorming` và `@architecture`.
2. **Thảo luận**: Thống nhất logic của tính năng mới. **Tuyệt đối không viết code ở bước này.**
3. **Ghi lại**: Sau khi chốt phương án, sử dụng kỹ năng `@architecture-decision-records` để tạo một file ADR ghi lại quyết định thiết kế.
4. **Phê duyệt**: Chờ người dùng phê duyệt kế hoạch trước khi chuyển sang bước tiếp theo.

### Bước 2: Lập trình C và Kiểm thử hệ thống nhúng (TDD & Implementation Phase)
1. **Kích hoạt kỹ năng**: `@test-driven-development`, `@pypict-skill` và `@memory-safety-patterns`.
2. **Mô hình PICT**: Sử dụng `@pypict-skill` để phân tích và tạo test suite bao phủ các cấu hình gói tin mesh.
3. **Áp dụng TDD**: Sử dụng `@test-driven-development` để viết unit test bằng C trước khi viết logic thực tế.
4. **Lập trình**: Viết mã C firmware, tuân thủ nghiêm ngặt `@memory-safety-patterns`.
5. **Gỡ lỗi**: Nếu lỗi hoặc test thất bại, gọi `@debugger` để phân tích nguyên nhân gốc rễ và vá lỗi.
6. **Cập nhật Script**: Dùng `@bash-scripting` để cập nhật script biên dịch nếu có file mới.

### Bước 3: Phân tích Log và Huấn luyện AI (Data & ML Phase)
1. **Kích hoạt kỹ năng**: `@python-testing-patterns`, `@python-performance-optimization` và `@ai-ml`.
2. **Pipeline**: Cập nhật pipeline Python để parse dữ liệu log.
3. **Kiểm thử Python**: Bắt buộc sử dụng `@python-testing-patterns` viết Unit Test cho code xử lý dữ liệu.
4. **ML**: Cấu hình các đặc trưng và huấn luyện thuật toán LightGBM.
5. **Rà soát**: Dùng `@debugger` nếu script gặp lỗi hoặc dữ liệu sai lệch.

### Bước 4: Hoàn thiện (Wrap-up Phase)
1. **Kích hoạt kỹ năng**: `@create-pr`.
2. **Đóng gói**: Đọc git diff của toàn bộ dự án và tạo thông tin Pull Request chuyên nghiệp.