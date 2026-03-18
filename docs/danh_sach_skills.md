# 📔 Ghi chú các Antigravity Skills đã cài đặt

Dưới đây là tóm tắt các kỹ năng (Skills) đã cài đặt để hỗ trợ phát triển dự án **BLE Mesh Gradient Routing**. Bạn có thể kích hoạt các kỹ năng này bằng cách sử dụng dấu `@` trong ô chat.

---

### 1. 🛠️ Lập trình C & Hệ thống nhúng
*   **`@memory-safety-patterns`**: Hướng dẫn các quy tắc an toàn bộ nhớ (RAII, sở hữu con trỏ) để tránh crash code C trên nRF54.
*   **`@binary-analysis-patterns`**: Hỗ trợ phân tích mã máy (disassembly) và gỡ lỗi sâu khi gặp các lỗi treo hệ thống hoặc lỗi thực thi khó tìm.

### 2. 🧠 Machine Learning & Python (LightGBM)
*   **`@python-pro`**: Áp dụng các kỹ thuật Python hiện đại cho các kịch bản huấn luyện mô hình và xử lý dữ liệu.
*   **`@python-performance-optimization`**: Tối ưu hóa hiệu năng code Python để xử lý tập dữ liệu log lớn nhanh hơn.
*   **`@ai-ml`**: Quy trình chuẩn để xây dựng và đánh giá các mô hình AI trực tiếp trong luồng làm việc.

### 3. 🏗️ Kiến trúc & Thiết kế Giao thức
*   **`@architecture`**: Phân tích các quyết định thiết kế cho giao thức Gradient Routing, đánh giá ưu/nhược điểm của các thay đổi lớn.
*   **`@architecture-decision-records` (ADRs)**: Giúp ghi lại nhật ký các quyết định kỹ thuật quan trọng để sau này dễ dàng tra cứu lý do tại sao một field hay logic lại được chọn.
*   **`@brainstorming`**: Công cụ để thảo luận và phác thảo các ý tưởng tính năng mới trước khi bắt tay vào viết code.

### 4. 🧪 Kiểm thử & Chất lượng (Testing & QA)
*   **`@test-driven-development`**: Quy trình phát triển hướng kiểm thử để viết code sạch và ít lỗi ngay từ đầu.
*   **`@python-testing-patterns`**: Chuyên dùng để viết unit test cho các script Python xử lý dữ liệu và huấn luyện mô hình.
*   **`@pypict-skill`**: Kỹ thuật kiểm thử tổ hợp (Pairwise Testing). Rất hữu ích khi cần kiểm tra nhiều sự hợp của các loại gói tin hoặc cấu hình mạng Mesh mà không tốn quá nhiều thời gian.
*   **`@debugger`**: Chuyên gia phân tích lỗi khi các bài test thất bại hoặc phần mềm hoạt động không đúng mong đợi.

### 5. 🚀 Quản lý dự án & Gỡ lỗi
*   **`@debugging-strategies`**: Phương pháp gỡ lỗi hệ thống để tìm ra nguyên nhân gốc rễ (root cause) của các lỗi mạng Mesh phức tạp.
*   **`@bash-scripting`**: Đảm bảo các script `.sh` dùng để flash và thu thập dữ liệu được viết chắc chắn, có xử lý lỗi tốt.
*   **`@create-pr`**: Hỗ trợ đóng gói các thay đổi code thành Pull Request hoàn chỉnh, chuyên nghiệp.

---

> [!TIP]
> **Cách dùng:** Bạn chỉ cần gõ nội dung kèm tên skill, ví dụ:
> *   *"@brainstorming hãy cùng tôi thiết kế một shell command mới để đo độ trễ mạng."*
> *   *"@memory-safety-patterns kiểm tra xem file gradient_srv.c này có nguy cơ rò rỉ bộ nhớ không."*
