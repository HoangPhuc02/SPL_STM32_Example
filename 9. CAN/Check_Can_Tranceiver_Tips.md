OK! Mình sẽ hướng dẫn bạn **mô phỏng Dominant State** **chỉ với 1 board duy nhất** để test điện áp **CANH ↑ / CANL ↓** (giống như đang truyền thực).

---

## Cách 1: **Chủ động kéo tay Dominant bằng điện trở**

(Chỉ dùng để test phần cứng Transceiver → Không cần data thật)

### Mục đích:

* **Force Bus Dominant** bằng cách **kéo CANL xuống GND** hoặc **kéo CANH lên VCC** qua điện trở.
* Quan sát xem **Transceiver phản ứng đúng** (CANH ↑, CANL ↓).

### Các bước:

1. **Gắn điện trở 120Ω** vào giữa **CANH ↔ CANL**.
2. Chuẩn bị **2 điện trở 1kΩ**.
3. Kéo Dominant bằng tay:

   * Cắm **1 điện trở 1kΩ từ CANL xuống GND** → Quan sát **CANL tụt ≈ 1.5V**, **CANH tăng ≈ 3.5V**.
   * Hoặc: Cắm **1kΩ từ CANH lên VCC (3.3V/5V)** → Quan sát tương tự.
4. Sau khi tháo ra → CANH & CANL phải về lại ≈ 2.5V.

### Kết quả mong đợi:

| Trạng thái                   | CANH   | CANL   |
| ---------------------------- | ------ | ------ |
| **Dominant (bạn kéo xuống)** | ≈ 3.5V | ≈ 1.5V |
| **Recessive (idle)**         | ≈ 2.5V | ≈ 2.5V |

---

## Cách 2: **Gửi Frame liên tục và đo bằng Oscilloscope/Voltmeter**

(Nếu phần mềm đã OK nhưng muốn nhìn tín hiệu thực sự)

### Các bước:

1. Dùng firmware gửi **CAN frame liên tục (10ms / lần)**.
2. Đảm bảo **kết nối có điện trở 120Ω ở 2 đầu bus (nếu chỉ có 1 transceiver thì 1 đầu thôi cũng được để mô phỏng tải)**.
3. Dùng Oscilloscope hoặc đồng hồ VOM đo:

   * **CANH phải "nhảy" từ 2.5V lên 3.5V lúc gửi data**.
   * **CANL phải tụt từ 2.5V xuống 1.5V lúc gửi data**.
4. Nếu đo vẫn luôn luôn 2.44V → **Transceiver không vào dominant (có thể do chưa có đối tượng nhận)**.

---

## Lưu ý Quan Trọng:

| Nếu chỉ có 1 transceiver         | Hành vi                                                                                                         |
| -------------------------------- | --------------------------------------------------------------------------------------------------------------- |
| **Không có thiết bị nhận (ACK)** | Sau khi gửi frame, CAN transceiver sẽ **release bus** ngay lập tức (về recessive 2.5V), vì **không có ai ACK**. |
| **Muốn giữ dominant state lâu**  | Bạn cần **có thiết bị thứ 2 đấu vào bus (có ACK bit phản hồi)** hoặc **force dominant bằng tay (Cách 1)**.      |

---

## Giải pháp Để Test Dominant State "Thật Sự":

* **Phải có 2 mạch CAN (hoặc 2 transceiver cùng trên 1 mạch) để giao tiếp nhau**.
* Khi đó, bạn sẽ thấy dominant state khi truyền (CANH ↑, CANL ↓).
* Hoặc đơn giản nhất: **Chế 1 con TJA1050 rời → Đấu vào cùng bus test chung**.

---

## Bạn có muốn mình **vẽ sơ đồ test force-dominant đơn giản chỉ với 1 transceiver** cho bạn dễ cắm jumper không?
