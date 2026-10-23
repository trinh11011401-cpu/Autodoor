# AutoDoor ver3 + CAN Driver theo AUTOSAR CP 4.4.0 (6 API trọng tâm)

## 6 API chính được ghép

| API | Vai trò | Liên hệ AUTOSAR SWS |
|---|---|---|
| `Can_Init` | Khởi tạo CAN0 trong STOPPED, 500 kbps, 3 HOH | 00223, 00245, 00259, 00246 |
| `Can_SetControllerMode` | STOPPED/STARTED | 00230, 00282, 00384 (**mới đáp ứng một phần**) |
| `Can_Write` | TX nonblocking HTH0/MB0 | 00233, 00212, 00213, 00214, 00275, 00276 |
| `Can_MainFunction_Write` | TX confirmation polling | 00225, 00031, 00016 |
| `Can_MainFunction_Read` | RX polling HRH MB4/5 | 00226, 00108, 00279 |
| `Can_DeInit` | STOPPED → UNINIT | 91002, 91009–91012 |

Các dịch vụ hỗ trợ `Can_MainFunction_Mode`, `Can_MainFunction_BusOff`, `Can_GetControllerMode` và **helper chẩn đoán không chuẩn** `Can_GetRawDiagnostics()` tồn tại để chạy trơn tru trên S32K144 và xử lý bus-off. Các API khác từ bundle 6 API ban đầu vẫn còn ở source nhưng **không thuộc phạm vi thuyết minh sáu API**.

## Những điểm ver3 được giữ nguyên

- **CAN 500 kbps** trên FlexCAN0, **PTE4 RX / PTE5 TX**, RX MB4 `0x100`, RX MB5 `0x102`, TX MB0 `0x101` và `0x103`.
- **MOTOR_NORMAL_PWM = 65%**, PWM khoảng **20 kHz**, L298N ENA PTD1, IN1 PTE0, IN2 PTE1.
- **DOOR_TRAVEL_COUNTS = 146**, giá trị **ước lượng ban đầu** từ số đo 293 xung ứng với khoảng 120°, chưa phải hiệu chuẩn chính thức; không dùng timer để khẳng định OPENED.
- Encoder x4 trên **PTB3/PTB2**, công tắc CLOSED **PTD13**.
- Heartbeat `0x102 A5` mỗi 200 ms; nếu quá **700 ms** và motor đang chạy sẽ cắt PWM và giữ fault.
- Obstacle/stall: đảo chiều có điều kiện, giữ deadtime 120 ms, fault được xóa khi thực sự về điểm cuối phù hợp.
- Sau bus-off, motor bị dừng và LOCK; hồi phục CAN phần cứng **không tự unlock**, không tự chạy motor. Yêu cầu heartbeat mới, CLEAR và UNLOCK.

## Luồng khởi tạo và vận hành

```text
Board_CanMcuClockInit → Board_CanPortInit
Can_Init(&Can_Config)  [STOPPED]
Can_SetControllerMode(CAN_CS_STARTED)
Can_MainFunction_Mode() → STARTED

Vòng lặp:
 App_CanSupervise()       [đọc ESR1/ECR qua helper, cắt motor khi bus-off]
 Can_MainFunction_BusOff()
 Can_MainFunction_Mode()
 App_CanRecover()         [STOPPED → DeInit → Init → STARTED]
 Can_MainFunction_Read()  [0x102 heartbeat trước, 0x100 command sau]
 Can_MainFunction_Write() [TX confirmation]
 App_CanTxService()       [Can_Write 0x101 hoặc 0x103]
```

Trong callback `CanIf_RxIndication` mới giải mã payload OPEN/CLOSE, CAN Driver không chứa logic cửa; `CanIf_TxConfirmation` nhận software PDU handle 1 (status) hoặc 2 (encoder).

## Đặc điểm khôi phục bus-off

- FlexCAN `BOFFREC` cấu hình theo **hướng hồi phục CAN có điều khiển bằng phần mềm**, không dựa vào hardware auto-recovery ngay khi bus còn lỗi.
- Phát hiện bus-off → cắt output motor → ghi snapshot các biến debug → `Can_MainFunction_BusOff` đưa controller về STOPPED và hủy TX pending.
- Sau ít nhất 1000 ms, nếu STOPPED, thử `Can_DeInit()`, init lại CAN0 và yêu cầu STARTED. Lỗi hoặc timeout chuyển mode giữ motor OFF và khóa fault; có thể cần reset board, không cố ép hủy khởi tạo khi controller vẫn STARTED.
- Sau khi STARTED lại, reset heartbeat count để không chấp nhận heartbeat cũ. Lệnh CLEAR chỉ được chấp nhận khi CAN đã sẵn sàng, không ở bus-off và có heartbeat mới. UNLOCK không hợp lệ khi fault bus-off còn latched.

## Kiểm thử bench theo thứ tự

1. **Rút nguồn motor 12V trước** (không chỉ gửi STOP bằng CAN). Cấp nguồn logic cho board, kiểm tra `can_last_status == E_OK`, `can_step == 11`, `dbg_core_hz` và trạng thái module/controller.
2. Phát `0x102 [A5]` chu kỳ 200ms: `dbg_hb_count` tăng. Thử mất heartbeat; trong lúc motor chạy sau này, motor phải STOP trong giới hạn timeout 700ms.
3. Test `0x100 [04 00]` UNLOCK, `0x100 [00 00]` STOP, `0x100 [05 00]` CLEAR, đọc `dbg_last_reject`; **chưa chạy OPEN/CLOSE khi ngắt nguồn motor**.
4. Xem 0x101 (DLC6) và 0x103 (DLC8), kiểm tra payload, `can_tx_count` và `dbg_status_tx_count`; đảm bảo có một node CAN khác ACK.
5. Kiểm tra limit PTD13, encoder A/B PTB3/PTB2 riêng; kiểm tra `dbg_limit_level`, `dbg_encoder_edges`, `dbg_encoder_count`, `dbg_open_target_count`.
6. Chỉ sau khi xác minh tín hiệu đúng, nối motor 12V; kiểm tra công tắc và emergency motor power cut; thử OPEN/CLOSE, đảo chiều khi gặp vật cản **trên mô hình không có người tiếp xúc**.
7. Mô phỏng bus-off khi **đã ngắt nguồn motor**; kiểm tra `dbg_can_busoff_count`, `dbg_can_reinit_attempts/success/failed`, `dbg_can_recovery_pending`, `dbg_locked`, `dbg_fault`.
8. Tách riêng bài test vòng đời `STOPPED → Can_DeInit → Can_Init → STARTED` khi chưa điều khiển cửa; không gọi `Can_DeInit` khi controller STARTED.



