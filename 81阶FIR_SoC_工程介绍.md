# 82 抽头（81 阶）FIR 滤波器 SoC 工程介绍（专家版）

> 面向专家讲解的结构化工程介绍，所有技术细节均与工程 RTL 源码一一对应，可随时回到代码举证。

---

## 一、开场一句话定位

> "这是一个基于 Cortex-M0 的轻量级 SoC，挂载了自研的 AXI DMA 和 82 抽头（81 阶）FIR 滤波器，目标是在紫光同创 MES50HP 开发板上实现一个能跑数字滤波任务的微型处理器系统。"

**展开**：CPU 是 ARM 官方 Cortex-M0（AHB 总线），通过 AHB→AXI Bridge（cortexm0ds）进入 AXI 世界；外设侧我们不是简单地把滤波器挂在 CPU 上轮询，而是引入了**自研 AXI 交叉开关 + 自研 DMA**，让 DMA 作为第二个总线主设备独立于 CPU 工作。系统跑在 FPGA 内部，用上电计数器产生复位，串口直连引脚，固件（C 语言）通过 UART/寄存器控制整条 DMA→FIR→DMA 数据链。

---

## 二、架构总图（核心）

```
Cortex-M0 (AHB)
   │
   ▼
AHB→AXI Bridge (cortexm0ds)
   │ AXI
   ▼
cmsdk_mcu_system 内部仲裁器（CMSDK 自带，含 SRAM/FLASH/APB 子系统）
   ├── SRAM  @ 0x2000_0000  (64KB, 链接器 RAM=0xFC00, 栈顶 0x2000_FC00)
   ├── FLASH @ 0x0000_0000  (64KB, 取指/启动)
   ├── APB 子系统: UART0 / GPIO / TIMER / WATCHDOG @ 0x4000_xxxx
   └── ext_* 端口 ──→ 自研 AXI Crossbar（外部扩展总线）
                          ├── S0: FIR 滤波器  @ 0x5000_0000
                          ├── S1: DMA 控制器  @ 0x5001_0000
                          └── S2: 回环 → CMSDK（SRAM/APB/FLASH 访问）
                                    ▲
                                    │ (extm_* 端口回到 CMSDK)
```

**强调**：CPU（M0）和 DMA（M1）是**两个 Master 挂在同一套 Crossbar** 上，Crossbar 用**单把锁**仲裁；DMA 也能通过 S2 回环口访问 SRAM 和 APB（比如 mem2uart 时直接读 SRAM、写 UART 寄存器）。这是典型的**主从双引擎架构**——CPU 管控制流，DMA 管数据流。

**地址解码（axi_crossbar.v 的 f_dec 函数）**：
| 地址窗口 | 从口 | 设备 |
|---|---|---|
| `0x5000_xxxx` | S0 | FIR（axi_fir_wrap） |
| `0x5001_xxxx` | S1 | DMA 寄存器 |
| 其余（0x2000/0x4000/0x0000…） | S2 | CMSDK 回环窗口（SRAM/APB/FLASH） |

---

## 三、三个自研模块（评分关键，专家必问）

### ① AXI Crossbar（rtl/axi_crossbar.v）

- **设计目标**：2 主（M0=CPU、M1=DMA）/ 3 从（S0 FIR、S1 DMA 寄存器、S2 CMSDK 回环），无死锁。
- **核心机制——单事务共享锁**：整个互连只有一把锁（代码中 `busy/busy_wr/owner/aw_done/ar_done/addr_q` 一组状态寄存器）。一旦某个 Master 的 **AW 或 AR 被授权，锁即被持有**，地址、大小、突发长度在授权瞬间被捕获并保持（`addr_q`），直到该事务的 **B 响应（写）或 R 响应（读）返回**才释放。这样保证：
  - 同一事务内 AW/W/B 与 AR/R 不会被另一 Master 插队重排；
  - 读写通道天然互斥，从协议层面排除死锁。
- **CPU 优先**：`PRIO_M0=1`（system_top.v 例化），只要 M0 有请求就获胜，配合 CMSDK 内部仲裁器（其对 B 通道是 PRIO_M1=1）**双向打破 A↔B 死锁**（代码注释原文）。
- **M0_CH_MASK = 3'b011**：把 S2（CMSDK 回环端口）从 M0 的可见从口里屏蔽掉——CPU 永远只访问 S0/S1，堵住组合环路（代码注释明确针对 STA-3001 类组合环）。
- **响应解复用**：B/R 只回给当前持锁的 Master（`grant_wr & b_any` / `grant_rd & r_any`），未获授权的 Master 收不到响应，不会误判事务完成。

> 一句话总结：**"用一把锁避免多 Master 死锁，CPU 永远优先，这是整个系统稳定性的基石。"**

### ② AXI DMA（rtl/axi_dma.v）

- **6 通道**（`NUM_CH = 6`），每个通道独立寄存器组：`n*0x100 + 0x00/04/08/0C/10/14` = CTRL / SRC / DST / LEN / CFG / STATUS，通道号取地址 `[11:8]`。
- **4 种传输类型**（CFG 寄存器 `[7:6]`）：
  - `00` mem2mem（SRAM↔SRAM）
  - `01` mem2fir（内存 → FIR DATA_IN）
  - `10` fir2mem（FIR DATA_OUT → 内存）
  - `11` mem2uart（内存 → UART0 DATA 寄存器）
- **四状态机**：`ST_IDLE → ST_RD（读源）→ ST_WR（写目的）→ ST_B（等 B 响应）→ 回 IDLE`；B 响应到达且计数归零才算一通道完成，置 IRQ/STATUS。
- **chunk 策略（代码注释原文）**：`beats = min(cfg_beats, 剩余字数, 源地址到 4KB 边界的余量, 目的地址到 4KB 边界的余量)`；`cfg_beats` 由 CFG[1:0] 选择 1/4/8/16 拍。中间用 **16 字（64 字节）暂存缓冲 bufq** 作读写之间的乒乓中转，`m_ar_len / m_aw_len = beats - 1`。
- **自适应协议降级**：判断两侧是否为"多口"（SRAM 窗口 / FIR 窗口走 INCR 多拍突发），APB/UART 等**单口强制单拍（beats=1）**——因为 AXI2APB 桥只支持 `AWLEN=0`（代码注释原文）。FIR 的 DATA_IN/DATA_OUT 是寄存器口，DMA 对它们用 **FIXED 流口**。
- **错误检测**：源/目的地址非 4 字节对齐或 LEN=0 → 置 `ch_err`，通道终止。

> 一句话总结：**"DMA 不是简单搬运，它能感知目标外设的协议限制，自动降级传输模式。"**

### ③ 82 抽头（81 阶）FIR 滤波器（custom/fir_81tap.v + rtl/axi_fir_wrap.v）

- **通用并行结构**：82 个抽头、82 个独立 16×16 乘法器全并行，**不假设系数对称**，任意非对称系数均正确（满足官方答疑 Q18）。
- **41 APM + 41 LUT 混合映射**：taps 0–40 的 41 个乘法器走 APM；taps 41–81 的 41 个乘法器用 `(* syn_multstyle = "logic" *)` 强制走 LUT。整片 PGL50H 仅 84 个 APM，该拆分使整芯片 APM 落在 **44/84**（41 FIR + 3 Cortex-M0）。
- **流水线**：
  - 乘积寄存器 `p1`（32bit）；
  - **7 级二叉加法树** L1–L7（33→39bit），82→41→21→11→6→3→2→1；
  - 41bit 累加 + Q1.15 舍入 + 饱和到 ±32767 / -32768；
  - 输入到输出固定 **9 拍**。
- **吞吐**：`in_ready` 恒为 1，**每周期收 1 个样本、输出 1 个样本**，无背压。
- **AXI 封装层（axi_fir_wrap.v）**：
  - 输入/输出各 **512 条目 FIFO**（`FIFO_AW=9`），输出 FIFO 接近满（≥496）时自动停喂核心（`out_afull`），防止溢出；
  - 寄存器：`0x00 CTRL[0]EN/[1]CLR`、`0x04 STATUS(BUSY/IN_FULL/OUT_EMPTY/OUT_OVF/IN_UDF/IN_CNT/OUT_CNT)`、`0x08 DATA_IN`、`0x0C DATA_OUT`、`0x10 IRQ_STAT`、`0x14 IRQ_EN`、`0x18 COEF_INDEX(0..81)`、`0x1C COEF_DATA`、`0x20 COEF_NUM=82`、`0x24/0x28` 总计数；
  - **运行时系数加载**：固件写 `COEF_INDEX` 再连续写 `COEF_DATA`，索引自动递增，逐拍写入 82 个系数；
  - `CTRL.CLR` 同时清空输入/输出 FIFO 与 FIR 历史寄存器（`x_sr`/流水线），切换系数后不会残留脏样本；
  - 输入/输出走寄存器口，所以 DMA 对 FIR 用 FIXED 流口传输。

> 一句话总结：**"41 APM + 41 LUT 的 82 抽头全并行 + 7 级加法树，在 50MHz 主频下做到 1 样本/拍，且支持任意非对称系数。"**

---

## 四、关键设计决策（体现思考深度）

| 问题 | 选择 | 原因（代码依据） |
|------|------|------|
| UART 引脚直连还是走 GPIO 复用？ | **直连** | 固件从不写 GPIO1 的 ALTFUNCSET，走 pin_mux 会让 TX 三态悬空、RX 悬空（system_top.v 注释原文：R9 floats） |
| 复位用按键还是上电计数器？ | **上电计数器** | `rst_cnt[19:0]` 上电从 0 自由计数，`sys_rst_n = rst_cnt[19]`，约 2^19 周期后自动释放；按键释放抖动会导致系统反复重启 |
| Crossbar 用轮询还是优先级？ | **CPU 优先 + 轮询兜底** | `PRIO_M0=1`：CPU 等响应会饿死 DMA，必须保证 CPU 优先；同时与内部仲裁器 B 通道的 PRIO_M1=1 配合，双向破 A↔B 死锁 |
| DMA 突发长还是短？ | **按目标自适应** | SRAM/FIR 走 INCR 多拍，APB/UART 强制单拍（AXI2APB 只支持 AWLEN=0） |
| 中断分配 IRQ15/16？ | **DMA→15，FIR→16** | 挂在 CMSDK 中断向量保留区（apbsubsys[15] 为保留位），与 APB 子系统中断按位 OR 汇入 NVIC（cmsdk_mcu_system.v:1328-1330） |
| FIR 系数固定还是可配？ | **运行时可配置，支持任意非对称** | COEF_INDEX/COEF_DATA 寄存器连续写 82 个系数，索引自动递增；不假设 h[k]=h[80-k]，满足答疑 Q18 |

---

## 五、当前进度和待完成

- **已完成**：顶层拼接（system_top.v）；UART 回显链路验证（RX 0x5A / TX 9600 输出 `\r\n`）；FIR 逐样本比对（[custom/tb_fir_81tap.v](custom/tb_fir_81tap.v) + `fir_golden.hex`）；Crossbar 单锁仲裁与 DMA 四状态机功能验证（模拟器 A~E 测试全 PASS）。
- **待完善**：
  1. DMA 长传输 / 多通道并发压力测试（chunk 突发逻辑已实现，尚未上板实测）；
  2. C 驱动封装（把 hello_soc.c 中的寄存器操作整理成 fir_/dma_ API）；
  3. 硬件联调（LED 指示、串口中断、真机 COM 口数据采集）。

---

## 六、一句话收尾

> "这个工程的核心价值不在于用了什么 IP，而在于**自研的 AXI Crossbar 和 DMA 让 DMA 能真正作为一个独立 Master 和 CPU 并行工作**——这是嵌入式系统里从'玩具'到'实用'的关键一步。"
