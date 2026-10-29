# MES50HP SoC 硬件测试工程
# 中国电子杯 - 带高阶 FIR 数字滤波器的 SoC 处理器设计

## 工程结构

```
soc_project/                      # 工程根目录（本仓库）
├── rtl/                          # Verilog 源文件
│   ├── system_top.v              # 顶层（连接所有模块 + 板级引脚）
│   ├── axi_crossbar.v            # 【自设计】2主3从AXI交叉开关
│   ├── axi_dma.v                 # 【自设计】6通道DMA控制器
│   └── axi_fir_wrap.v            # 【自设计】FIR的AXI接口封装
│
├── core/                         # Cortex-M0核 + AHB->AXI桥（已有）
│   └── cortexm0ds/
│       ├── cortexm0ds.v          # ARM Cortex-M0 DesignStart r1p0
│       ├── ahb2axi4_*.v          # AHB到AXI桥接（支持单拍/Burst）
│       └── ...
│
├── mcu/                          # MCU封装层（已有）
│   └── cmsdk_mcu.v               # 顶层MCU，含UART/GPIO/时钟
│
├── mcu_system/                   # 外设IP（已有）
│   ├── cmsdk_axi_memory/         # AXI SRAM/FLASH模型
│   ├── cmsdk_axi_gpio/           # AXI GPIO（LED 等通用 IO）
│   ├── cmsdk_axi_to_apb/         # AXI到APB桥（连接UART）
│   ├── cmsdk_apb_uart/           # APB UART（9600波特率）
│   └── cmsdk_mcu_system/         # 系统总线架构
│
├── custom/                       # FIR滤波器（已有）
│   ├── fir_81tap.v               # 82抽头(81阶)通用并行FIR，41 APM + 41 LUT，1 sample/cycle
│   ├── fir_coeffs.vh             # Q1.15系数（82个，支持任意非对称）
│   └── fir_stim.hex              # 测试激励（5404采样点）
│
├── pnr/
│   └── mes50hp_soc.fdc           # 引脚约束（50MHz时钟/UART0/LEDs）
│
├── generate_bitstream/
│   └── system_top.sbit / .smsk   # 可烧录位流
│
├── bitstream/
│   └── multiseed_summary.csv     # 4 个 PnR 种子的结果汇总
│
├── tools/
│   └── hex2words.py              # image.hex -> image_words.hex
│
└── software/                     # C驱动代码
    ├── hello_soc.c               # 自测 A–E：FIR/DMA 寄存器、DMA 搬运、FIR 直流响应、UART 回显
    ├── uart_stdout.c             # UART0 标准输出封装
    └── makefile                  # GCC编译脚本
```

---

## 内存地址映射

| 地址范围 | 外设 | 说明 |
|---------|------|------|
| 0x0000_0000 - 0x0000_FFFF | FLASH | Boot ROM（image.hex）|
| 0x2000_0000 - 0x2000_FFFF | SRAM | 64KB 指令+数据存储器 |
| 0x4000_4000 - 0x4000_40FF | UART0 | 9600bps，CPU stdout |
| 0x4001_0000 - 0x4001_0FFF | GPIO0 | LED 输出 |
| 0x4001_1000 - 0x4001_10FF | GPIO1 | 备用；UART0 直连引脚，固件不写 ALTFUNCSET |
| 0x5000_0000 - 0x5000_00FF | FIR Filter | 82抽头(81阶)，寄存器见 `rtl/axi_fir_wrap.v` 文件头 |
| 0x5001_0000 - 0x5001_06FF | DMA Controller | 6通道，每通道0x100字节；全局寄存器从 0x600 开始 |

---

## 开发板引脚映射（MES50HP / PGL50H-6IFBG484）

| 信号 | FPGA Pin | 说明 |
|------|----------|------|
| clk_50m | P20 | 板载50MHz晶振 |
| key1_n | K18 | 复位按键（低有效）|
| uart0_tx | R9 | UART0 TX → CP2102 RX |
| uart0_rx | R8 | UART0 RX ← CP2102 TX |
| led[0] | B2 | LED1 - 系统运行指示 |
| led[1] | A2 | LED2 - FIR活动指示 |
| led[2] | B3 | LED3 - DMA活动指示 |
| led[3-7] | A3/C5/A5/F7/F8 | LED4-8 - 备用 |

---

## 第一步：创建 PDS 工程

### 1.1 启动 Pango Design Suite
双击桌面快捷方式或从开始菜单启动

### 1.2 新建工程
1. 点击 **New Project**
2. **Project Name:** `soc_project`
3. **Project Location:** `D:\ICT\pds_workspace\soc_project`
4. 勾选 Create Project Subdirectory → Next

### 1.3 选择工程类型
- 选择 **RTL Project** → Next

### 1.4 添加源文件（依次添加以下文件）

**顶层文件：**
- `rtl\system_top.v`

**AXI互连模块：**
- `rtl\axi_crossbar.v`
- `rtl\axi_dma.v`
- `rtl\axi_fir_wrap.v`

**Cortex-M0核（按顺序）：**
- `core\cortexm0ds\ahb_axi_define.v`
- `core\cortexm0ds\ahb2axi4_burst.v`
- `core\cortexm0ds\ahb2axi4_fifo.v`
- `core\cortexm0ds\ahb2axi4_if.v`
- `core\cortexm0ds\ahb2axi4_ahb.v`
- `core\cortexm0ds\ahb2axi4_axi.v`
- `core\cortexm0ds\cortexm0ds_logic.v`
- `core\cortexm0ds\cortexm0ds.v`
- `core\models\cm0_dbg_reset_sync.v`
- `core\cortexm0_integration\cortexm0_systick.v`
- `core\cortexm0_integration\cortexm0_wic.v`
- `core\cortexm0_integration\cortexm0integration.v`
- `core\cortexm0_dap\cortexm0dap.v`

**MCU层：**
- `mcu\cmsdk_mcu_defs.v`
- `mcu\models\cmsdk_clock_gate.v`
- `mcu\cmsdk_mcu_pin_mux.v`
- `mcu\cmsdk_mcu_clkctrl.v`
- `mcu\cmsdk_mcu.v`

**MCU系统（按顺序）：**
- `mcu_system\cmsdk_axi_memory\cmsdk_axi_memory_defs.v`
- `mcu_system\cmsdk_axi_memory\cmsdk_axi_flash.v`
- `mcu_system\cmsdk_axi_memory\cmsdk_axi_sram.v`
- `mcu_system\cmsdk_axi_memory\cmsdk_axi_ram_beh.v`
- `mcu_system\cmsdk_axi_slave_mux\cmsdk_axi_addr_decode.v`
- `mcu_system\cmsdk_axi_slave_mux\cmsdk_axi_slave_mux.v`
- `mcu_system\cmsdk_axi_to_apb\ahb_axi_define.v`
- `mcu_system\cmsdk_axi_to_apb\axi2apb_if.v`
- `mcu_system\cmsdk_axi_to_apb\axi2apb_axi.v`
- `mcu_system\cmsdk_axi_to_apb\axi2apb_apb.v`
- `mcu_system\cmsdk_axi2apb_subsystem\cmsdk_irq_sync.v`
- `mcu_system\cmsdk_axi2apb_subsystem\cmsdk_apb_test_slave.v`
- `mcu_system\cmsdk_axi2apb_subsystem\cmsdk_axi2apb_subsystem.v`
- `mcu_system\cmsdk_apb_uart\cmsdk_apb_uart.v`
- `mcu_system\cmsdk_mcu_system\cmsdk_axi_default_slave.v`
- `mcu_system\cmsdk_mcu_system\cmsdk_axi_sysrom_table.v`
- `mcu_system\cmsdk_mcu_system\cmsdk_axi_sysctrl.v`
- `mcu_system\cmsdk_mcu_system\cmsdk_mcu_system.v`

**FIR滤波器：**
- `custom\fir_coeffs.vh`（include文件，会自动包含）
- `custom\fir_81tap.v`

→ Next

### 1.5 添加约束文件
- 点击 **Add Constraint Files**
- 选择 `pnr\mes50hp_soc.fdc`
- → Next

### 1.6 选择器件
- **Device Family:** Logos
- **Device Model:** PGL50H
- **Package:** IFBG484
- **Speed Grade:** -6
- → Next → Finish

---

## 第二步：编译与下载

### 2.1 编译流程
在 Navigator 面板中依次双击运行：

```
① Compile      ← 语法检查 + 编译
② Synthesize   ← 逻辑综合
③ Device Map   ← 器件映射（此时可以添加.pcf物理约束）
④ Place & Route ← 布局布线
⑤ Report Timing ← 时序报告（确认时序收敛）
⑥ Generate Bitstream ← 生成 .sbit / .smsk 位流文件
```

### 2.2 生成C固件（可选，用于CPU测试）
在 `software/` 目录下打开命令行：
```cmd
cd software
make
```
这会生成 `hello_soc.hex`（Verilog hex格式，可直接放入 FPGA FLASH）

### 2.3 下载验证
1. 用 USB 线连接开发板（JTAG接口）
2. 打开 PDS 的 **Program Flash** 工具
3. 选择生成的位流文件 `generate_bitstream/system_top.sbit`（掩码文件为同目录的 `.smsk`）
4. 点击下载
5. 观察 LED 和串口输出

---

## 第三步：验证测试

### 3.1 LED 验证
- **LED1（B2）**：唯一由硬件直接驱动的状态灯，50MHz 下约 1Hz 闪烁（25M 周期翻转）
- **LED2–LED8（A2/B3/A3/C5/A5/F7/F8）**：映射到 GPIO0[7:1]，当前固件不写 GPIO0，保持默认电平

### 3.2 UART 验证
1. 用 USB Type-C 线连接开发板（CP2102）
2. 打开设备管理器确认 COM 口
3. 打开串口助手（波特率 9600, 8N1, 无流控）
4. 预期输出（上电后自动跑 A–F 自测，最后进入回显循环）：
   ```
   ========================================
     MES50HP SoC - FIR + DMA Test
     Cortex-M0 @ 50MHz
   ========================================

   Test A: FIR COEF_NUM
   Test B: DMA VERSION
   Test C: DMA mem2mem
   Test D: FIR DC response
   Test F: FIR asymmetric 82-coef golden (1024 samples)

   ========================================
   Test Summary: N PASS, 0 FAIL
   ========================================
   *** ALL TESTS PASSED ***
   UART echo ready...
   ```
5. 发送任意字符，串口会回显 `echo: 0x?? '?'`

### 3.3 FIR 功能验证（比赛阶段）
```c
// 通过 CPU 配置 FIR
volatile uint32_t *fir = (volatile uint32_t *)0x50000000;
fir[0] = 0x00000001;  // CTRL: EN=1
// 写入输入数据
fir[2] = 0x00001234;  // DATA_IN
// 等待完成
while (!(fir[1] & 0x00000002));  // 等待 DONE
// 读取输出
uint16_t result = fir[3] & 0xFFFF;  // DATA_OUT
```

---

## 第四步：比赛正式开发

### 需要继续完善的模块

#### 4.1 AXI DMA 完善（axi_dma.v）
当前是简化版本，需要实现：
- 完整的 AXI Burst 传输（INCR模式）
- 6个通道的独立地址/长度配置
- 传输类型：mem→mem, mem→FIR, FIR→mem, mem→UART
- 链式传输支持（一个通道完成后自动触发下一个）

#### 4.2 AXI Crossbar 完善（axi_crossbar.v）
当前是简化版本，需要实现：
- 真正的仲裁逻辑（轮询/优先级）
- 各从设备的独立响应
- 支持 DMA 作为 Master 访问 SRAM/FIR/UART

#### 4.3 C 驱动程序（hello_soc.c 扩展）
需要编写：
```c
// FIR 配置驱动
void fir_configure(uint16_t *coeffs, int num_coeffs);
void fir_start(void);
int16_t fir_get_result(void);

// DMA 配置驱动
void dma_configure(int ch, uint32_t src, uint32_t dst, uint32_t len, int type);
void dma_start(int ch);
int dma_is_done(int ch);

// 系统初始化
void soc_init(void);
```

---

## 关键注意事项

1. **UART 连接方式**：UART0 直连物理引脚，不经过 GPIO1 的 alternate function
   - `uart0_tx` = R9，`uart0_rx` = R8
   - 固件从不写 GPIO1 的 ALTFUNCSET；若走 pin_mux，TX 会三态、R9 悬空

2. **FIR 系数**：82 抽头（81 阶），支持任意非对称系数
   - `fir_coeffs.vh` 中已定义 H0~H81（复位默认值，可为任意 82 个 Q1.15 系数）
   - 运行时写 `COEF_INDEX` 后连续写 `COEF_DATA`，索引自动递增；`CTRL.CLR` 清空 FIR 历史

3. **时钟约束**：50MHz 时钟，目标主频 ≥25MHz（满足要求）

4. **不能使用的开源代码**：比赛评分要求计分项不能使用开源代码
   - Cortex-M0 核：ARM 官方授权，可正常使用
   - CMSDK 外设：ARM 官方参考设计，可正常使用
   - **AXI Crossbar、DMA、FIR Wrapper 必须自己写**

5. **仿真与板级的区别**：
   - 两者都使用 UART0；`make sim` 把 BAUDDIV 设为 16 并置 CTRL[6] HSTM 位（1 bit/PCLK），用于加速仿真
   - 板级 `make` 使用 BAUDDIV=5208，50MHz 下为 9600bps
   - 约束文件中已正确映射到 R8/R9 引脚
