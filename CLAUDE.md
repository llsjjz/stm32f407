# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

Bare-metal firmware for an STM32F407IGH6 (UFBGA176), driving DM-J4340P-2EC MIT-mode CAN bus servos. No RTOS — a superloop in `main.c`.

## Build & Flash

The ARM toolchain must be on `PATH`; it lives at `~/gnu_toolchain/xpack-arm-none-eabi-gcc-15.2.1-1.1/bin` (set by the environment; see `.vscode/c_cpp_properties.json`).

```bash
cmake --preset Debug          # configure (Ninja generator, build/Debug/)
cmake --build --preset Debug  # build -> build/Debug/stm32f407IGH6_26_9_20_0.{elf,hex,map}
cmake --build --preset Debug --target clean
```

Release exists too (`--preset Release`, `-Os -g0`). Debug is `-O0 -g3` and defines `DEBUG`.

Flash + verify + reset over ST-Link (OpenOCD):

```bash
openocd -f flash.cfg
```

`flash.cfg` hardcodes the absolute path to `build/Debug/stm32f407IGH6_26_9_20_0.elf`, so it only flashes the Debug build and breaks if the repo moves.

There are no tests, no linter, and no CI in this repo. `-Wall` plus `-fstack-usage` are on; `.su` files land next to the objects in `build/Debug/`.

## Layout & the generated/hand-written split

- `Core/` — STM32CubeMX-generated HAL init (`main.c`, `gpio.c`, `can.c`, `usart.c`, `stm32f4xx_it.c`, `stm32f4xx_hal_msp.c`).
- `func/` — hand-written application code. `func/Inc` + `func/Src` are the live code; `func/Inc/func_main.h` is the single entry point.
- `cmake/stm32cubemx/CMakeLists.txt` — CubeMX-generated CMake glue, regenerated on each CubeMX export.
- `func/func/` — **dead legacy code** (PID, UART parsing, servo). Not in any CMake `target_sources`, never compiled. It references `tim.h` and `huart6`, neither of which exists in this project. Don't treat it as reference for current conventions, and don't "fix" build errors in it.

### Keep CubeMX regeneration from eating your work

Generated sources protect user edits with `/* USER CODE BEGIN X */ ... /* USER CODE END X */` markers. Always put hand edits to `Core/` files inside those blocks, or the next `.ioc` regen silently discards them.

New application files must be registered in **two** places in `cmake/stm32cubemx/CMakeLists.txt`:
- includes under `MX_Include_Dirs`, marked `#user/include` (`func/Inc` is already there)
- sources under `MX_Application_Src`, marked `#user/src` (`func_main.c`, `func_can.c` already there)

Adding a `.c` file under `func/Src` without touching that list compiles nothing, silently — there is no glob. Regeneration also rewrites this file, so re-check the `#user/*` entries after any CubeMX export.

## Runtime flow

`main()` → `HAL_Init()` → `SystemClock_Config()` → `MX_GPIO_Init()` / `MX_CAN1_Init()` / `MX_USART1_UART_Init()` → `func_main_init()` (in `Core/Src/main.c` `USER CODE BEGIN 2`) → `while(1) func_main_run()`.

`func_main_init()` currently only calls `func_can_init()`. Put new peripheral/application init there rather than in generated init functions.

Clock: HSI → PLL (M=8, N=168, P=2) → 168 MHz SYSCLK, AHB /1, APB1 /4 (42 MHz), APB2 /2 (84 MHz). HSE/LSE pins are configured in the `.ioc` but `SystemClock_Config()` uses HSI — don't assume the crystal is running.

## CAN / motor protocol

This is the core of the project. CAN1 on PD0/PD1 at 1 Mbit/s (prescaler 3, BS1 9TQ, BS2 4TQ), normal mode, auto-retransmit + auto-bus-off.

### Command ID vs feedback ID — two separate registers

The datasheet configures each motor with two independent IDs, and conflating them is the easiest way to misread this code:

- **`ESC_ID`** (register `0x08`, "receive ID") — the ID a motor accepts *commands* on. Unique per motor: `0x201`..`0x204`, encoded by `CAN_MOTOR_ID_t` (`0x200 + n`). Commands are addressed per motor on this ID.
- **`MST_ID`** (register `0x07`, "feedback ID") — the ID a motor *reports on*. All four motors are set to the same value, `0x200`, so the STM32 sees every motor's feedback on one ID.
- The feedback frame's `D[0]` disambiguates the source without needing a per-motor feedback ID: its **low nibble is the sender's `ESC_ID` low byte** (`1`..`4`) and its high nibble is `ERR`. That is why a single-ID filter still works.
- The header defines one macro, `CAN_MASTER_ID` (`0x200`), which currently fills both roles. They are distinct registers and need not hold the same value.

### Prerequisites on the motor side

Set per motor with the 达妙 debug assistant, before any of this works: `ESC_ID` = `0x201`..`0x204` (unique), `MST_ID` = `0x200` (**all four** — the factory default is `0`, a frequent cause of "code looks right but no feedback arrives"), and `CTRL_MODE` = `1` (MIT). `PMAX`/`VMAX`/`TMAX` must match the bounds in `func/Inc/func_can.h`; torque is the controlled quantity, so a `TMAX` mismatch scales `t_ff` proportionally and silently breaks torque calibration.

### Frame details

- Fixed-point packing lives in `func/Src/func_can.c`: `float_to_uint` / `uint_to_float` map physical values to raw bit fields over `[-Pmax,Pmax]`, `[-Vmax,Vmax]`, `[-Tmax,Tmax]`. Out-of-range floats wrap/truncate without clamping — clamp at the call site.
- MIT command frame layout is in `func_can_transmit_MIT`: 16-bit position, 12-bit velocity, 12-bit torque+16-bit `Kp`, 16-bit `Kd`, bit-packed big-endian across 8 bytes. Specific bits, not byte-aligned.
- Special commands are the `func_can_transmit_Enable/DisEnable/SetZero/ClearErr` helpers — all-`0xFF` payload with a distinct trailing command byte.
- Feedback is **request/response, not free-running**: a motor transmits its status frame only after receiving a command frame that matches its `ESC_ID` (datasheet §CAN通信). No command sent → no feedback, and the motor's `TIMEOUT` protection (register `0x09`) drops it out of enabled state. Round-robin all four `ESC_ID`s, not just motor 1.
- **Receive path is interrupt-driven**: RX FIFO0 pending → `HAL_CAN_RxFifo0MsgPendingCallback()` (bottom of `func_can.c`). It derives the motor index from the `D[0]` low nibble (the sender's `ESC_ID` low byte), fills `motor_info[index]`, and populates the derived `*_f` float fields. `motor_info[4]` is a global (`extern` in `func_can.h`) mutated from ISR context — snapshot it in the main loop rather than reading fields one at a time if coherence matters.
- Filter bank 0 is deliberately a 32-bit ID-mask accepting `0x200` exactly. That is correct: every motor reports to the shared `MST_ID`, so nothing else needs to pass. Do not widen it to `0x201`..`0x204` — that silently drops all feedback.

Adding a motor command means touching the enum-free index math (`id - CAN_MASTER_ID - 1`), the fixed-point conversion, and the header's bounds together.

## Conventions

- Comments are in Chinese, often with a block explaining the *reasoning*/*math* (see the conversion functions and the legacy `pid.h` header block). Match that style when adding non-obvious code.
- Public API in `func/Inc/*.h`, implementation in `func/Src/*.c`; headers include `main.h` then the generated peripheral headers they need.
- No `printf` over UART wiring beyond `syscalls.c` stubs; USART1 (PA9/PB7, 9600 8N1) is initialized but unused by application code.

## Reference material

`Datum/DM-J4340P-2EC V1.1 减速电机使用说明书V1.4 2026-09-14.pdf` is the motor datasheet — the authority for frame layout, Pmax/Vmax/Tmax ranges, and error codes.

---

# 中文翻译

> 以下为上方英文内容的完整中文对照，小节一一对应。标识符、文件名、命令与代码块保持原样，便于对照查找。

## 项目

面向 STM32F407IGH6（UFBGA176）的裸机固件，用于驱动 DM-J4340P-2EC MIT 模式 CAN 总线伺服电机。无 RTOS —— `main.c` 中是一个超级循环（superloop）。

## 构建与烧录

ARM 工具链必须在 `PATH` 中；它位于 `~/gnu_toolchain/xpack-arm-none-eabi-gcc-15.2.1-1.1/bin`（由环境配置，见 `.vscode/c_cpp_properties.json`）。

```bash
cmake --preset Debug          # 配置（Ninja 生成器，输出到 build/Debug/）
cmake --build --preset Debug  # 构建 -> build/Debug/stm32f407IGH6_26_9_20_0.{elf,hex,map}
cmake --build --preset Debug --target clean
```

也存在 Release 配置（`--preset Release`，`-Os -g0`）。Debug 为 `-O0 -g3` 并定义 `DEBUG`。

通过 ST-Link 烧录 + 校验 + 复位（OpenOCD）：

```bash
openocd -f flash.cfg
```

`flash.cfg` 硬编码了 `build/Debug/stm32f407IGH6_26_9_20_0.elf` 的绝对路径，因此只能烧录 Debug 构建，且仓库目录移动后会失效。

本仓库没有测试、没有 linter、没有 CI。启用了 `-Wall` 和 `-fstack-usage`；`.su` 文件生成在 `build/Debug/` 中对应目标文件旁边。

## 目录结构与"生成代码/手写代码"的划分

- `Core/` —— STM32CubeMX 生成的 HAL 初始化代码（`main.c`、`gpio.c`、`can.c`、`usart.c`、`stm32f4xx_it.c`、`stm32f4xx_hal_msp.c`）。
- `func/` —— 手写应用代码。`func/Inc` + `func/Src` 是当前有效代码；`func/Inc/func_main.h` 是唯一入口。
- `cmake/stm32cubemx/CMakeLists.txt` —— CubeMX 生成的 CMake 胶水文件，每次 CubeMX 导出都会重新生成。
- `func/func/` —— **已废弃的遗留代码**（PID、UART 解析、舵机）。不在任何 CMake `target_sources` 中，从不参与编译。它引用了 `tim.h` 和 `huart6`，二者在本项目中都不存在。不要把它的写法当作当前约定的参考，也不要去"修复"其中的编译错误。

### 防止 CubeMX 重新生成时覆盖你的修改

生成的源文件用 `/* USER CODE BEGIN X */ ... /* USER CODE END X */` 标记保护用户修改。对 `Core/` 文件的手工修改务必放在这些块内，否则下一次 `.ioc` 重新生成会静默丢弃它们。

新增的应用文件必须在 `cmake/stm32cubemx/CMakeLists.txt` 中的**两处**登记：
- 头文件路径加在 `MX_Include_Dirs` 下，标有 `#user/include`（`func/Inc` 已在此处）
- 源文件加在 `MX_Application_Src` 下，标有 `#user/src`（`func_main.c`、`func_can.c` 已在此处）

在 `func/Src` 下新增 `.c` 文件而不改这份列表，会静默地什么都不编译 —— 这里没有 glob 通配。重新生成也会重写该文件，所以每次 CubeMX 导出后都要复查 `#user/*` 条目。

## 运行流程

`main()` → `HAL_Init()` → `SystemClock_Config()` → `MX_GPIO_Init()` / `MX_CAN1_Init()` / `MX_USART1_UART_Init()` → `func_main_init()`（位于 `Core/Src/main.c` 的 `USER CODE BEGIN 2`）→ `while(1) func_main_run()`。

`func_main_init()` 目前只调用 `func_can_init()`。新增的外设/应用初始化请放在这里，而不是放进生成的 init 函数中。

时钟：HSI → PLL（M=8、N=168、P=2）→ 168 MHz SYSCLK，AHB /1，APB1 /4（42 MHz），APB2 /2（84 MHz）。`.ioc` 中配置了 HSE/LSE 引脚，但 `SystemClock_Config()` 使用的是 HSI —— 不要假定晶振已在运行。

## CAN / 电机协议

这是本项目的核心。CAN1 使用 PD0/PD1，速率 1 Mbit/s（预分频 3，BS1 9TQ，BS2 4TQ），正常模式，自动重传 + 自动离线恢复。

### 控制帧 ID 与反馈帧 ID —— 是两个独立寄存器

手册给每台电机配置两个互不相关的 ID，把二者混为一谈是误读这套代码的最快途径：

- **`ESC_ID`**（寄存器 `0x08`，"接收 ID"）—— 电机**接收控制命令**所用的 ID。逐台不同：`0x201`..`0x204`，由 `CAN_MOTOR_ID_t`（`0x200 + n`）编码。控制帧按此 ID 逐台寻址。
- **`MST_ID`**（寄存器 `0x07`，"反馈 ID"）—— 电机**上报状态**所用的 ID。四台全部设为同一值 `0x200`，因此 STM32 在一个 ID 上就能收到全部四台电机的反馈。
- 反馈帧的 `D[0]` 无需逐台反馈 ID 即可区分来源：其**低 4 位是发送方的 `ESC_ID` 低 8 位**（`1`..`4`），高 4 位是 `ERR`。这正是单一 ID 滤波器仍然可用的原因。
- 头文件中只定义了一个宏 `CAN_MASTER_ID`（`0x200`），目前同时充当这两个角色。但它们是不同的寄存器，取值不要求相同。

### 电机侧的前置配置

用达妙调试助手逐台设置，一切才能工作：`ESC_ID` = `0x201`..`0x204`（逐台不同），`MST_ID` = `0x200`（**四台都要设** —— 出厂默认为 `0`，是"代码看着没问题却收不到反馈"的常见原因），`CTRL_MODE` = `1`（MIT）。`PMAX`/`VMAX`/`TMAX` 必须与 `func/Inc/func_can.h` 中的量程一致；扭矩是受控量，`TMAX` 不一致会使 `t_ff` 等比例缩放，静默破坏扭矩标定。

### 帧格式细节

- 定点数打包位于 `func/Src/func_can.c`：`float_to_uint` / `uint_to_float` 在 `[-Pmax,Pmax]`、`[-Vmax,Vmax]`、`[-Tmax,Tmax]` 区间上把物理量映射为原始位域。超出范围的浮点数会回绕/截断而不会限幅 —— 请在调用处自行限幅。
- MIT 指令帧格式见 `func_can_transmit_MIT`：16 位位置、12 位速度、12 位力矩 + 16 位 `Kp`、16 位 `Kd`，按大端跨 8 字节按位打包。是按位而非按字节对齐的。
- 特殊指令为 `func_can_transmit_Enable/DisEnable/SetZero/ClearErr` 这组辅助函数 —— 全 `0xFF` 载荷外加一个独特的末位命令字节。
- 反馈是**问询式而非周期上报**：只有收到与该电机 `ESC_ID` 匹配的控制帧后，电机才发送状态帧（手册《CAN 通信》章节）。不发控制帧 → 没有反馈，且电机的 `TIMEOUT` 保护（寄存器 `0x09`）会使其退出使能状态。要**轮流**给四个 `ESC_ID` 都发帧，不能只发 1 号。
- **接收路径是中断驱动**：RX FIFO0 挂起 → `HAL_CAN_RxFifo0MsgPendingCallback()`（位于 `func_can.c` 末尾）。它由 `D[0]` 低 4 位（即发送方的 `ESC_ID` 低 8 位）推导出电机序号，填充 `motor_info[index]`，并计算派生的 `*_f` 浮点字段。`motor_info[4]` 是全局变量（在 `func_can.h` 中 `extern`），会在中断上下文中被修改 —— 若关心数据一致性，请在主循环中整体快照，而不是逐字段读取。
- 滤波器组 0 有意配置为 32 位 ID 掩码模式、精确放行 `0x200`。**这是正确的**：所有电机都上报到共用的 `MST_ID`，其他 ID 无需放行。不要把它放宽到 `0x201`..`0x204` —— 那样会静默丢弃全部反馈。

新增一条电机指令，需要同时改动无枚举的序号运算（`id - CAN_MASTER_ID - 1`）、定点转换以及头文件中的量程定义。

## 编码约定

- 注释使用中文，通常是解释*思路*/*数学推导*的整块说明（参见转换函数和已废弃的 `pid.h` 头部注释块）。添加不易理解的代码时请沿用这种风格。
- 对外 API 放 `func/Inc/*.h`，实现放 `func/Src/*.c`；头文件先包含 `main.h`，再包含所需的外设生成头文件。
- 除 `syscalls.c` 的桩函数外，没有接 UART 的 `printf` 输出；USART1（PA9/PB7，9600 8N1）已初始化但应用代码未使用。

## 参考资料

`Datum/DM-J4340P-2EC V1.1 减速电机使用说明书V1.4 2026-09-14.pdf` 是电机数据手册 —— 帧格式、Pmax/Vmax/Tmax 量程和错误码的权威依据。

## 用户要求
1.需要对文件进行写操作时，必须用中文询问用户。
2.请使用中文和用户交流。
3.进行操作时，请增加这个操作的目的方便用户理解。