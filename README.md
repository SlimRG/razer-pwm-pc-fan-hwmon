# Repository name: **razer-pwm-pc-fan-hwmon**

---

## README (EN)

### What this is

### Features

* **8 fan channels**:

  * Read RPM: `fan1_input … fan8_input` (RPM)
  * Set PWM: `pwm1 … pwm8` (0..255)
  * Enable mode: `pwm1_enable … pwm8_enable` (see “Control model”)
* **DKMS** installation (auto rebuild on kernel updates). ([manpages.debian.org][2])
* **Secure Boot aware**: supports DKMS module signing and MOK enrollment workflow. ([Ubuntu Wiki][3])
* **Stability “force write”**: every **2500 ms** the driver re-applies cached PWM values (best-effort) to prevent device drift/resets (mirrors behavior found in some userland implementations).

### Supported hardware

* Razer PWM PC Fan Controller (USB HID). (Check with `lsusb -d 1532:0f3c`.)

### How control works (important)

Linux uses the **hwmon PWM convention**:

* `pwmN` is typically **0..255** (0 = off, 255 = full speed). ([Linux Kernel Archives][4])
* `pwmN_enable` is typically:

  * `1` = manual/open-loop mode controlled by `pwmN`
  * `2` = automatic/closed-loop mode (if supported by hardware/driver)
  * `0` sometimes means “no control” depending on driver ([Linux Kernel Archives][4])

**This driver implements manual mode only** (the publicly known protocol shows a “manual” mode value; “auto” is not reliably documented). Behavior:

* `echo 1 > pwmN_enable` → manual enabled
* `echo X > pwmN` → sets duty cycle
* `echo 0 > pwmN_enable` → driver sets channel to a safe **default 50%** (not “auto”).
  This is done specifically so standard tools like **fancontrol** keep working even without a real hardware “auto” mode.

### Files in this repo

* `razer_pwm_pc_fan_controller.c` — kernel module source
* `Makefile` — DKMS/manual build
* `dkms.conf` — DKMS metadata
* `install_dkms.sh` — one-shot installer (recommended)
* `uninstall_dkms.sh` — uninstall helper
* `99-razer-pwm-fan.rules` — optional udev rule for auto-binding to this driver

---

## Installation (Debian/Ubuntu, “latest”)

### 1) Quick install (recommended)

Run from the repo folder:

```bash
chmod +x install_dkms.sh
sudo ./install_dkms.sh
```

What it does:

* Installs dependencies, registers module with **DKMS**, builds, installs.
* If **Secure Boot** is enabled, it triggers **MOK enrollment** (see next section). ([Ubuntu Wiki][3])

### 2) Secure Boot: what to expect

If Secure Boot is enabled, unsigned third-party kernel modules are usually blocked, so DKMS modules must be signed by a key enrolled into MOK. ([Ubuntu Wiki][3])

Check Secure Boot state:

```bash
mokutil --sb-state
```

Enroll DKMS public key (common default path):

```bash
sudo mokutil --import /var/lib/dkms/mok.pub
```

On the next reboot you’ll see **MOK Manager** (blue screen):

* choose **Enroll MOK**
* confirm fingerprint
* enter the one-time password you set
* reboot again ([documentation.ubuntu.com][5])

After enrollment:

```bash
sudo modprobe razer_pwm_pc_fan_controller
```

> Alternative Ubuntu-friendly method: `update-secureboot-policy --enroll-key` (uses the same underlying MOK flow). ([Ubuntu Wiki][3])

---

## Verify it works

### 1) Module loaded

```bash
lsmod | grep razer_pwm
dmesg | tail -n 80
```

### 2) hwmon nodes exist

```bash
grep -R . /sys/class/hwmon/hwmon*/name | grep -i razer
```

### 3) Read RPM / set PWM manually

Find the right hwmon directory (example `hwmon7`), then:

```bash
cd /sys/class/hwmon/hwmon7

# Read RPM
cat fan1_input

# Set channel 1 to ~60%
echo 1 | sudo tee pwm1_enable
echo 153 | sudo tee pwm1
```

(153 ≈ 60% × 255)

### 4) Use standard tools (lm-sensors + fancontrol)

Install tools:

```bash
sudo apt install -y lm-sensors fancontrol
```

See sensors:

```bash
sensors
```

Run PWM mapping wizard:

```bash
sudo pwmconfig
```

`pwmconfig` scans your hwmon PWM outputs and helps map which `pwmN` controls which `fanM`, then can generate a `fancontrol` config. ([manpages.ubuntu.com][1])

Start fancontrol:

```bash
sudo systemctl enable --now fancontrol
```

`fancontrol` reads a config file and continuously sets PWM based on temperatures. ([linux.die.net][6])

---

## Optional: ensure the device binds to this driver (udev)

Sometimes the device will bind to a generic HID driver first. Linux supports manual bind/unbind and the `driver_override` mechanism. ([Linux Kernel Archives][7])

Enable the provided rule:

```bash
sudo cp 99-razer-pwm-fan.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Manual bind/unbind example (HID bus id looks like `0003:1532:0F3C.xxxx`):

```bash
echo -n "0003:1532:0F3C.0001" | sudo tee /sys/bus/hid/drivers/hid-generic/unbind
echo razer_pwm_pc_fan | sudo tee /sys/bus/hid/devices/0003:1532:0F3C.0001/driver_override
echo -n "0003:1532:0F3C.0001" | sudo tee /sys/bus/hid/drivers/razer_pwm_pc_fan/bind
```

(Write your actual HID id.) ([Unix & Linux Stack Exchange][8])

---

## Uninstall

```bash
chmod +x uninstall_dkms.sh
sudo ./uninstall_dkms.sh
```

---

## Troubleshooting

### “Operation not permitted” / module won’t load on Secure Boot

* Check `mokutil --sb-state`
* Check kernel message: `dmesg | grep -i -E "Lockdown|signature|module verification"`
* Enroll MOK key and reboot (see Secure Boot section). ([documentation.ubuntu.com][5])

### `pwmconfig` says “no pwm-capable sensor modules”

That usually means no driver is exposing `pwm*` in hwmon or you’re looking at the wrong hwmon node. Ensure this driver is bound and check `/sys/class/hwmon/.../pwm1`. ([manpages.ubuntu.com][1])

### Fans don’t respond / RPM reads 0

* Confirm fans are 4-pin PWM and connected to the right ports.
* Try setting nonzero PWM and wait a moment.
* Some fans won’t report tach at very low duty.

---

## README (RU)

### Что это

Этот репозиторий — **внешний (out-of-tree) драйвер ядра Linux** для Debian/Ubuntu для **Razer PWM PC Fan Controller** (**USB HID**, **VID 0x1532 / PID 0x0f3c**). Драйвер экспортирует устройство через **стандартный интерфейс `hwmon`** в sysfs, поэтому управление/мониторинг доступны через **lm-sensors / sensors / pwmconfig / fancontrol** и другие “обычные” утилиты. ([manpages.ubuntu.com][1])

### Возможности

* **8 каналов**:

  * RPM: `fan1_input … fan8_input`
  * PWM: `pwm1 … pwm8` (0..255)
  * Режим: `pwm1_enable … pwm8_enable`
* Установка через **DKMS** (авто-пересборка при обновлении ядра). ([manpages.debian.org][2])
* **Secure Boot** учтён: DKMS-подпись модулей + MOK-enroll. ([Ubuntu Wiki][3])
* **Force-write каждые 2500 ms**: драйвер периодически повторно применяет текущие PWM значения для стабильности.

### Модель управления (важно)

Для `hwmon` обычно:

* `pwmN` — значение **0..255** (0 = стоп, 255 = максимум). ([Linux Kernel Archives][4])
* `pwmN_enable`:

  * `1` = ручной режим (управление через `pwmN`)
  * `2` = авто (если железо/драйвер поддерживает)
  * иногда `0` = “без управления” (зависит от драйвера) ([Linux Kernel Archives][4])

**Этот драйвер реализует только ручной режим** (auto в публично известном протоколе надёжно не описан). Поэтому:

* `echo 1 > pwmN_enable` — ручной режим
* `echo X > pwmN` — установить PWM
* `echo 0 > pwmN_enable` — ставит “безопасный дефолт” **50%** (не “авто”).

---

## Установка (Debian/Ubuntu “последние”)

### Быстрый способ

```bash
chmod +x install_dkms.sh
sudo ./install_dkms.sh
```

### Secure Boot (что будет)

При включённом Secure Boot неподписанные модули обычно блокируются, поэтому DKMS должен подписывать модуль ключом, который добавлен в MOK. ([Ubuntu Wiki][3])

Проверка:

```bash
mokutil --sb-state
```

Импорт публичного ключа DKMS (частый путь по умолчанию):

```bash
sudo mokutil --import /var/lib/dkms/mok.pub
```

Дальше **после перезагрузки** появится **MOK Manager**:

* **Enroll MOK**
* подтвердить отпечаток
* ввести пароль, который задали
* перезагрузиться ещё раз ([documentation.ubuntu.com][5])

После этого:

```bash
sudo modprobe razer_pwm_pc_fan_controller
```

(Альтернатива для Ubuntu: `update-secureboot-policy --enroll-key`.) ([Ubuntu Wiki][3])

---

## Проверка

1. sysfs:

```bash
grep -R . /sys/class/hwmon/hwmon*/name | grep -i razer
```

2. ручной тест:

```bash
cd /sys/class/hwmon/hwmonX
cat fan1_input
echo 1 | sudo tee pwm1_enable
echo 153 | sudo tee pwm1
```

3. стандартные утилиты:

* `pwmconfig` ищет PWM-выходы и связывает их с вентиляторами. ([manpages.ubuntu.com][1])
* `fancontrol` по конфигу регулирует PWM от температур. ([linux.die.net][6])

---

## Если устройство “схватил” другой HID-драйвер

Linux поддерживает bind/unbind и `driver_override`. ([Linux Kernel Archives][7])
Включи udev-правило (если нужно) или сделай bind вручную (см. секцию EN выше).

---

## Удаление

```bash
chmod +x uninstall_dkms.sh
sudo ./uninstall_dkms.sh
```

---

## README (中文 / CH)

### 这是什么

本仓库提供一个 **Linux 内核（out-of-tree）驱动**（面向 Debian/Ubuntu），用于 **Razer PWM PC Fan Controller**（USB HID，**VID 0x1532 / PID 0x0f3c**）。驱动通过标准 **`hwmon` sysfs** 导出风扇/ PWM 控制，因此可以用 **lm-sensors / sensors / pwmconfig / fancontrol** 等常见工具进行监控与控制。 ([manpages.ubuntu.com][1])

### 功能

* **8 路风扇通道**

  * 转速：`fan1_input … fan8_input`（RPM）
  * PWM：`pwm1 … pwm8`（0..255）
  * 使能：`pwm1_enable … pwm8_enable`
* 使用 **DKMS** 安装（内核更新自动重编译）。 ([manpages.debian.org][2])
* 支持 **Secure Boot**：DKMS 模块签名 + MOK 注册流程。 ([Ubuntu Wiki][3])
* **每 2500ms 强制重写（force-write）**：定期重新下发当前 PWM 设置，提高稳定性。

### 控制模型（重要）

`hwmon` 常见约定：

* `pwmN` 一般是 **0..255**（0=停止，255=全速）。 ([Linux Kernel Archives][4])
* `pwmN_enable` 常见含义：

  * `1` = 手动/开环（由 `pwmN` 控制）
  * `2` = 自动/闭环（如果硬件/驱动支持）
  * 有些驱动中 `0` 表示不控制 ([Linux Kernel Archives][4])

**本驱动只实现手动模式**（公开资料中“自动模式”协议不够可靠），因此：

* 写 `1`：手动控制
* 写 `pwmN`：设置占空比
* 写 `0`：设置为安全默认 **50%**（不是“自动”）。

---

## 安装（Debian/Ubuntu 最新版）

### 一键安装（推荐）

```bash
chmod +x install_dkms.sh
sudo ./install_dkms.sh
```

### Secure Boot 说明

Secure Boot 启用时，未签名的第三方模块通常无法加载；需要 DKMS 使用密钥签名，并将公钥注册到 MOK。 ([Ubuntu Wiki][3])

检查状态：

```bash
mokutil --sb-state
```

导入 DKMS 公钥（常见路径）：

```bash
sudo mokutil --import /var/lib/dkms/mok.pub
```

重启后会出现 **MOK Manager**：

* 选择 **Enroll MOK**
* 确认指纹
* 输入一次性密码
* 再次重启 ([documentation.ubuntu.com][5])

之后加载模块：

```bash
sudo modprobe razer_pwm_pc_fan_controller
```

（Ubuntu 也可用 `update-secureboot-policy --enroll-key` 引导注册。） ([Ubuntu Wiki][3])

---

## 验证

```bash
grep -R . /sys/class/hwmon/hwmon*/name | grep -i razer
```

进入对应 hwmon 目录后：

```bash
cat fan1_input
echo 1 | sudo tee pwm1_enable
echo 153 | sudo tee pwm1
```

使用标准工具：

* `pwmconfig` 用于检测 PWM 通道并建立 fan ↔ pwm 对应关系。 ([manpages.ubuntu.com][1])
* `fancontrol` 根据温度曲线持续调节 PWM。 ([linux.die.net][6])

---

## 设备被其它 HID 驱动抢占（可选）

Linux 支持 `bind/unbind` 与 `driver_override` 强制绑定到特定驱动。 ([Linux Kernel Archives][7])
可使用仓库自带 udev 规则（`99-razer-pwm-fan.rules`）实现自动绑定。

---

Если хочешь, я могу прямо в README добавить **“Copy-paste” секцию** (готовые команды для поиска正确的 `hwmonX`、自动生成 fancontrol 基础配置、以及常见错误的 dmesg 过滤命令) — но уже сейчас этого достаточно для обычного пользователя, чтобы поставить DKMS, пройти Secure Boot/MOK и управлять вентиляторами стандартными средствами.

[1]: https://manpages.ubuntu.com/manpages/jammy/man8/pwmconfig.8.html?utm_source=chatgpt.com "pwmconfig - tests the PWM outputs of sensors and ..."
[2]: https://manpages.debian.org/unstable/dkms/dkms.8.en.html?utm_source=chatgpt.com "dkms(8) — dkms — Debian unstable — Debian Manpages"
[3]: https://wiki.ubuntu.com/UEFI/SecureBoot/DKMS?utm_source=chatgpt.com "UEFI/SecureBoot/DKMS"
[4]: https://www.kernel.org/doc/Documentation/hwmon/g762?utm_source=chatgpt.com "g762"
[5]: https://documentation.ubuntu.com/security/security-features/platform-protections/secure-boot/?utm_source=chatgpt.com "UEFI Secure Boot"
[6]: https://linux.die.net/man/8/fancontrol?utm_source=chatgpt.com "fancontrol(8) - Linux man page"
[7]: https://www.kernel.org/doc/html/v6.1/driver-api/driver-model/binding.html?utm_source=chatgpt.com "Driver Binding — The Linux Kernel documentation"
[8]: https://unix.stackexchange.com/questions/12005/how-to-use-linux-kernel-driver-bind-unbind-interface-for-usb-hid-devices?utm_source=chatgpt.com "How to use Linux kernel driver bind/unbind interface for ..."
