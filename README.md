# iceasy-eval-gd32vw55

GD32VW553-IOT 评测

使用 Linux 上的 SEGGER Embedded Studio

## 食用方法

* [SEGGER Embedded Studio 安装][segger]

  Arch Linux 上还可以用 AUR 的 `embedded-studio` 安装，截至写时构建脚本是 2024 下半年的版本，
  略微有些过期，可以自己更新 URL. 相比于 GigaDevice 的 Embedded Builder， 最大的好处是跨平台.
  当然爱折腾的同学也可以直接丢掉以上所有的 IDE，大致浏览了一下 SEGGER 工程的描述文件，手动用
  GCC 命令行编译也挺简单的，之后有空弄一下.

* RISC-V 工具链

  官方工程选的是 [Nuclei 的 Embedded Toolchain][nuclei]，应该说 GigaDevice 的 RISC-V 芯片很大
  程度上是 Nuclei 的 IP core 在主导，也就不奇怪了. Arch Linux 上可以通过 AUR 的 `nuclei-sys`
  安装，也可以使用 `extra/riscv64-elf-gcc` 配合 `extra/riscv32-elf-newlib`.

  安装了 SEGGER Embedded Studio 的话，使用 `riscv32-none-elf` 也 OK，不过最好仍然需要
  `extra/riscv64-elf-gcc` 里的 `libgcc` 与 `extra/riscv32-elf-newlib` 里的标准 C 库.

* GigaDevice GD32VW55x SDK

  将官方的 Wi-Fi & BLE SDK 压缩包 [`GD32VW55x_RELEASE_V1.0.3a.zip`][sdk] 解压到 `/vendor`
  目录下，注意要脱去外层目录，即最终的目录结构应该类似

```
/vendor
├── config
├── docs
├── MBL
├── MSDK
├── release_notes.txt
├── ROM-EXPORT
└── scripts
```

## 烧录

Arch Linux 可安装 AUR 的 `gd32-isp-console-bin`，不过读 GD32VW553 的 option bytes 好像有一些问题.
之后可能逆向一下协议，自己做一个方便一点的 ISP 下载程序.

[nuclei]: https://nucleisys.com/download.php
[segger]: https://www.segger.com/downloads/embedded-studio/
[sdk]: https://gd32mcu.com/cn/download/7?kw=GD32VW5
