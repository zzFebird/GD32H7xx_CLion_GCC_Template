# GD32H7XX CLion+CMake+GCC模板

终于搓出了使用CLion+CMake+GCC的编译模板，这下可以和继续和keil说再见了，感谢GD技术人员提供的协助

测试环境（并不是说只有这个才能用）：

CLion Nova2024EAP

CMake 3.28.1

gcc-arm-none-eabi 12.2.1

openocd GD特制版本 [下载链接](https://github.com/xutongxin1/gd32f30x_gcc/releases/tag/openocd)

只是修改了cmake和H7对应的文件。DSP对应的是f303的

error: #error "Please select the target GD32H7XX device used in your application (in gd32h7xx.h file)"

修改了此处问题
