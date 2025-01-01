################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
build-1519658607: ../i2c_controller_rw_multibyte_fifo_interrupts.syscfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: SysConfig'
	"/mnt/ccs/ccs/utils/sysconfig_1.22.0/sysconfig_cli.sh" --script "/home/guest/ide/default/i2c_controller_rw_multibyte_fifo_interrupts_LP_MSPM0C1104_nortos_ticlang/i2c_controller_rw_multibyte_fifo_interrupts.syscfg" -o "." -s "/mnt/tirex-content/mspm0_sdk_2_03_00_07/.metadata/product.json" --compiler ticlang
	@echo 'Finished building: "$<"'
	@echo ' '

device_linker.cmd: build-1519658607 ../i2c_controller_rw_multibyte_fifo_interrupts.syscfg
device.opt: build-1519658607
device.cmd.genlibs: build-1519658607
ti_msp_dl_config.c: build-1519658607
ti_msp_dl_config.h: build-1519658607
Event.dot: build-1519658607

%.o: ./%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	ulimit -t 30 ; ulimit -s 1024 ; "/mnt/ccs/ccs/tools/compiler/ti-cgt-armllvm_4.0.1.LTS/bin/tiarmclang" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"/home/guest/ide/default/i2c_controller_rw_multibyte_fifo_interrupts_LP_MSPM0C1104_nortos_ticlang" -I"/home/guest/ide/default/i2c_controller_rw_multibyte_fifo_interrupts_LP_MSPM0C1104_nortos_ticlang/Debug" -I"/mnt/tirex-content/mspm0_sdk_2_03_00_07/source/third_party/CMSIS/Core/Include" -I"/mnt/tirex-content/mspm0_sdk_2_03_00_07/source" -gdwarf-3 -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$(shell echo $<)"
	@echo 'Finished building: "$<"'
	@echo ' '

startup_mspm0c110x_ticlang.o: /mnt/tirex-content/mspm0_sdk_2_03_00_07/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0c110x_ticlang.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	ulimit -t 30 ; ulimit -s 1024 ; "/mnt/ccs/ccs/tools/compiler/ti-cgt-armllvm_4.0.1.LTS/bin/tiarmclang" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"/home/guest/ide/default/i2c_controller_rw_multibyte_fifo_interrupts_LP_MSPM0C1104_nortos_ticlang" -I"/home/guest/ide/default/i2c_controller_rw_multibyte_fifo_interrupts_LP_MSPM0C1104_nortos_ticlang/Debug" -I"/mnt/tirex-content/mspm0_sdk_2_03_00_07/source/third_party/CMSIS/Core/Include" -I"/mnt/tirex-content/mspm0_sdk_2_03_00_07/source" -gdwarf-3 -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$(shell echo $<)"
	@echo 'Finished building: "$<"'
	@echo ' '


