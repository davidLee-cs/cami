################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
application/%.o: ../application/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	ulimit -t 30 ; ulimit -s 1024 ; "/mnt/ccs/ccs/tools/compiler/ti-cgt-armllvm_4.0.1.LTS/bin/tiarmclang" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"/home/guest/ide/default/i2c_controller_rw_multibyte_fifo_interrupts_LP_MSPM0C1104_nortos_ticlang" -I"/home/guest/ide/default/i2c_controller_rw_multibyte_fifo_interrupts_LP_MSPM0C1104_nortos_ticlang/Debug" -I"/mnt/tirex-content/mspm0_sdk_2_03_00_07/source/third_party/CMSIS/Core/Include" -I"/mnt/tirex-content/mspm0_sdk_2_03_00_07/source" -gdwarf-3 -MMD -MP -MF"application/$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$(shell echo $<)"
	@echo 'Finished building: "$<"'
	@echo ' '


