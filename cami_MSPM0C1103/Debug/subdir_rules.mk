################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
build-340739649: ../cami_MSPM0C1104_adc.syscfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: SysConfig'
	"C:/ti/ccs2001/ccs/utils/sysconfig_1.22.0/sysconfig_cli.bat" --script "C:/_IdeaSpark/FirmWare_Project/4Bdream/CAMI/cami_MSPM0C1103/cami_MSPM0C1104_adc.syscfg" -o "." -s "C:/ti/mspm0_sdk_2_04_00_06/.metadata/product.json" -d "MSPM0C110X" -p "WSON-8(DSG)" -r "Default" --context "system" --compiler ticlang
	@echo 'Finished building: "$<"'
	@echo ' '

device_linker.cmd: build-340739649 ../cami_MSPM0C1104_adc.syscfg
device.opt: build-340739649
device.cmd.genlibs: build-340739649
ti_msp_dl_config.c: build-340739649
ti_msp_dl_config.h: build-340739649
Event.dot: build-340739649

%.o: ./%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ccs2001/ccs/tools/compiler/ti-cgt-armllvm_4.0.1.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"C:/_IdeaSpark/FirmWare_Project/4Bdream/CAMI/cami_MSPM0C1103" -I"C:/_IdeaSpark/FirmWare_Project/4Bdream/CAMI/cami_MSPM0C1103/Debug" -I"C:/ti/mspm0_sdk_2_04_00_06/source/third_party/CMSIS/Core/Include" -I"C:/ti/mspm0_sdk_2_04_00_06/source" -gdwarf-3 -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

startup_mspm0c110x_ticlang.o: C:/ti/mspm0_sdk_2_04_00_06/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0c110x_ticlang.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ccs2001/ccs/tools/compiler/ti-cgt-armllvm_4.0.1.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"C:/_IdeaSpark/FirmWare_Project/4Bdream/CAMI/cami_MSPM0C1103" -I"C:/_IdeaSpark/FirmWare_Project/4Bdream/CAMI/cami_MSPM0C1103/Debug" -I"C:/ti/mspm0_sdk_2_04_00_06/source/third_party/CMSIS/Core/Include" -I"C:/ti/mspm0_sdk_2_04_00_06/source" -gdwarf-3 -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


