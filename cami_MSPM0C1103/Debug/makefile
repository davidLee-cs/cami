################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ccs2001/ccs/tools/compiler/ti-cgt-armllvm_4.0.1.LTS

GEN_OPTS__FLAG := @"./device.opt" 
GEN_CMDS__FLAG := -Wl,-l"./device_linker.cmd" 

ORDERED_OBJS += \
"./ti_msp_dl_config.o" \
"./startup_mspm0c110x_ticlang.o" \
"./application/accelerometer.o" \
"./application/bma5.o" \
"./application/bma530.o" \
"./application/bma530_features.o" \
"./application/common.o" \
"./application/main_ver1p3.o" \
"./application/opt3007_functions.o" \
"./application/opt3007_hostControl.o" \
"./application/opt3007_registers.o" \
$(GEN_CMDS__FLAG) \
-Wl,-ldevice.cmd.genlibs \
-Wl,-llibc.a \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include application/subdir_vars.mk
-include subdir_rules.mk
-include application/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
chemi_MSPM0C1104_ver1p0.out 

EXE_OUTPUTS__QUOTED += \
"chemi_MSPM0C1104_ver1p0.out" 


# All Target
all: $(OBJS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "chemi_MSPM0C1104_ver1p0.out"

# Tool invocations
chemi_MSPM0C1104_ver1p0.out: $(OBJS) $(GEN_CMDS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: Arm Linker'
	"C:/ti/ccs2001/ccs/tools/compiler/ti-cgt-armllvm_4.0.1.LTS/bin/tiarmclang.exe" @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -gdwarf-3 -Wl,-m"chemi_MSPM0C1104_ver1p0.map" -Wl,-i"C:/ti/mspm0_sdk_2_04_00_06/source" -Wl,-i"C:/_IdeaSpark/FirmWare_Project/4Bdream/CAMI/cami_MSPM0C1103" -Wl,-i"C:/_IdeaSpark/FirmWare_Project/4Bdream/CAMI/cami_MSPM0C1103/Debug/syscfg" -Wl,-i"C:/ti/ccs2001/ccs/tools/compiler/ti-cgt-armllvm_4.0.1.LTS/lib" -Wl,--diag_wrap=off -Wl,--display_error_number -Wl,--warn_sections -Wl,--xml_link_info="chemi_MSPM0C1104_ver1p0_linkInfo.xml" -Wl,--rom_model -o "chemi_MSPM0C1104_ver1p0.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(GEN_MISC_FILES__QUOTED)$(GEN_FILES__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "ti_msp_dl_config.o" "startup_mspm0c110x_ticlang.o" "application\accelerometer.o" "application\bma5.o" "application\bma530.o" "application\bma530_features.o" "application\common.o" "application\main_ver1p3.o" "application\opt3007_functions.o" "application\opt3007_hostControl.o" "application\opt3007_registers.o" 
	-$(RM) "ti_msp_dl_config.d" "startup_mspm0c110x_ticlang.d" "application\accelerometer.d" "application\bma5.d" "application\bma530.d" "application\bma530_features.d" "application\common.d" "application\main_ver1p3.d" "application\opt3007_functions.d" "application\opt3007_hostControl.d" "application\opt3007_registers.d" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

