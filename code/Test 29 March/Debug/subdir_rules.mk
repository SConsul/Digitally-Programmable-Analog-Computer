################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs900/ccs/tools/compiler/ti-cgt-c2000_18.12.1.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --vcu_support=vcu0 --include_path="C:/ti/ccs900/ccs/tools/compiler/ti-cgt-c2000_18.12.1.LTS/include" --include_path="C:/ti/c2000/C2000Ware_1_00_06_00/device_support/f2806x/headers/include" --include_path="C:/ti/c2000/C2000Ware_1_00_06_00/device_support/f2806x/common/include" --include_path="C:/ti/c2000/C2000Ware_1_00_06_00/libraries/math/IQmath/c28/include" --include_path="C:/ti/c2000/C2000Ware_1_00_06_00/libraries/math/FPUfastRTS/c28/include" -g --define="_DEBUG" --define="LARGE_MODEL" --quiet --verbose_diagnostics --diag_warning=225 --diag_suppress=10063 --issue_remarks --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2806x_Adc.obj: C:/ti/c2000/C2000Ware_1_00_06_00/device_support/f2806x/common/source/F2806x_Adc.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs900/ccs/tools/compiler/ti-cgt-c2000_18.12.1.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --vcu_support=vcu0 --include_path="C:/ti/ccs900/ccs/tools/compiler/ti-cgt-c2000_18.12.1.LTS/include" --include_path="C:/ti/c2000/C2000Ware_1_00_06_00/device_support/f2806x/headers/include" --include_path="C:/ti/c2000/C2000Ware_1_00_06_00/device_support/f2806x/common/include" --include_path="C:/ti/c2000/C2000Ware_1_00_06_00/libraries/math/IQmath/c28/include" --include_path="C:/ti/c2000/C2000Ware_1_00_06_00/libraries/math/FPUfastRTS/c28/include" -g --define="_DEBUG" --define="LARGE_MODEL" --quiet --verbose_diagnostics --diag_warning=225 --diag_suppress=10063 --issue_remarks --preproc_with_compile --preproc_dependency="F2806x_Adc.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2806x_CodeStartBranch.obj: C:/ti/c2000/C2000Ware_1_00_06_00/device_support/f2806x/common/source/F2806x_CodeStartBranch.asm $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs900/ccs/tools/compiler/ti-cgt-c2000_18.12.1.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --vcu_support=vcu0 --include_path="C:/ti/ccs900/ccs/tools/compiler/ti-cgt-c2000_18.12.1.LTS/include" --include_path="C:/ti/c2000/C2000Ware_1_00_06_00/device_support/f2806x/headers/include" --include_path="C:/ti/c2000/C2000Ware_1_00_06_00/device_support/f2806x/common/include" --include_path="C:/ti/c2000/C2000Ware_1_00_06_00/libraries/math/IQmath/c28/include" --include_path="C:/ti/c2000/C2000Ware_1_00_06_00/libraries/math/FPUfastRTS/c28/include" -g --define="_DEBUG" --define="LARGE_MODEL" --quiet --verbose_diagnostics --diag_warning=225 --diag_suppress=10063 --issue_remarks --preproc_with_compile --preproc_dependency="F2806x_CodeStartBranch.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2806x_DefaultIsr.obj: C:/ti/c2000/C2000Ware_1_00_06_00/device_support/f2806x/common/source/F2806x_DefaultIsr.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs900/ccs/tools/compiler/ti-cgt-c2000_18.12.1.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --vcu_support=vcu0 --include_path="C:/ti/ccs900/ccs/tools/compiler/ti-cgt-c2000_18.12.1.LTS/include" --include_path="C:/ti/c2000/C2000Ware_1_00_06_00/device_support/f2806x/headers/include" --include_path="C:/ti/c2000/C2000Ware_1_00_06_00/device_support/f2806x/common/include" --include_path="C:/ti/c2000/C2000Ware_1_00_06_00/libraries/math/IQmath/c28/include" --include_path="C:/ti/c2000/C2000Ware_1_00_06_00/libraries/math/FPUfastRTS/c28/include" -g --define="_DEBUG" --define="LARGE_MODEL" --quiet --verbose_diagnostics --diag_warning=225 --diag_suppress=10063 --issue_remarks --preproc_with_compile --preproc_dependency="F2806x_DefaultIsr.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2806x_GlobalVariableDefs.obj: C:/ti/c2000/C2000Ware_1_00_06_00/device_support/f2806x/headers/source/F2806x_GlobalVariableDefs.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs900/ccs/tools/compiler/ti-cgt-c2000_18.12.1.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --vcu_support=vcu0 --include_path="C:/ti/ccs900/ccs/tools/compiler/ti-cgt-c2000_18.12.1.LTS/include" --include_path="C:/ti/c2000/C2000Ware_1_00_06_00/device_support/f2806x/headers/include" --include_path="C:/ti/c2000/C2000Ware_1_00_06_00/device_support/f2806x/common/include" --include_path="C:/ti/c2000/C2000Ware_1_00_06_00/libraries/math/IQmath/c28/include" --include_path="C:/ti/c2000/C2000Ware_1_00_06_00/libraries/math/FPUfastRTS/c28/include" -g --define="_DEBUG" --define="LARGE_MODEL" --quiet --verbose_diagnostics --diag_warning=225 --diag_suppress=10063 --issue_remarks --preproc_with_compile --preproc_dependency="F2806x_GlobalVariableDefs.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2806x_PieCtrl.obj: C:/ti/c2000/C2000Ware_1_00_06_00/device_support/f2806x/common/source/F2806x_PieCtrl.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs900/ccs/tools/compiler/ti-cgt-c2000_18.12.1.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --vcu_support=vcu0 --include_path="C:/ti/ccs900/ccs/tools/compiler/ti-cgt-c2000_18.12.1.LTS/include" --include_path="C:/ti/c2000/C2000Ware_1_00_06_00/device_support/f2806x/headers/include" --include_path="C:/ti/c2000/C2000Ware_1_00_06_00/device_support/f2806x/common/include" --include_path="C:/ti/c2000/C2000Ware_1_00_06_00/libraries/math/IQmath/c28/include" --include_path="C:/ti/c2000/C2000Ware_1_00_06_00/libraries/math/FPUfastRTS/c28/include" -g --define="_DEBUG" --define="LARGE_MODEL" --quiet --verbose_diagnostics --diag_warning=225 --diag_suppress=10063 --issue_remarks --preproc_with_compile --preproc_dependency="F2806x_PieCtrl.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2806x_PieVect.obj: C:/ti/c2000/C2000Ware_1_00_06_00/device_support/f2806x/common/source/F2806x_PieVect.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs900/ccs/tools/compiler/ti-cgt-c2000_18.12.1.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --vcu_support=vcu0 --include_path="C:/ti/ccs900/ccs/tools/compiler/ti-cgt-c2000_18.12.1.LTS/include" --include_path="C:/ti/c2000/C2000Ware_1_00_06_00/device_support/f2806x/headers/include" --include_path="C:/ti/c2000/C2000Ware_1_00_06_00/device_support/f2806x/common/include" --include_path="C:/ti/c2000/C2000Ware_1_00_06_00/libraries/math/IQmath/c28/include" --include_path="C:/ti/c2000/C2000Ware_1_00_06_00/libraries/math/FPUfastRTS/c28/include" -g --define="_DEBUG" --define="LARGE_MODEL" --quiet --verbose_diagnostics --diag_warning=225 --diag_suppress=10063 --issue_remarks --preproc_with_compile --preproc_dependency="F2806x_PieVect.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2806x_Spi.obj: C:/ti/c2000/C2000Ware_1_00_06_00/device_support/f2806x/common/source/F2806x_Spi.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs900/ccs/tools/compiler/ti-cgt-c2000_18.12.1.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --vcu_support=vcu0 --include_path="C:/ti/ccs900/ccs/tools/compiler/ti-cgt-c2000_18.12.1.LTS/include" --include_path="C:/ti/c2000/C2000Ware_1_00_06_00/device_support/f2806x/headers/include" --include_path="C:/ti/c2000/C2000Ware_1_00_06_00/device_support/f2806x/common/include" --include_path="C:/ti/c2000/C2000Ware_1_00_06_00/libraries/math/IQmath/c28/include" --include_path="C:/ti/c2000/C2000Ware_1_00_06_00/libraries/math/FPUfastRTS/c28/include" -g --define="_DEBUG" --define="LARGE_MODEL" --quiet --verbose_diagnostics --diag_warning=225 --diag_suppress=10063 --issue_remarks --preproc_with_compile --preproc_dependency="F2806x_Spi.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2806x_SysCtrl.obj: C:/ti/c2000/C2000Ware_1_00_06_00/device_support/f2806x/common/source/F2806x_SysCtrl.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs900/ccs/tools/compiler/ti-cgt-c2000_18.12.1.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --vcu_support=vcu0 --include_path="C:/ti/ccs900/ccs/tools/compiler/ti-cgt-c2000_18.12.1.LTS/include" --include_path="C:/ti/c2000/C2000Ware_1_00_06_00/device_support/f2806x/headers/include" --include_path="C:/ti/c2000/C2000Ware_1_00_06_00/device_support/f2806x/common/include" --include_path="C:/ti/c2000/C2000Ware_1_00_06_00/libraries/math/IQmath/c28/include" --include_path="C:/ti/c2000/C2000Ware_1_00_06_00/libraries/math/FPUfastRTS/c28/include" -g --define="_DEBUG" --define="LARGE_MODEL" --quiet --verbose_diagnostics --diag_warning=225 --diag_suppress=10063 --issue_remarks --preproc_with_compile --preproc_dependency="F2806x_SysCtrl.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2806x_usDelay.obj: C:/ti/c2000/C2000Ware_1_00_06_00/device_support/f2806x/common/source/F2806x_usDelay.asm $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs900/ccs/tools/compiler/ti-cgt-c2000_18.12.1.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --vcu_support=vcu0 --include_path="C:/ti/ccs900/ccs/tools/compiler/ti-cgt-c2000_18.12.1.LTS/include" --include_path="C:/ti/c2000/C2000Ware_1_00_06_00/device_support/f2806x/headers/include" --include_path="C:/ti/c2000/C2000Ware_1_00_06_00/device_support/f2806x/common/include" --include_path="C:/ti/c2000/C2000Ware_1_00_06_00/libraries/math/IQmath/c28/include" --include_path="C:/ti/c2000/C2000Ware_1_00_06_00/libraries/math/FPUfastRTS/c28/include" -g --define="_DEBUG" --define="LARGE_MODEL" --quiet --verbose_diagnostics --diag_warning=225 --diag_suppress=10063 --issue_remarks --preproc_with_compile --preproc_dependency="F2806x_usDelay.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


