################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
smartrf_settings/smartrf_settings.obj: ../smartrf_settings/smartrf_settings.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.6.LTS/bin/armcl" -mv7M3 --code_state=16 --float_support=vfplib -me --include_path="C:/git/UofACapstone18_grp1_git/Thermostat_CC1350_Concentrator" --include_path="C:/ti/simplelink_cc13x0_sdk_1_50_00_08/kernel/tirtos/packages/ti/sysbios/posix" --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.6.LTS/include" --define=BOARD_DISPLAY_USE_LCD=1 --define=DeviceFamily_CC13X0 -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --preproc_with_compile --preproc_dependency="smartrf_settings/smartrf_settings.d_raw" --obj_directory="smartrf_settings" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

smartrf_settings/smartrf_settings_predefined.obj: ../smartrf_settings/smartrf_settings_predefined.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.6.LTS/bin/armcl" -mv7M3 --code_state=16 --float_support=vfplib -me --include_path="C:/git/UofACapstone18_grp1_git/Thermostat_CC1350_Concentrator" --include_path="C:/ti/simplelink_cc13x0_sdk_1_50_00_08/kernel/tirtos/packages/ti/sysbios/posix" --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.6.LTS/include" --define=BOARD_DISPLAY_USE_LCD=1 --define=DeviceFamily_CC13X0 -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --preproc_with_compile --preproc_dependency="smartrf_settings/smartrf_settings_predefined.d_raw" --obj_directory="smartrf_settings" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


