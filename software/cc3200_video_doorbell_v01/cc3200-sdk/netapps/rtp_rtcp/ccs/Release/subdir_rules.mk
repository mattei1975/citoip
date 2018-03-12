################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
rtcp_rtp.obj: D:/CC3xxx/CC3101/Code/00_OV788/CC3200SDK_1.2.0/cc3200-sdk/netapps/rtp_rtcp/rtcp_rtp.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.5/bin/armcl" -mv7M4 --code_state=16 --float_support=vfplib --abi=eabi -me --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.5/include" --include_path="D:/CC3xxx/CC3101/Code/00_OV788/CC3200SDK_1.2.0/cc3200-sdk/oslib/" -g --define=ccs --define=cc3200 --define=SL_PLATFORM_MULTI_THREADED --define=DEBUG_PRINT_ --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="rtcp_rtp.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


