CFLAGS += \
  -mfpu=vfpv3 \
  -mfloat-abi=hard \
  -mabi=aapcs \
  -mcpu=cortex-a9 \
  -specs=/home/christi/Documents/Studium/hwsys/paranut/systems/doom_demo_2/doom_demo/hardware/firmware/Xilinx.spec \
  -L/home/christi/Documents/Studium/hwsys/paranut/systems/doom_demo_2/doom_demo/hardware/firmware/firmware_bsp/ps7_cortexa9_0/lib -L./ \
  -DCFG_TUSB_MCU=OPT_MCU_ZYBO \
  -MMD -MP \
  -Wl,-build-id=none
  #-mthumb
  #-lxil -lrsa -Wl,--start-group,-lxil,-lgcc,-lc,--end-group \
  #-Wl,--start-group,-lrsa,-lxil,-lgcc,-lc,--end-group \
  #-nostdlib -nostartfiles \
  #-flto \
  
  
LIBS += -lrsa -lxil -lxilffs

# suppress warning caused by vendor mcu driver
CFLAGS += -Wno-error=cast-align -Wno-error=shadow -Wno-error=undef -Wno-error=error-implicit-function-declaration \
		  -Wno-error=strict-prototypes -Wno-error=unused-parameter -Wno-error=type-limits -Wno-error=maybe-uninitialized

XILINX_DRIVER = hw/mcu/xilinx/XilinxProcessorIPLib/drivers/
XILINX_INCLUDES = /home/christi/Documents/Studium/hwsys/paranut/systems/doom_demo_2/doom_demo/hardware/firmware/firmware_bsp/ps7_cortexa9_0/include/
XILINX_STANDALONE= hw/mcu/xilinx/lib/bsp/standalone/

# All source paths should be relative to the top level.
LD_FILE = hw/bsp/$(BOARD)/lscript.ld 

#SRC_C += \
#	$(XILINX_DRIVER)/xparameters.c \
#	$(XILINX_DRIVER)/gpiops/xgpiops.c \
#	$(XILINX_DRIVER)/xplatform_info.c \
#	$(XILINX_DRIVER)/xscutimer.c \
#	$(XILINX_DRIVER)/xscugic.c \
#	$(XILINX_DRIVER)/xil_exception.c \
#	$(XILINX_DRIVER)/xil_printf.c \
#	$(XILINX_DRIVER)/xusbps.c \
#	$(XILINX_DRIVER)/xpseudo_asm.c \
#	$(XILINX_DRIVER)/xreg_cortexa9.c \
#	$(XILINX_DRIVER)/xil_cache.c \

# TODO: add startup code
#SRC_S += \

INC += \
	$(XILINX_INCLUDES) \
	$(TOP)/hw/bsp/$(BOARD)

# For TinyUSB port source
VENDOR = xilinx
CHIP_FAMILY = zybo

# TODO: ADD FREERTOS_PORT
# For freeRTOS port source
# FREERTOS_PORT = 
PARANUT_HOME=/home/christi/Documents/Studium/hwsys/paranut/

flash-%:
	$(PARANUT_HOME)/tools/paranut_flash -c $(SYSTEMS_DIR)/doom_demo_2/doom_demo//hardware/build/system.hdf $*
	
flash-bit:
	$(PARANUT_HOME)/tools/paranut_flash -c -b $(SYSTEMS_DIR)/doom_demo_2/doom_demo//hardware/build/system.bit $(SYSTEMS_DIR)/$*/hardware/build/system.hdf $(EXEC)

