#
# Makefile for the Silicon Image 8620 MHL TX device driver
#


# kbuild part of makefile
MHL_PRODUCT_NUM=8620
#
# color annotations to use instead of leading newline chars
ccflags-y += -DANSI_COLORS

ccflags-y += -DBUILD_NUM_STRING=\"$(MHL_BUILD_NUM)$(DEVELOPER_BUILD_ID)\"
ccflags-y += -DMHL_PRODUCT_NUM=$(MHL_PRODUCT_NUM)
ccflags-y += -DMHL_DRIVER_NAME=\"sii$(MHL_PRODUCT_NUM)drv\"
ccflags-y += -DMHL_DEVICE_NAME=\"sii-$(MHL_PRODUCT_NUM)\"
#ccflags-y += -DINPUT_DEV_ACCESSORY
#
# RCP_INPUTDEV_SUPPORT supports android key mapping for incoming RCP keys.
# Disable this support if key mapping is not required.
ccflags-y += -DRCP_INPUTDEV_SUPPORT
#
# FORCE_OCBUS_FOR_ECTS is used to identify code added for ECTS temporary fix that prohibits use of eCBUS.
# in addition module parameter force_ocbus_for_ects needs to be set as 1 to achieve ECTS operation.
ccflags-y += -DFORCE_OCBUS_FOR_ECTS
#
# MEDIA_DATA_TUNNEL_SUPPORT is for MDT support in chip independent code
ccflags-y += -DMEDIA_DATA_TUNNEL_SUPPORT
#
# BIST_INITIATOR enables driver to act as a BIST test initiator
ccflags-y += -DBIST_INITIATOR
#
# EARLY_HSIC enables hsic_init to configure the transmitter for USB host mode
#ccflags-y += -DEARLY_HSIC
#
# DEFER_RX_POWER ensures RX core is not powered up until EDID is read from downstream device.
# TODO: This option has to be removed. By default the RX core should be powered up later.
ccflags-y += -DDEFER_RX_POWER

#
# MANUAL_EDID_FETCH ensures RX core is not powered up until EDID is read from downstream device.
# TODO: This option has to be removed. By default the RX core should be powered up later.
ccflags-y += -DMANUAL_EDID_FETCH

# PRINT_ALL_INTR enables additional verbosity in the main driver level interrupt handler. 
#ccflags-y += -DPRINT_ALL_INTR

# PRINT_DDC_ABORTS enables logging of all DDC_ABORTs. Default "disabled" - helps MHL2 hot plug.
# Enable only if you must for debugging. 
#ccflags-y += -DPRINT_DDC_ABORTS

ccflags-y += -DDEBUG 
#
# Enable workaround for issue described in bugzilla 28919
ccflags-y += -DSWWA_28919
#
# the next three lines are optional - they enable greater verbosity in debug output
ccflags-y  += -DENABLE_EDID_INFO_PRINT
ccflags-y += -DENABLE_EDID_DEBUG_PRINT
ccflags-y += -DENABLE_DUMP_INFOFRAME
#
# uncomment next line to prevent EDID parsing. Useful for debugging.
#ccflags-$(CONFIG_DEBUG_DRIVER) += -DEDID_PASSTHROUGH

obj-$(CONFIG_SII8620_MHL_TX) += sii$(MHL_PRODUCT_NUM)drv.o
sii$(MHL_PRODUCT_NUM)drv-objs  += platform.o
sii$(MHL_PRODUCT_NUM)drv-objs  += mhl_linux_tx.o
sii$(MHL_PRODUCT_NUM)drv-objs  += mhl_rcp_inputdev.o
sii$(MHL_PRODUCT_NUM)drv-objs  += mhl_supp.o
sii$(MHL_PRODUCT_NUM)drv-objs  += si_8620_drv.o
sii$(MHL_PRODUCT_NUM)drv-objs  += si_mhl2_edid_3d.o
sii$(MHL_PRODUCT_NUM)drv-objs  += si_mdt_inputdev.o
sii$(MHL_PRODUCT_NUM)drv-objs  += si_emsc.o
sii$(MHL_PRODUCT_NUM)drv-objs  += si_emsc_hid.o


	