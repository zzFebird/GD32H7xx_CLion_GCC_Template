add_library(usb)

#file(GLOB_RECURSE USB_SRC *.c)

#file(GLOB_RECURSE USB_HEADER *.h)
#foreach(header ${USB_HEADER})
#    get_filename_component(dir ${header} DIRECTORY)
#    list(APPEND USB_INC ${dir})
#endforeach()
#list(REMOVE_DUPLICATES USB_INC)

#target_sources(usb PRIVATE ${USB_SRC})
#target_include_directories(usb PUBLIC ${USB_INC})

target_sources(usb PRIVATE
    GD32H7xx_usbhs_library/driver/Source/drv_usb_core.c
    GD32H7xx_usbhs_library/driver/Source/drv_usb_dev.c
    GD32H7xx_usbhs_library/driver/Source/drv_usbd_int.c
    GD32H7xx_usbhs_library/device/core/Source/usbd_core.c
    GD32H7xx_usbhs_library/device/core/Source/usbd_enum.c
    GD32H7xx_usbhs_library/device/core/Source/usbd_transc.c
    USB_APP/gd32h7xx_usb_hw.c
)

target_include_directories(usb PUBLIC
    GD32H7xx_usbhs_library/driver/Include
    GD32H7xx_usbhs_library/device/core/Include
    GD32H7xx_usbhs_library/ustd/common
    USB_APP
)

set(usb_class "msc")

if (usb_class STREQUAL "cdc")
    target_sources(usb PRIVATE
        GD32H7xx_usbhs_library/device/class/cdc/Source/cdc_acm_core.c
    )
    target_include_directories(usb PUBLIC
        GD32H7xx_usbhs_library/device/class/cdc/Include
        GD32H7xx_usbhs_library/ustd/class/cdc
    )
elseif(usb_class STREQUAL "msc")
    target_sources(usb PRIVATE
        GD32H7xx_usbhs_library/device/class/msc/Source/usbd_msc_bbb.c
        GD32H7xx_usbhs_library/device/class/msc/Source/usbd_msc_core.c
        GD32H7xx_usbhs_library/device/class/msc/Source/usbd_msc_data.c
        GD32H7xx_usbhs_library/device/class/msc/Source/usbd_msc_scsi.c
        USB_APP/usbd_storage_msd.c
    )
    target_include_directories(usb PUBLIC
        GD32H7xx_usbhs_library/device/class/msc/Include
        GD32H7xx_usbhs_library/ustd/class/msc
    )
endif()

target_compile_definitions(usb PUBLIC -DUSE_USB_FS -DUSE_IRC48M)

target_link_libraries(usb PRIVATE drivers)