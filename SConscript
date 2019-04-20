from building import *

cwd = GetCurrentDir()

Import('asenv')
MODULES = asenv['MODULES']

srcs = []

if(('USB_SERIAL' in MODULES) or ('USB_CAN' in MODULES)):
    srcs += Glob('Src/usb*.c')
    srcs += Glob('Src/main.c')
    srcs += Glob('Src/stm32f1xx_hal_msp.c')
    srcs += Glob('Src/stm32f1xx_it.c')
    srcs += Glob('Src/usb.xml')
    if(asenv['RELEASE']!='asboot'):
        srcs += Glob('Src/can1_isr.xml')
    srcs += Glob('Drivers/STM32F1xx_HAL_Driver/Src/*.c')
    srcs += Glob('Middlewares/ST/STM32_USB_Device_Library/Core/Src/*.c')
    srcs += Glob('Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/*.c')
    SrcRemove(srcs, ['stm32f1xx_hal_flash.c'])
    asenv.Append(CPPPATH=[cwd+'/Inc',
                        cwd+'/Drivers/STM32F1xx_HAL_Driver/Inc',
                        cwd+'/Drivers/CMSIS/Device/ST/STM32F1xx/Include',
                        cwd+'/Drivers/CMSIS/Include',
                        cwd+'/Middlewares/ST/STM32_USB_Device_Library/Core/Inc',
                        cwd+'/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc'])
    asenv.Append(CPPDEFINES=['STM32F107xC'])

if('SDCARD' in MODULES):
    srcs += Glob('Src/spi1_isr.xml')

Return('srcs')
