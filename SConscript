from building import *

cwd = GetCurrentDir()

Import('asenv')
MODULES = asenv['MODULES']

objs = []

if('USB_SERIAL' in MODULES):
    env = ForkEnv(asenv)
    srcs = Glob('Src/usb*.c')
    srcs += Glob('Src/stm32f1xx_it.c')
    srcs += Glob('Drivers/STM32F1xx_HAL_Driver/Src/*.c')
    srcs += Glob('Middlewares/ST/STM32_USB_Device_Library/Core/Src/*.c')
    srcs += Glob('Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/*.c')
    SrcRemove(srcs, ['stm32f1xx_hal_flash.c'])
    env.Append(CPPPATH=[cwd+'/Inc',
                        cwd+'/Drivers/STM32F1xx_HAL_Driver/Inc',
                        cwd+'/Drivers/CMSIS/Device/ST/STM32F1xx/Include',
                        cwd+'/Drivers/CMSIS/Include',
                        cwd+'/Middlewares/ST/STM32_USB_Device_Library/Core/Inc',
                        cwd+'/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc'])
    env.Append(CPPDEFINES=['STM32F107xC'])
    objs += env.Object(srcs)

Return('objs')
