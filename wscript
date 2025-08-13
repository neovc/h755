#!/usr/bin/env python
# encoding: utf-8

import os
import time
from datetime import datetime

from waflib.Build import BuildContext

APPNAME = 'h755'

top = '.'
out = 'build'
cwd = os.getcwd()
now = datetime.now()
bin_date = now.strftime("%y%m%d")
bin_target = APPNAME + "." + bin_date + ".bin"
elf_target = APPNAME + "." + bin_date + ".elf"
hex_target = APPNAME + "." + bin_date + ".hex"

libopencm3_revision = os.popen('cd libopencm3 && git log --pretty=format:"%h" --abbrev=8 2> /dev/null | head -1 || echo {0}'.format("1.")).read().strip()
libopencm3_revision_date = os.popen('cd libopencm3 && git log --pretty=format:"%ad" --date=format:"%Y%m%d" 2>/dev/null | head -1 | cut -f 1 -d\' \'').read().strip()

def down_libopencm3():
    if not os.path.exists('libopencm3'):
        print('begin downloading libopencm3 library...\n')
        os.system('git clone git@github.com:neovc/stm32h7 libopencm3')

def update_libopencm3():
    os.system('cd libopencm3 && git pull && make TARGETS=stm32/h7')

def options(ctx):
    ctx.load('gcc')

    down_libopencm3()

    ctx.add_option('--arch', action='store', default='cortex-m7', help='MCU arch')
    ctx.add_option('--toolchain', action='store', default='arm-none-eabi-', help='Set toolchain prefix')
    ctx.add_option('--update', action='store_true', help='Update libopencm3 source')

def configure(ctx):
    ctx.env.CC = ctx.options.toolchain + "gcc"
    ctx.env.AR = ctx.options.toolchain + "ar"
    ctx.load('gcc')

    # Locate programs
    ctx.find_program('st-flash', var='STFLASH')
    ctx.find_program(ctx.options.toolchain + 'size', var='SIZE')
    ctx.find_program(ctx.options.toolchain + 'objcopy', var='OBJCOPY')

    # Generate build arguments
    ctx.env.append_unique('CFLAGS', ['-Wall', '-DSTM32H7', '-fno-common', '-Os', '-mthumb', '-mcpu=cortex-m7', '-mhard-float', '-mfpu=fpv5-d16', '-fno-exceptions', '-ffunction-sections', '-fdata-sections', '-Wempty-body', '-Wtype-limits', '-Wmissing-parameter-type', '-Wuninitialized', '-fno-strict-aliasing', '-Wno-unused-function', '-Wno-stringop-truncation', '-fsingle-precision-constant'])

    ctx.env.append_unique('LINKFLAGS', ['--static', '-nostartfiles', '-Wl,--gc-sections', '-mthumb', '-mcpu=cortex-m7', '-mhard-float', '-mfpu=fpv5-d16'])

    ctx.env.append_unique('LDFLAGS', ['--specs=nano.specs', '-Wl,--start-group', '-lc', '-lgcc', '-lnosys', '-Wl,--end-group', '-lm'])

    ctx.env.append_unique('INCLUDES', ['../rtos/include', '../src', '../libopencm3/include'])

    # FreeRTOS
    ctx.env.append_unique('FILES', ['rtos/*.c', 'src/h755.c'])

    ctx.env.append_unique('LINKFLAGS', ['-T' + cwd + '/flash.ld', '-Wl,-Map=flash.map'])

    if ctx.options.update == True or not os.path.exists('libopencm3/lib/libopencm3_stm32h7.a'):
        update_libopencm3()

def build(ctx):
    # Linker script

    ctx.program(
        source=ctx.path.ant_glob(ctx.env.FILES),
        target=elf_target,
        stlib=['opencm3_stm32h7'],
        stlibpath=[cwd + '/libopencm3/lib']
    )
    ctx(rule='${OBJCOPY} -O binary ${SRC} ${TGT}', source=elf_target, target=bin_target, name='objcopy', always=True)
    ctx(name="size", rule='${SIZE} ${SRC}', source=elf_target, always=True)

def flash(ctx):
    ctx(name='flash', rule='${STFLASH} erase && ${STFLASH} write ${SRC} 0x8000000', source=bin_target, always=True)

class Program(BuildContext):
    cmd = 'flash'
    fun = 'flash'
