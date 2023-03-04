#include <common.h>
#include <bootstage.h>
#include <command.h>
#include <cpu_func.h>
#include <dm.h>
#include <log.h>
#include <asm/global_data.h>
#include <dm/root.h>
#include <env.h>
#include <image.h>
#include <u-boot/zlib.h>
#include <asm/byteorder.h>
#include <linux/libfdt.h>
#include <mapmem.h>
#include <fdt_support.h>
#include <asm/bootm.h>
#include <asm/secure.h>
#include <linux/compiler.h>
#include <bootm.h>
#include <vxworks.h>
#include <asm/cache.h>

#include <asm/setup.h>

unsigned long do_go_exec(ulong (*entry)(int, char *const[]), int argc,
                         char *const argv[]) {
  return armv8_switch_to_el1(argc, argv, 0, 0, entry, ES_TO_AARCH64);
}
