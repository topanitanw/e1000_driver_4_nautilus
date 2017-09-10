/*
 * Automatically generated C config: don't edit
 * Nautilus version: 
 * Thu Sep  7 20:58:39 2017
 */
#define AUTOCONF_INCLUDED

/*
 * Platform/Arch Options
 */
#define NAUT_CONFIG_X86_64_HOST 1
#undef NAUT_CONFIG_XEON_PHI
#undef NAUT_CONFIG_HVM_HRT
#define NAUT_CONFIG_MAX_CPUS 256
#define NAUT_CONFIG_MAX_IOAPICS 16
#undef NAUT_CONFIG_PALACIOS

/*
 * Nautilus AeroKernel Build Config
 */
#define NAUT_CONFIG_USE_NAUT_BUILTINS 1
#undef NAUT_CONFIG_CXX_SUPPORT
#define NAUT_CONFIG_TOOLCHAIN_ROOT ""

/*
 * Interface Options
 */
#define NAUT_CONFIG_THREAD_EXIT_KEYCODE 196

/*
 * Nautilus AeroKernel Configuration
 */
#define NAUT_CONFIG_MAX_THREADS 1024
#undef NAUT_CONFIG_USE_TICKETLOCKS
#undef NAUT_CONFIG_VIRTUAL_CONSOLE_CHARDEV_CONSOLE
#define NAUT_CONFIG_VIRTUAL_CONSOLE_SERIAL_MIRROR 1
#define NAUT_CONFIG_VIRTUAL_CONSOLE_SERIAL_MIRROR_ALL 1

/*
 * Scheduler Options
 */
#define NAUT_CONFIG_UTILIZATION_LIMIT 99
#define NAUT_CONFIG_SPORADIC_RESERVATION 10
#define NAUT_CONFIG_APERIODIC_RESERVATION 10
#define NAUT_CONFIG_HZ 10
#undef NAUT_CONFIG_AUTO_REAP
#undef NAUT_CONFIG_WORK_STEALING
#undef NAUT_CONFIG_INTERRUPT_THREAD
#undef NAUT_CONFIG_APERIODIC_DYNAMIC_QUANTUM
#undef NAUT_CONFIG_APERIODIC_DYNAMIC_LIFETIME
#undef NAUT_CONFIG_APERIODIC_LOTTERY
#define NAUT_CONFIG_APERIODIC_ROUND_ROBIN 1
#undef NAUT_CONFIG_REAL_MODE_INTERFACE
#undef NAUT_CONFIG_ISOCORE

/*
 * Garbage Collection Options
 */
#undef NAUT_CONFIG_GARBAGE_COLLECTION

/*
 * AeroKernel Performance Optimizations
 */
#undef NAUT_CONFIG_FPU_SAVE
#define NAUT_CONFIG_KICK_SCHEDULE 1
#undef NAUT_CONFIG_HALT_WHILE_IDLE
#undef NAUT_CONFIG_THREAD_OPTIMIZE
#undef NAUT_CONFIG_USE_IDLE_THREADS

/*
 * Debugging
 */
#define NAUT_CONFIG_DEBUG_INFO 1
#define NAUT_CONFIG_DEBUG_PRINTS 1
#undef NAUT_CONFIG_ENABLE_ASSERTS
#undef NAUT_CONFIG_PROFILE
#undef NAUT_CONFIG_SILENCE_UNDEF_ERR
#undef NAUT_CONFIG_ENABLE_STACK_CHECK
#undef NAUT_CONFIG_ENABLE_REMOTE_DEBUGGING
#undef NAUT_CONFIG_DEBUG_PAGING
#undef NAUT_CONFIG_DEBUG_BOOTMEM
#undef NAUT_CONFIG_DEBUG_BUDDY
#undef NAUT_CONFIG_DEBUG_KMEM
#undef NAUT_CONFIG_DEBUG_FPU
#undef NAUT_CONFIG_DEBUG_SMP
#undef NAUT_CONFIG_DEBUG_SFI
#undef NAUT_CONFIG_DEBUG_CXX
#undef NAUT_CONFIG_DEBUG_THREADS
#undef NAUT_CONFIG_DEBUG_GROUP
#undef NAUT_CONFIG_DEBUG_SCHED
#undef NAUT_CONFIG_DEBUG_GROUP_SCHED
#undef NAUT_CONFIG_DEBUG_TIMERS
#undef NAUT_CONFIG_DEBUG_SYNCH
#undef NAUT_CONFIG_DEBUG_BARRIER
#undef NAUT_CONFIG_DEBUG_NUMA
#undef NAUT_CONFIG_DEBUG_VIRTUAL_CONSOLE
#define NAUT_CONFIG_DEBUG_DEV 1
#define NAUT_CONFIG_DEBUG_CHARDEV 1
#define NAUT_CONFIG_DEBUG_BLKDEV 1
#define NAUT_CONFIG_DEBUG_NETDEV 1
#undef NAUT_CONFIG_DEBUG_FILESYSTEM
#undef NAUT_CONFIG_DEBUG_LOADER

/*
 * Parallel Runtime Integration
 */
#undef NAUT_CONFIG_LEGION_RT
#undef NAUT_CONFIG_NDPC_RT
#undef NAUT_CONFIG_NESL_RT
#define NAUT_CONFIG_NO_RT 1

/*
 * Device options
 */

/*
 * Serial Options
 */
#define NAUT_CONFIG_SERIAL_REDIRECT 1
#define NAUT_CONFIG_SERIAL_REDIRECT_PORT 2
#undef NAUT_CONFIG_APIC_FORCE_XAPIC_MODE
#undef NAUT_CONFIG_APIC_TIMER_CALIBRATE_INDEPENDENTLY
#undef NAUT_CONFIG_DEBUG_APIC
#undef NAUT_CONFIG_DEBUG_IOAPIC
#define NAUT_CONFIG_DEBUG_PCI 1
#undef NAUT_CONFIG_DEBUG_PS2
#undef NAUT_CONFIG_DEBUG_PIT
#undef NAUT_CONFIG_HPET
#undef NAUT_CONFIG_VIRTIO_PCI
#undef NAUT_CONFIG_E1000_PCI
#define NAUT_CONFIG_E1000E_PCI 1
#define NAUT_CONFIG_DEBUG_E1000E_PCI 1
#undef NAUT_CONFIG_RAMDISK
#undef NAUT_CONFIG_ATA

/*
 * Filesystems
 */
#undef NAUT_CONFIG_EXT2_FILESYSTEM_DRIVER
#undef NAUT_CONFIG_FAT32_FILESYSTEM_DRIVER

/*
 * Lua support
 */
#undef NAUT_CONFIG_LOAD_LUA
