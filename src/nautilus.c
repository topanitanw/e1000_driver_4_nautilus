#include <nautilus.h>
#include <cga.h>
#include <paging.h>
#include <idt.h>
#include <spinlock.h>
#include <cpu.h>
#include <msr.h>
#include <cpuid.h>
#include <serial.h>
#include <smp.h>
#include <irq.h>
#include <thread.h>
#include <idle.h>
#include <percpu.h>

#include <barrier.h>
#include <rwlock.h>
#include <condvar.h>

#include <dev/apic.h>
#include <dev/pci.h>
#include <dev/ioapic.h>
#include <dev/timer.h>
#include <dev/kbd.h>

#include <lib/liballoc_hooks.h>
#include <lib/liballoc.h>

extern void go_c (int argc, char ** argv);

static struct naut_info nautilus_info;
extern spinlock_t printk_lock;

static void xcall_test (void * arg)
{
    printk("Running xcore test on core %u\n", my_cpu_id());
}

extern void ipi_test_setup(void);
extern void ipi_begin_test(cpu_id_t t);

static void tfun (void * in, void ** out)
{
    while (1) {
        printk("thread tfun running (tid=%u)\n", get_tid());
        yield();
    }
}

void 
main (unsigned long mbd, unsigned long magic) 
{
    struct naut_info * naut = &nautilus_info;

    memset(naut, 0, sizeof(struct naut_info));

    term_init();

    spinlock_init(&printk_lock);

    show_splash();

    setup_idt();

    serial_init();

    detect_cpu();

    paging_init(&(naut->sys.mem), mbd);

    init_liballoc_hooks();

    disable_8259pic();

    smp_early_init(naut);

    ioapic_init(&(naut->sys));

    apic_init(naut);

    kbd_init(naut);

    timer_init(naut);

    pci_init(naut);

    // setup per-core area for BSP
    msr_write(MSR_GS_BASE, (uint64_t)naut->sys.cpus[0]);

    sched_init();

    smp_setup_xcall_bsp(naut->sys.cpus[0]);

    smp_bringup_aps(naut);


    //thread_start(side_screensaver, NULL, NULL, 0, TSTACK_DEFAULT, NULL, 1);

    //smp_xcall(1, xcall_test, (void*)0xdeadbeef, 1);

    /*
    ipi_test_setup();

    ipi_begin_test(1);
    ipi_begin_test(8);
    ipi_begin_test(4);
    tls_test();
    */

    /*
    char * blah[] = {"test", 0};
    
    panic("end\n");
    go_c(1, blah);
    */
    /*
    int ret = thread_fork();

    if (ret == 0) {
        printk("Nautilus forked child thread yielding on core %u (tid=%u)\n", my_cpu_id(), get_tid());
    } else {
        printk("Nautilus main thread yielding on core %d (forked child %u)\n", my_cpu_id(), ret);
    }
    */


    thread_start(tfun, 
                  NULL,
                  NULL,
                  0,
                  TSTACK_DEFAULT,
                  NULL,
                  0);

    sti();

    while (1) {
        printk("nautilus main thread (core 0) yielding\n");
        yield();
    }
}

