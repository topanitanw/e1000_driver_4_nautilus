#/bin/bash -x

sudo qemu-system-x86\_64 -vga std -smp 2 -m 1024 \
	 -serial stdio -cdrom tinycore/tinycore.iso \
 	 -net nic,model=e1000 -net tap,ifname=tap0,script=no \
 	 -net dump,file=./tinycore.pcap | tee serial.out

