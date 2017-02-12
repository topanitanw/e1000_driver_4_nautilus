#/bin/bash -x

# sudo qemu-system-x86\_64 -smp 2 -m 2048 \
# 	-serial stdio -cdrom nautilus.iso \
# 	-net nic,model=e1000 -net tap,ifname=tap0,script=no \
# 	-net dump,file=./tx.pcap | tee serial.out

qemu-system-x86\_64 -smp 2 -m 2048 -serial stdio -cdrom nautilus.iso \
	-netdev user,id=network0 \
	-device e1000,netdev=network0,mac=06:05:04:03:02:01 | tee serial.out
