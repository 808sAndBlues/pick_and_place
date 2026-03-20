sudo ip link set ${ARM_MAC} up

sudo ip addr add 192.168.1.2/24 dev ${ARM_MAC}