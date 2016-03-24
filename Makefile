ccflags-y := -Wno-unused-variable \
	     -Wno-unused-function \
	     -Wno-unused-parameters
obj-m := nrf24l01p.o
all:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD)
clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) clean
