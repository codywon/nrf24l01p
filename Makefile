ccflags-y := -DDEBUG -Wno-unused-function
obj-m := nrf24l01p.o
all:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD)
clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) clean
