SHELL := /bin/bash
KERNELVERSION  ?= $(shell uname --kernel-release)
KSRC := /lib/modules/$(KERNELVERSION)/build

INSTALLDIR := /lib/modules/$(KERNELVERSION)/kernel/drivers/power/supply
MODDESTDIR := /lib/modules/$(KERNELVERSION)/kernel/drivers/power/supply
DKMSDIR := /usr/src/photonicat-pm

obj-m += photonicat-pm.o

all:
	$(MAKE) -C $(KSRC) M=$(shell pwd) modules

clean:
	$(MAKE) -C $(KSRC) M=$(shell pwd) clean

install: all
	@rm --force --verbose $(INSTALLDIR)/photonicat-pm.ko
	@mkdir --parent --verbose $(MODDESTDIR)
	@install --preserve-timestamps -D --mode=644 *.ko $(INSTALLDIR)
	@depmod --all $(KVER)
	@printf "%s\n" "Installion finished."

uninstall:
	@rm -f $(INSTALLDIR)/photonicat-pm.ko
	@depmod --all
	@printf "%s\n" "Uninstall finished."

dkms: all
	if [ -d $(DKMSDIR) ]; then\
	rm --recursive $(DKMSDIR)/*;\
	cp --recursive * $(DKMSDIR)/;\
	else \
		mkdir --verbose $(DKMSDIR);\
	cp --recursive * $(DKMSDIR);\
		dkms add -m photonicat-pm -v 1.0.0;\
	fi
	dkms build photonicat-pm --force -v 1.0.0
	dkms install photonicat-pm --force -v 1.0.0
	printf "%s\n" "Loading module"
	modprobe photonicat-pm
