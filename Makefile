SUBDIRS = 3-6

.PHONY : all install uninstall distclean clean subdirs $(SUBDIRS)
.NOTPARALLEL :

all : install

install : subdirs
uninstall : subdirs
distclean : subdirs
clean : subdirs

subdirs : $(SUBDIRS)

$(SUBDIRS):
	$(MAKE) -C $@ $(MAKECMDGOALS)
