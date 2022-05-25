include ../conf/setup.conf

GCCVER			= 8.2.0
GCC			= $(TOOLS)/source/gcc-$(GCCVER)
BINUTILS		= $(TOOLS)/source/binutils-2.29.1
UNCERTAIN_BINUTILS	= $(TOOLS)/source/uncertain-binutils-2_32
NEWLIB			= $(TOOLS)/source/newlib-2.5.0.20170922
PRINTF			= $(TOOLS)/source/printf-4.0.0
PUTCHAR			= $(TOOLS)/source/putchar
UNCERTAIN		= $(TOOLS)/source/uncertain

PREFIX_UNCERTAIN	= $(PREFIX)-uncertain


all: binutils gcc newlib printf

uncertain: binutils uncertain-binutils gcc newlib printf libuncertain

binutils: binutils-pre binutils-post

binutils-pre:
	if test -d $(TOOLS)/bin; then true; else mkdir $(TOOLS)/bin; fi;\
	if test -d $(TOOLS)/$(TARGET); then true; else mkdir $(TOOLS)/$(TARGET); fi;\
	cd $(BINUTILS);\
	$(DEL) objdir; mkdir -p objdir; cd objdir;\
	export MAKEINFO=missing && ../configure --disable-docs --target=$(TARGET-ARCH)\
		--prefix=$(PREFIX) --disable-libssp --disable-nls -v;\
	$(MAKE) CC=$(TOOLCC) CFLAGS="-D_FORTIFY_SOURCE=1" LD=$(TOOLCC) all install;\

binutils-post:
	cp $(PREFIX)/bin/$(TARGET-ARCH)* $(TOOLS)/bin/;\
	$(DEL) $(BINUTILS)/objdir;\


uncertain-binutils: uncertain-binutils-pre uncertain-binutils-post

uncertain-binutils-pre:
	if test -d $(TOOLS)/bin; then true; else mkdir $(TOOLS)/bin; fi
	if test -d $(TOOLS)/$(TARGET); then true; else mkdir $(TOOLS)/$(TARGET); fi
	cd $(UNCERTAIN_BINUTILS) \
	&& $(DEL) objdir \
	&& mkdir -p objdir \
	&& cd objdir \
	&& ../configure --disable-docs --target=$(TARGET-ARCH)\
		--prefix=$(PREFIX_UNCERTAIN) --disable-libssp --disable-nls \
	&& $(MAKE) CC=$(TOOLCC) CFLAGS="-D_FORTIFY_SOURCE=1" LD=$(TOOLCC) MAKEINFO=true all install

uncertain-binutils-post:
	cd $(PREFIX_UNCERTAIN)/bin/ \
	&& for f in $(TARGET-ARCH)*; do cp "$$f" "uncertain-$$f"; done
	cp $(PREFIX_UNCERTAIN)/bin/uncertain-$(TARGET-ARCH)* $(TOOLS)/bin/
	$(DEL) $(BINUTILS)/objdir


#
#
gcc: gcc-pre gcc-post
g++: g++-pre g++-post

#
#	On older macOS with MacPorts (e.g., Sierra), you might need to add:
#
#		--with-gmp=/opt/local --with-mpfr=/opt/local --with-mpc=/opt/local --with-libiconv-prefix=/opt/local
#
#	to the configure flags for both g++-pre and gcc-pre, and add
#
#		"-D_DARWIN_C_SOURCE" to both CFLAGS and CXXFLAGS flags
#
g++-pre:
	if test -d $(TOOLS)/bin; then true; else mkdir $(TOOLS)/bin; fi;\
	if test -d $(TOOLS)/$(TARGET); then true; else mkdir $(TOOLS)/$(TARGET); fi;\
	cd $(GCC);\
	$(DEL) objdir; mkdir -p objdir; cd objdir;\
	export CC=$(TOOLCC) CXX=$(TOOLCXX) LD=$(TOOLCC) CFLAGS='-fgnu89-inline -ansi -std=c99 -D_XOPEN_SOURCE=600 -D_DARWIN_C_SOURCE' CXXFLAGS='-D_XOPEN_SOURCE=600 -D_DARWIN_C_SOURCE' MAKEINFO=missing && ../configure --disable-docs --target=$(TARGET-ARCH) --prefix=$(PREFIX)\
		--disable-libssp --with-gnu-as --with-gnu-ld --with-newlib\
		--enable-languages="c,c++" $(ADDITIONAL_ARCH_FLAGS)\
		--with-headers=$(NEWLIB)/newlib/libc/include --with-gmp=/opt/local --with-mpfr=/opt/local --with-mpc=/opt/local --with-libiconv-prefix=/opt/local -v;\
	$(MAKE) CC=$(TOOLCC) CXX=$(TOOLCXX) LD=$(TOOLCC) CFLAGS="-fgnu89-inline -std=c99 -ansi -D_XOPEN_SOURCE=600 -D_DARWIN_C_SOURCE" CXXFLAGS='-D_XOPEN_SOURCE=600 -D_DARWIN_C_SOURCE' all;\
	$(MAKE) CC=$(TOOLCC) CXX=$(TOOLCXX) LD=$(TOOLCC) CFLAGS="-fgnu89-inline -ansi -std=c99 -D_XOPEN_SOURCE=600 -D_DARWIN_C_SOURCE" CXXFLAGS='-D_XOPEN_SOURCE=600 -D_DARWIN_C_SOURCE' install;\

g++-post:
	cp $(PREFIX)/lib/gcc/$(TARGET-ARCH)/$(GCCVER)/*.a $(SUNFLOWERROOT)/tools/tools-lib/$(TARGET)/;\
	cp $(PREFIX)/lib/*.a $(SUNFLOWERROOT)/tools/tools-lib/$(TARGET)/;\
	ln -s $(PREFIX)/bin/$(TARGET-ARCH)* $(TOOLS)/bin/;\
	$(DEL) $(GCC)/objdir;\
#	ln -s $(PREFIX)/$(TARGET-ARCH)/bin/* $(TOOLS)/bin/;\


gcc-pre:
	if test -d $(TOOLS)/bin; then true; else mkdir $(TOOLS)/bin; fi;\
	if test -d $(TOOLS)/$(TARGET); then true; else mkdir $(TOOLS)/$(TARGET); fi;\
	cd $(GCC);\
	$(DEL) objdir; mkdir -p objdir; cd objdir;\
	export CC=$(TOOLCC) CXX=$(TOOLCXX) LD=$(TOOLCC) CFLAGS='-fgnu89-inline -ansi -std=c99 -D_XOPEN_SOURCE=600 -D_DARWIN_C_SOURCE' CXXFLAGS='-D_XOPEN_SOURCE=600 -D_DARWIN_C_SOURCE' MAKEINFO=missing && ../configure --disable-docs --target=$(TARGET-ARCH) --prefix=$(PREFIX)\
		--disable-libssp --with-gnu-as --with-gnu-ld --with-newlib\
		--enable-languages=c $(ADDITIONAL_ARCH_FLAGS) --disable-libstdcxx-pch --disable-libmudflap --disable-libgomp\
		--with-headers=$(NEWLIB)/newlib/libc/include --with-gmp=/opt/local --with-mpfr=/opt/local --with-mpc=/opt/local --with-libiconv-prefix=/opt/local -v;\
	$(MAKE) CC=$(TOOLCC) CXX=$(TOOLCXX) LD=$(TOOLCC) CFLAGS="-fgnu89-inline -ansi -std=c99 -D_XOPEN_SOURCE=600 -D_DARWIN_C_SOURCE" CXXFLAGS='-D_XOPEN_SOURCE=600 -D_DARWIN_C_SOURCE';\
	$(MAKE) CC=$(TOOLCC) CXX=$(TOOLCXX) LD=$(TOOLCC) CFLAGS="-fgnu89-inline -ansi -std=c99 -D_XOPEN_SOURCE=600 -D_DARWIN_C_SOURCE" CXXFLAGS='-D_XOPEN_SOURCE=600 -D_DARWIN_C_SOURCE' install;\


gcc-post:
	cp $(PREFIX)/lib/gcc/$(TARGET-ARCH)/$(GCCVER)/*.a $(SUNFLOWERROOT)/tools/tools-lib/$(TARGET)/;\
	cp $(PREFIX)/lib/*.a $(SUNFLOWERROOT)/tools/tools-lib/$(TARGET)/;\
	ln -s $(PREFIX)/bin/$(TARGET-ARCH)* $(TOOLS)/bin/;\
	ln -s $(TOOLS)/bin/$(TARGET-ARCH)-gcc $(TOOLS)/bin/$(TARGET-ARCH)-cc;\
	$(DEL) $(GCC)/objdir;\
#	ln -s $(PREFIX)/$(TARGET-ARCH)/bin/* $(TOOLS)/bin/;\



newlib: newlib-pre newlib-post

newlib-pre:
	if test -d $(TOOLS)/bin; then true; else mkdir $(TOOLS)/bin; fi;\
	if test -d $(TOOLS)/$(TARGET); then true; else mkdir $(TOOLS)/$(TARGET); fi;\
	cd $(NEWLIB);\
	$(DEL) objdir; mkdir -p objdir; cd objdir;\
	../configure --target=$(TARGET-ARCH) --prefix=$(PREFIX)\
		-v --with-stabs --nfp;\
	$(MAKE) PATH=$(PATH):$(TOOLS)/$(TARGET)/bin:$(TOOLS)/bin -j5 CC=$(TOOLCC) all;\
	$(MAKE) PATH=$(PATH):$(TOOLS)/$(TARGET)/bin:$(TOOLS)/bin CC=$(TOOLCC) install;\

newlib-post:
	cp $(PREFIX)/$(TARGET-ARCH)/lib/*.a $(SUNFLOWERROOT)/tools/tools-lib/$(TARGET)/;\
	$(DEL) $(NEWLIB)/objdir;\



#
#	This is from @harrysarson:
#
printf: printf-pre printf-post

printf-pre:
	$(DEL) objdir; mkdir -p objdir; cd objdir;\
	$(CC) -Wall -Werror -g -c $(PRINTF)/printf.c;\
	$(CC) -Wall -Werror -g -c $(PUTCHAR)/putchar.c;\
	$(AR) cr libprintf.a printf.o putchar.o;

printf-post:
	cp $(PRINTF)/*.h $(SUNFLOWERROOT)/tools/$(TARGET)/include/
	cp objdir/lib*.a $(SUNFLOWERROOT)/tools/tools-lib/$(TARGET)/
	$(DEL) objdir;

libuncertain: libuncertain-pre libuncertain-post

libuncertain-pre:
	$(DEL) objdir; mkdir -p objdir; cd objdir;\
	$(UNCERTAIN_AS) --march=rv32ifdxun -g -c $(UNCERTAIN)/uncertain.S -o uncertain.o;\
	$(AR) cr libuncertain.a uncertain.o;

libuncertain-post:
	cp $(UNCERTAIN)/*.h $(SUNFLOWERROOT)/tools/$(TARGET)/include/
	cp objdir/lib*.a $(SUNFLOWERROOT)/tools/tools-lib/$(TARGET)/
	$(DEL) objdir;

clean:
	@echo '==> $(BINUTILS)'; $(DEL) $(BINUTILS)/objdir
	@echo '==> $(UNCERTAIN_BINUTILS)'; $(DEL) $(UNCERTAIN_BINUTILS)/objdir
	@echo '==> $(GCC)'; $(DEL) $(GCC)/objdir
	@echo '==> $(NEWLIB)'; $(DEL) $(NEWLIB)/objdir

nuke:
	@echo '==> $(BINUTILS)'; $(DEL) $(BINUTILS)/objdir
	@echo '==> $(UNCERTAIN_BINUTILS)'; $(DEL) $(UNCERTAIN_BINUTILS)/objdir
	@echo '==> $(GCC)'; $(DEL) $(GCC)/objdir
	@echo '==> $(NEWLIB)'; $(DEL) $(NEWLIB)/objdir
	@echo '==> $(TOOLS)'; $(DEL) $(TOOLS)/bin
	@echo '==> $(TOOLS)/tools-lib/$(TARGET)/*.a'; $(DEL) $(SUNFLOWERROOT)/tools/tools-lib/$(TARGET)/*.a
	@set -e; for dir in $(SUPPORTED-TARGETS); do\
		($(DEL) $$dir; echo 'rm -rf' $$dir);\
	done;\
