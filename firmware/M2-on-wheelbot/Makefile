# --------------------------------------------------------
# Custom ATmega Makefile
# created by: Chao Liu(chaoliu@seas.upenn.edu)
# updated: Aug 28, 2016
# --------------------------------------------------------

# --------------------------------------------------------
# Support atmega88a, atmega168a and atmega32u4
# --------------------------------------------------------

# --------------------------------------------------------
# you shouldn't change anything below here,
# unless you really know what you're doing
# --------------------------------------------------------

# --------------------------------------------------------
# Specify the device you are using and its clock.
# --------------------------------------------------------

DEVICE		= atmega32u4
CLOCK		= 16000000

# --------------------------------------------------------
# if you are using JTAGICE mkII, let PROGRAMMER = jtag2isp;
# if you are using AVRISP mkII, let PROGRAMMER = avrispmkII;
# if you are using USB, let PROGRAMMER = USB.
# --------------------------------------------------------

PROGRAMMER	= USB

ifeq ($(DEVICE), atmega88a)
	TARGET_DEVICE = m88
	DEVICE_LABEL = ATmega88a
else ifeq ($(DEVICE), atmega168a)
	TARGET_DEVICE = m168
	DEVICE_LABEL = ATmega168a
else ifeq ($(DEVICE), atmega32u4)
	TARGET_DEVICE = m32u4
	DEVICE_LABEL = ATmega32U4
else
$(error DEVICE = $(DEVICE) is unknown.)
endif

COMPILE = avr-gcc -Wall -Os -DF_CPU=$(CLOCK) -mmcu=$(DEVICE)

SRCDIR = src
INCDIR = inc	# directory for header files

INCLUDES += -I$(INCDIR)

_SOURCES += $(wildcard $(SRCDIR)/*.c)
SOURCES = $(notdir $(_SOURCES))

OBJDIRS = obj_$(DEVICE)
OBJECTS := $(patsubst %.c,%.o, $(SOURCES))
OBJECTS_POS = $(addprefix $(OBJDIRS)/,$(OBJECTS))

vpath %.c $(dir $(_SOURCES))	# directory for source files
vpath %.o $(OBJDIRS)            # directory for object files
vpath %.elf $(OBJDIRS)
vpath %.hex .

# symbolic targets:
all:	main.hex
.PHONY : all

.c.o:
	@[ ! -e $@ ] && mkdir -p $(OBJDIRS)
	@$(COMPILE) $(INCLUDES) -c $< -o $(OBJDIRS)/$@
	@echo "[CC]  $^"

.S.o:
	@$(COMPILE) $(INCLUDES) -x assembler-with-cpp -c $< -o $(OBJDIRS)/$@
	@echo "[>-----Generate $@ Successfully-----<]"

.c.s:
	@$(COMPILE) $(INCLUDES) -S $< -o $(OBJDIRS)/$@
	@echo "[>-----Generate $@ Successfully-----<]"

fuse:
ifeq ($(DEVICE), $(filter $(DEVICE), atmega88a atmega168a))
	@avrdude -p $(TARGET_DEVICE) -c $(PROGRAMMER) -P usb -U lfuse:w:0xe2:m -U hfuse:w:0xdf:m -B10
	@echo "[>-----Program Fuse Done-----<]"
else ifeq ($(DEVICE), atmega32u4)
	@avrdude -p $(TARGET_DEVICE) -c $(PROGRAMMER) -P usb -U lfuse:w:0xe2:m -U hfuse:w:0xdf:m -B10
	@echo "[>-----Program Fuse Done-----<]"
endif

install: flash

flash: all
ifeq ($(DEVICE), $(filter $(DEVICE), atmega88a atmega168a))
	@avrdude -p $(TARGET_DEVICE) -c $(PROGRAMMER) -P usb -e -U flash:w:main.hex -B9
	@echo "[>-----$(DEVICE_LABEL) Loaded-----<]"
endif
ifeq ($(DEVICE), atmega32u4)
ifneq ($(PROGRAMMER), USB)
	@avrdude -p $(TARGET_DEVICE) -c $(PROGRAMMER) -P usb -e -U flash:w:main.hex -B9
	@echo "[>-----$(DEVICE_LABEL) Loaded-----<]"
else
	@dfu-programmer $(DEVICE) erase
	@dfu-programmer $(DEVICE) flash main.hex
	@echo "[>-----$(DEVICE_LABEL) Loaded-----<]"
endif
endif

clean:
	rm -fr main.hex $(OBJDIRS)

# file targets:
main.elf: $(OBJECTS)
	@$(COMPILE) -o $(OBJDIRS)/main.elf $(OBJECTS_POS) -lm
	@echo "[ELF] $(OBJDIRS)/$@"

main.hex: main.elf
	@rm -f main.hex
	@avr-objcopy -j .text -j .data -O ihex $(OBJDIRS)/main.elf main.hex
	@avr-size --format=avr --mcu=$(DEVICE) $(OBJDIRS)/main.elf
	@echo "[>-----Generate $@ Successfully-----<]"
	@echo "[>-----Build Successfully-----<]"

# Targets for code debugging and analysis:
disasm:	main.elf
	avr-objdump -d $(OBJDIRS)/main.elf

cpp:
	$(COMPILE) -E main.c
