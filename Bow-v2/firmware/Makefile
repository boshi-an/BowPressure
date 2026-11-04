.DEFAULT_GOAL := all
PREFIX ?= arm-none-eabi-
CC = $(PREFIX)gcc
PYTHON ?= python3
SPI_BR ?= 3
CFLAGS = -mcpu=cortex-m0plus -mthumb -std=gnu11 -Os -g3 -Wall -Wextra -Werror \
         -ffreestanding -fno-builtin -ffunction-sections -fdata-sections \
         -Ivendor/st -Ivendor/cmsis -DSPI_BR=$(SPI_BR)
SOURCES = src/startup.c src/main.c
DEPS = $(wildcard src/*.inc src/*.h vendor/*/*.h) stm32c011f6.ld Makefile
.PHONY: all v2 v2_no_ltc v3 v3_no_ltc test clean FORCE
all: v2 v2_no_ltc v3 v3_no_ltc
# Target name, hardware revision, optional feature flags.
define VERSION
$(1): build/$(1).hex build/$(1).bin
build/$(1).elf: $(SOURCES) $(DEPS) config/v$(2).h FORCE
	@mkdir -p build
	$(CC) $(CFLAGS) -include config/v$(2).h $(3) $(SOURCES) -nostdlib -Tstm32c011f6.ld -Wl,--gc-sections,-Map=build/$(1).map -lgcc -o $$@
	$(PREFIX)size $$@
build/$(1).hex: build/$(1).elf
	$(PREFIX)objcopy -O ihex $$< $$@
build/$(1).bin: build/$(1).elf
	$(PREFIX)objcopy -O binary $$< $$@
endef
$(eval $(call VERSION,v2,2,))
$(eval $(call VERSION,v2_no_ltc,2,-DDISABLE_LTC_IRQ))
$(eval $(call VERSION,v3,3,))
$(eval $(call VERSION,v3_no_ltc,3,-DDISABLE_LTC_IRQ))
test:
	PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=host $(PYTHON) -m unittest discover -s tests -p 'test_*.py' -v
clean:
	rm -rf build
