#-------------------------------------------------------------------------------#
#                                                                               #
#                         Copyright (c) James Pearman                           #
#                                    2014                                       #
#                             All Rights Reserved                               #
#                                                                               #
#-------------------------------------------------------------------------------#
#                                                                               #
#     The author is supplying this software for use with the VEX IQ             #
#     control system. This file can be freely distributed and teams are         #
#     authorized to freely use this program , however, it is requested that     #
#     improvements or additions be shared with the Vex community via the vex    #
#     forum.  Please acknowledge the work of the authors when appropriate.      #
#     Thanks.                                                                   #
#                                                                               #
#     Licensed under the Apache License, Version 2.0 (the "License");           #
#     you may not use this file except in compliance with the License.          #
#     You may obtain a copy of the License at                                   #
#                                                                               #
#       http://www.apache.org/licenses/LICENSE-2.0                              #
#                                                                               #
#     Unless required by applicable law or agreed to in writing, software       #
#     distributed under the License is distributed on an "AS IS" BASIS,         #
#     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  #
#     See the License for the specific language governing permissions and       #
#     limitations under the License.                                            #
#                                                                               #
#     The author can be contacted on the vex forums as jpearman                 #
#     or electronic mail using jbpearman_at_mac_dot_com                         #
#     Mentor for team 8888 RoboLancers, Pasadena CA.                            #
#-------------------------------------------------------------------------------#
#                                                                               #
#     Makefile for msp430 vex iq reference sensor when using GCC from the       #
#     command line.                                                             #
#                                                                               #
#     The msp430 gcc toolchain needs to be in the PATH environmental variable   #
#                                                                               #
#-------------------------------------------------------------------------------#
#
# 'make' builds everything
# 'make clean' deletes everything except source files and Makefile
# You need to set the MCU for your project.
# TARGET is the name of the executable file to be produced 
# $(TARGET).elf $(TARGET).hex and $(TARGET).map are all generated.
# 
#-------------------------------------------------------------------------------#
# Basename for the output files 
TARGET	   = viq_sensor

# Select the target processor
# The MSP430 launchpad ships with the following
MCU		   = msp430g2553
#MCU		= msp430g2452

# Source files
SOURCES = main.c viq_device.c viq_flash.c viq_i2c_hl.c viq_i2c_usci.c viq_i2c_usi.c

# Additional include files
#INCLUDES =

# Add or subtract whatever MSPGCC flags you want. There are plenty more
#-------------------------------------------------------------------------------#
CFLAGS	 = -mmcu=$(MCU) -g -Os -Wall -Wunused $(INCLUDES)	
ASFLAGS	 = -mmcu=$(MCU) -x assembler-with-cpp -Wa,-gstabs
LDFLAGS	 = -mmcu=$(MCU) -Wl,-Map=$(TARGET).map
#-------------------------------------------------------------------------------#
CC		 = msp430-gcc
LD		 = msp430-ld
AR		 = msp430-ar
AS		 = msp430-gcc
GASP	 = msp430-gasp
NM		 = msp430-nm
OBJCOPY	 = msp430-objcopy
RANLIB	 = msp430-ranlib
STRIP	 = msp430-strip
SIZE	 = msp430-size
READELF	 = msp430-readelf
CP		 = cp -p
RM		 = rm -f
MV		 = mv
#-------------------------------------------------------------------------------#
# the file which will include dependencies
DEPEND = $(SOURCES:.c=.d)

# all the object files
OBJECTS = $(SOURCES:.c=.o)

all: $(TARGET).elf $(TARGET).hex
$(TARGET).elf: $(OBJECTS)
	echo "Linking $@"
	$(CC) $(OBJECTS) $(LDFLAGS) $(LIBS) -o $@
	echo
	echo ">>>> Size of Firmware <<<<"
	$(SIZE) $(TARGET).elf
	echo
%.hex: %.elf
	$(OBJCOPY) -O ihex $< $@
%.o: %.c
	echo "Compiling $<"
	$(CC) -c $(CFLAGS) -o $@ $<
	
# rule for making assembler source listing, to see the code
%.lst: %.c
	$(CC) -c $(CFLAGS) -Wa,-anlhd $< > $@
	
# include the dependencies unless we're going to clean, then forget about them.
ifneq ($(MAKECMDGOALS), clean)
-include $(DEPEND)
endif

# dependencies file
# includes also considered, since some of these are our own
# (otherwise use -MM instead of -M)
%.d: %.c
	echo "Generating dependencies $@ from $<"
	$(CC) -M ${CFLAGS} $< >$@
.SILENT:
.PHONY: clean
clean:
	-$(RM) $(OBJECTS)
	-$(RM) $(TARGET).map
	-$(RM) $(TARGET).elf $(TARGET).hex
	-$(RM) $(TARGET).lst
	-$(RM) $(SOURCES:.c=.lst)
	-$(RM) $(DEPEND)
