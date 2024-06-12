# ULX2S / ULX3S JTAG programmer usage:

```
ULX2S / ULX3S JTAG programmer v 3.0.92  
Usage: ujprog [option(s)] [bitstream_file]

 Valid options:
  -p PORT       Select USB JTAG / UART PORT (default is 0)
  -P TTY        Select TTY port (valid only with -t or -a)
  -j TARGET     Select bitstream TARGET as SRAM (default) or FLASH (XP2 only)
  -f ADDR       Start writing to SPI flash at ADDR, optional with -j flash
  -s FILE       Convert bitstream to SVF FILE and exit
  -r            Reload FPGA configuration from internal Flash (XP2 only)
  -t            Enter terminal emulation mode after completing JTAG operations
  -b SPEED      Set baudrate to SPEED (300 to 3000000 bauds)
  -e FILE       Send and execute a f32c (MIPS/RISCV) binary FILE
  -x SPEED      Set binary transfer speed, optional with -e
  -a FILE       Send a raw FILE
  -d            debug (verbose)
  -D DELAY      Delay transmission of each byte by DELAY ms
  -q            Suppress messages
```

# Compiling

Unless regularly compiling for different targets, consider copying or
symlinking the respective `Makefile.[target]` to your `Makefile` and
use just "make" to compile, for example

`ln -s Makefile.linux Makefile`

`make`


## BSD

`make -f Makefile.bsd`


## Linux PC (i386/amd64)
Native linux Debian/Ubuntu, maybe many others too,
including WSL Ubuntu, but reminder there's no WSL support for USB devices, only tty!

`make -f Makefile.linux`


## Linux RaspberryPI-3 (armhf)

Edit Makefile.linux and enable this:
`ARCHNAME = arm-linux-gnueabihf`

`make -f Makefile.linux`


## OSX

`make -f Makefile.osx`


## Windows

`make -f Makefile.win`

# Troubleshooting

*** WINDOWS ***

Most issues come from windows platform. In some cases
ujprog doesn't work. Sometimes it is just a matter of dynamic
linking (DLL file "ftd2xx.dll" or "ftd2xx64.dll", easiest is
just to copy this file from FTDI D2XX CDM driver to the same
directory where ujprog.exe is)

On VoIFS there is strange problem related with ujprog.exe
compiled with mingw. ujprog.exe, if started from "wrong" directory
doesn't work. When started from "wrong" directory, ujprog.exe
will exit without printing any error or any other message while
it should print help/usage message shown on top of this page.
Possible cause of this problem is that "ftd2xx64.dll" (for win64)
is found copied to System32 directory under name "ftd2xx.dll" (for win32).

Possible solution would be to remove all ftd2xx copies and copy
ujprog.exe and dll to another directory and try again.

*** LINUX ***

Here we have much better success, ujprog is statically linked and
doesn't depend on any other file. Most issues come from user permissions
so ujprog should be either run as root or the user should be given
permissions to access USB and serial device, that's usually done
easily with some udev rule:

    # /etc/udev/rules.d/80-fpga-ulx3s.rules
    # this is for usb-serial tty device
    SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6015", \
      MODE="664", GROUP="dialout"
    # this is for ujprog libusb access
    ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6015", \
      GROUP="dialout", MODE="666"

*** APPLE ***

There can be many problems, I don't know what to do
one of the issues is that ujprog executable may needs
some dynamic linked library of specific version like libusb

*** CMake ***

It is standard CMake procedure:
    # mkdir build
    # cmake ..
    # make
    # make install

You can also pass optional parameters:

    # cmake -DBUILD_STATIC=ON -DLIBFTDISTATIC=/opt/libftdi/lib/libftdi.a -DLIBUSB0STATIC=/opt/libusb0/lib/libusb.a ..
    # make install/strip

