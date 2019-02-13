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

#Compiling
Unless regularly compiling for different targets, consider copying the respective `Makefile.[target]` to your `Makefile`.


## BSD

`make -f Makefile.bsd`


## Linux 
including WSL Ubuntu, but reminder there's no WSL support for USB devices, only tty!

`make -f Makefile.linux`


## OSX

`make -f Makefile.linux`


## Windows

`make -f Makefile.win`


## MinGW (32 bit)

complied with i686-w64-mingw32; this uses the same ftd2xx.lib as used for linux (CDM v2.12.28 WHQL Certified\i386\ftd2xx.lib)

`make -f Makefile.ming32`


## MinGW (64 bit)

compiled with x86_64-w64-mingw32-gcc (installed with `sudo apt-get install mingw-w64`)

Note this uses the 64bit ftd2xx.amd64.lib (CDM v2.12.28 WHQL Certified\amd64\ftd2xx.lib)

`make -f Makefile.ming32_64`


# Changes by gojimmypi Feb 13 2019:

Added this README.md

Microsoft dumpbin reports 51E00677 time/date Fri Jul 12 06:36:55 2013 for current repo `ftd2xx.lib` 

The most recent `ftd2xx.lib` in 2.12.28 is 599AE440 time/date Mon Aug 21 06:46:40 2017

See https://www.ftdichip.com/Drivers/CDM/CDM%20v2.12.28%20WHQL%20Certified.zip on https://www.ftdichip.com/Drivers/D2XX.htm

I copied the FTDI zip file: CDM v2.12.28 WHQL Certified\i386\ftd2xx.lib to repo ./`ftd2xx.lib`

I copied the FTDI zip file: CDM v2.12.28 WHQL Certified\ftd2xx.h to ./`ftd2xx.h`

Compiling with x86_64-w64-mingw32-gcc results in this error `ftd2xx.lib` (32 bit vs 64 bit conflict):

```
$ make -f Makefile.ming32_64
x86_64-w64-mingw32-gcc ujprog.o  -lftd2xx -o "ujprog.exe" -s -static -L. -lftd2xx.amd64.lib
/usr/bin/x86_64-w64-mingw32-ld: skipping incompatible ./ftd2xx.lib when searching for -lftd2xx
/usr/bin/x86_64-w64-mingw32-ld: cannot find -lftd2xx
/usr/bin/x86_64-w64-mingw32-ld: cannot find -lftd2xx.amd64.lib
collect2: error: ld returned 1 exit status
Makefile.ming32_64:26: recipe for target 'ujprog.exe' failed
make: *** [ujprog.exe] Error 1
```

So I created `Makefile.ming32_64` and added the 64-bit (not static) version (`CDM v2.12.28 WHQL Certified\amd64\ftd2xx.lib`) as `ftd2xx.amd64.lib`

I also added a `Makefile.ming32` to compile with `i686-w64-mingw32` but I could not test this.


