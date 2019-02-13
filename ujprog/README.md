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


## MinGW (Windows 32 bit targe exe; cross compiled from linux)

TODO: how to get 32 bit mingw environment?

complied with `i686-w64-mingw32`; this uses the same `ftd2xx.lib` as used for linux (CDM v2.12.28 WHQL Certified\i386\ftd2xx.lib)

`make -f Makefile.ming32`


## MinGW (Windows 64 bit target exe; cross compiled from linux)

compiled with `x86_64-w64-mingw32-gcc` (installed with `sudo apt-get install mingw-w64`)

Note this uses the 64bit `ftd2xx.amd64.lib` (`CDM v2.12.28 WHQL Certified\amd64\ftd2xx.lib`)

`make -f Makefile.ming32_64`

# NOTE on Windows Drivers

The JTAG features of this ujprog cannot be used concurrently with OpenOCD.

In order to use OpenOCD with the ULX3S, the `libusbK` dirvers are needed. One way of manually changing the drivers is to use [Zadig](https://zadig.akeo.ie/). The problem with using the `libusbK` drivers is that this ujprog will no longer work, as it needs the FTDI drivers.

## Change ULX3S Driver to libusbK using Zadig
The ULX3S is using the FTDI drivers if it shows up in the Device Manager - Ports (COM & LPT)

![ULX3S-as-FTDI-device](https://github.com/gojimmypi/f32c_tools/raw/master/ujprog/images/ULX3S-as-FTDI-device.PNG)

Launch Zadig and click on `Options - List all Devices`.  Select the ULC3S device from the dropdown

![Zadig-FTDI-to-libusbK](https://github.com/gojimmypi/f32c_tools/raw/master/ujprog/images/Zadig-FTDI-to-libusbK.PNG)

## Change ULX3S Driver to FTDI 

The FTDI drivers should already be installed. If so, Windows will automatically use these drivers when a new ULXS3 is plugged in. If the FTDI Drivers are not installed, they can be downloaded from https://www.ftdichip.com/Drivers/D2XX.htm (the setup executable noted in the comments column may be the easiest way to install the drivers). 

The ULX3S is using the libusbK drivers if it shows up in Device Manager - libusbK USB Devices. (typically when using OpenOCD)

![ULX3S-as-libusbK-device](https://raw.githubusercontent.com/gojimmypi/f32c_tools/master/ujprog/images/ULX3S-as-libusbK-device.PNG)

To remove the libusbK drivers, right click on your ULX3S device in Device Manager and select `Uninstall Device`:

![Uninstall-libusbK-device](https://raw.githubusercontent.com/gojimmypi/f32c_tools/master/ujprog/images/Uninstall-libusbK-device.PNG)

Then click the Uninstall button (don't check the box unless you want to actually uninstall the drivers from Windows and then reinstall the drivers later; we are only uninstalling the device):

![Uninstall-libusbK-device-step2](https://raw.githubusercontent.com/gojimmypi/f32c_tools/master/ujprog/images/Uninstall-libusbK-device-step2.PNG)

After clicking the Uninstall button, `Device Manager` may flicker a bit, but no message is typically shown. If the device was removed it will no longer be visible. If there are no other libusbK devices, then then entire `libusbK USB Devices` container will also be gone.

To complete the process of installing the FDTI drivers: Unplug the ULX3S, wait 30 seconds and plug it back in. Windows should automatically use the FTDI drivers and a new COM port will appear in `Device Manager - Ports (COM & LPT)` as shown above.

# Changes by gojimmypi Feb 13 2019:

Added this README.md

Microsoft dumpbin reports 51E00677 time/date Fri Jul 12 06:36:55 2013 for current repo `ftd2xx.lib` 

The most recent `ftd2xx.lib` in 2.12.28 is 599AE440 time/date Mon Aug 21 06:46:40 2017

See https://www.ftdichip.com/Drivers/CDM/CDM%20v2.12.28%20WHQL%20Certified.zip on https://www.ftdichip.com/Drivers/D2XX.htm

I copied the FTDI zip file: 

* `CDM v2.12.28 WHQL Certified\ftd2xx.h` to ./`ftd2xx.h`
* `CDM v2.12.28 WHQL Certified\i386\ftd2xx.lib` to repo ./`ftd2xx.lib`
* `CDM v2.12.28 WHQL Certified\amd64\ftd2xx.lib` to repo ./`ftd2xx.amd64.lib`

Compiling with `x86_64-w64-mingw32-gcc` results in this incompatibility error for `ftd2xx.lib` (32 bit vs 64 bit conflict):

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


