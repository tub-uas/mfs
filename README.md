# Modular Flight System (MFS)

This repository contains the software running on the ESP32 chips that are part of the Modular Flight System (MFS). We are currently supporting 4 different board types / PCBs (Printed Circuit Boards):

- RAI (Radio Actuator Interface)
- AHRS (Altitude Heading Reference System)
- PSU (Power Supply Unit)
- GPS (Global Positioning System)

All boards share a common (this) code base, since they all use very similar driver / boilerplate software.

## Getting started

Install PlatformIO Core using the guide provided [here](https://docs.platformio.org/en/latest/core/installation.html). More specifically, run this command in your console `python3 -c "$(curl -fsSL https://raw.githubusercontent.com/platformio/platformio/master/scripts/get-platformio.py)" ` and add this line `export PATH=$PATH:~/.platformio/penv/bin` to your `~/.profile`.

You should now be able to compile and upload code to the boards via the 6 pin UART program header using the ESP-Prog. Power up the boards using a battery or a power supply (make sure the voltage is around 12v). Ensure that the ESP-Prog does not provide any power to the boards via the 6 pin header (set the jumpers accordingly). Connect the board of interest (if its not the PSU board itself) to the PSU boad via the CAN D-SUB connector and ensure that at least one LED lights up, indicating that the board is powered up. Finally connect the ESP-Prog to your computer and the board via the 6 pin header to the ESP-Prog.

#### Commands of interest
- Compile and upload from terminal `pio run -t upload -e <BOARD>`
- Monitor serial traffic in terminal `pio device monitor --raw`
- Compile, upload and monitor `pio run -t upload -e <BOARD> && pio device monitor --raw`
- List available devices `pio device list`
- Menu Config `pio run --t menuconfig`
- Run static checks on code `pio check`

`<BOARD>` can be any of the following:
- `RAI`
- `PSU`
- `AHRS`
- `GPS`

#### Issues
The `Cannot open /dev/ttyUSBx: Permission denied` error is caused by the user not having access to the serial ports. More specifically, the user is not in the `dialout` group. Run this `sudo usermod -a -G dialout $USER` and log out and back in again to fix it.

For anyone who likes knowing what they're running before they run it:
- `usermod` - modify a user account
- `-a` - add the user to supplementary groups
- `-G` - a list of supplementary groups (man page says to use `-a` only with `-G`)
- `dialout` - group that controls access to serial ports (and other hardware too)
- `$USER` - Bash variable containing current username (not a builtin, usually automatically set env variable)

## Architecture

TODO


## Individual boards

#### RAI

The RAI (Radio Actuator Interface) is the most important board of all boards. It enables communication from the RC (Radio Control) receiver, which receives information form the pilots transmitter, to the servos and the motor. It is also able to transmit and receive control data over the CAN bus. This way the pilot can enable the autopilot / controller running on the Flight Control Computer (FCC). The software running on the RAI needs to be extremely reliable and bug free. Any fault could cause a loss of communication with the pilot which would inevitably lead to a loss of the aircraft.

The main tasks of the RAI include:
- Receiving control data from the RC receiver via UART (Universal Asynchronous Receiver-Transmitter)/ SUMD (Digital SUM Signal)
- Control of servo and motor via PWM (Puls Width Modulatiom)
- Communication via CAN with the FCC (transmitting and receiving control data)
- Switching between manual (RC receiver) and automatic (FCC) flight control
- In case of communication loss activate fail safe


#### AHRS

The AHRS (Altitude Heading Reference System) provides orientation (attitude) information.

Steps needed for IMU / AHRS calibration:

- [ ] Mercury


#### PSU

PSU (Power Supply Unit) supplies power to the system via the CAN bus. It also provides information about motor (and system) voltage, current and power consumption.   


#### GPS

The GPS (Global Positioning System) provides the system with positioning information. It is actually using the GNSS (Global Navigation Satellite System), i.e. it is using not only the GPS, but also the Galileo, GLONASS, etc. systems.


## CAN IDs

#### General

This table provides a high level overview over what CAN IDs are used by which subsystem:

| Range Dec   | Range Hex     | Function |
|:-----------:|:-------------:|:--------:|
| 0 - 99      | 0x000 - 0x063 | SYS/PRIO |
| 100 - 199   | 0x064 - 0x0C7 | SYS/PRIO |
| 200 - 299   | 0x0C8 - 0x12B | ...      |
| 300 - 399   | 0x12C - 0x18F | FCC      |
| 400 - 499   | 0x190 - 0x1F3 | RAI      |
| 500 - 599   | 0x1F4 - 0x257 | AHRS     |
| 600 - 699   | 0x258 - 0x2BB | ...      |
| 700 - 799   | 0x2BC - 0x31F | AIR      |
| 800 - 899   | 0x320 - 0x383 | GPS      |
| 900 - 999   | 0x384 - 0x3E7 | ...      |
| 1000 - 1099 | 0x3E8 - 0x44B | PSU      |
| 1100 - 1199 | 0x44C - 0x4AF | ...      |
| 1200 - 1299 | 0x4B0 - 0x513 | ...      |
| 1300 - 1399 | 0x514 - 0x577 | ...      |
| 1400 - 1499 | 0x578 - 0x5DB | ...      |
| 1500 - 1599 | 0x5DC - 0x63F | ...      |
| 1600 - 1699 | 0x640 - 0x6A3 | ...      |
| 1700 - 1799 | 0x6A4 - 0x707 | TELE/LOG |
| 1800 - 1899 | 0x708 - 0x76B | ...      |
| 1900 - 1999 | 0x76C - 0x7CF | LIGHTS   |
| 2000 - 2048 | 0x7D0 - 0x800 | AUX      |


#### More information

For more information about the actual messages send over the CAN protocol have a look at [can_ids.h](./include/twai_ids.h) and [can_meta.h](./include/twai_meta.h). There, you will find the actual IDs of the individual CAN messages, as well as meta information, such as the length of the data chunk send (i.e. the number of CAN messages) and more.

## Coding Guide

#### Whitespace
* Indent with tabs, not spaces.
* End file with a new line (as requested per C99 standard).
* Use empty line to divide code into logical chunks.
* Put spaces around binary operators : `x <= 2 && y > 10` is way easier to read than `x<=2&&y>10`.
* Put spaces after the following keywords : `if`, `switch`, `case`, `for`, `do`, `while`.

#### Placing braces
For this, we will follow the good old prophets of C : Kernighan and Ritchie. It also happens to be the rule of the Linux Kernel. The way the prophets taught us is to put the opening brace last on the line and the closing brace first :

```cpp
if (x == 42) {
    // do stuff
}
```

Note that the closing brace is always on its own line except if it is followed by a continuation of the same statement, for example in a `do..while` block or in an `if..else` block :

```cpp
if (x == 42) {
    // do stuff
} else if (x == 34) {
    // do something else
} else {
    // do yet another thingie
}
```

```cpp
do {
    // do stuff
} while (x < 42);
```

Function definitions look like this :

```cpp
void do_something_useful(void) {
    // Code goes here
}
```

Always use braces. This avoids mistakes and improves readability, without costing much of your time. Don't do this :

```cpp
if (x == 42)
    do_stuff();
```

But this :

```cpp
if (x == 42) {
    do_stuff();
}
```

#### Naming conventions
* Do *not* use CamelCaseNotation, use underscore_notation instead.
* Short variable names are ok if they are still understandable : `i` is ok for a loop counter, but `foo` for a function name isn't.
* If you *really* need a global variable, but it can live with file-only scope, declare it with the `static` keyword.

#### File Structure
* Put all globally needed function declarations, variable declarations and `#define`s in a `.h` file.
* Put all locally needed function declarations and `#define`s in a `.c` file with the same name.
* Put all function implementations in the `.c` file with the same name.  
* Use classic `#ifndef` include guards in header files (as seen below).
* This way the code interface and implementation are cleanly separated.

#### Interaction with C++
(Only applicable if using C and C++ together.)

To allow for easy inclusion of your headers in C++ files, you should put `extern "C"` in your header. If you don't do this, you will get weird link time error because of C++ name mangling.

For example:
```cpp
#ifndef MYHEADER_H_
#define MYHEADER_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Snip */

#ifdef __cplusplus
}
#endif
#endif
```
