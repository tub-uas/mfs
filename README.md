# Modular Flight System (MFS)

## CAN IDs

#### General

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


## Coding Guide

#### Whitespace
* Indent with tabs, not spaces.
* End file with a new line (as requested per C99 standard).
* Use empty line to divide code into logical chunks.
* Put spaces around binary operators : `x <= 2 && y > 10` is way easier to read than `x<=2&&y>10`.
* Put spaces after the following keywords : `if`, `switch`, `case`, `for`, `do`, `while`.

#### Placing braces
For this, we will follow the good old prophets of C : Kernighan and Ritchie.
It also happens to be the rule of the Linux Kernel.
The way the prophets taught us is to put the opening brace last on the line and the closing brace first :

```cpp
if (x == 42) {
    // do stuff
}
```

Note that the closing brace is always on its own line except if it is followed by a continuation
of the same statement, for example in a `do..while` block or in an `if..else` block :

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

Always use braces.
This avoids mistakes and improves readability, without costing much of your time.
Don't do this :

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

To allow for easy inclusion of your headers in C++ files, you should put `extern "C"` in your header.
If you don't do this, you will get weird link time error because of C++ name mangling.

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
