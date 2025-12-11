# Micro Blackdeath --- Firmware for the "Black Death" Noise Synth

## Short Description

This repository contains the **microbdinterp** firmware for the Black
Death noise synth.\
The firmware provides several instruction-based CPU/Plague algorithms
(including **Brainfuck, SIR, Redcode, Biota**) as well as filter
modulation using a **MAX7400-like filter**.\
It reads ADC inputs (pots/samples), writes PWM/output values, and offers
various "Plague" algorithms for sound generation.

## Important Files

-   **`microbdinterp.c`** --- Main firmware source (ADC, instruction
    sets, plague algorithms, main).
-   **`Makefile`** --- Build and flash rules (avr-gcc, avr-objcopy,
    avrdude).

## Hardware / Connections (USBasp ↔ Board)

If you use a `usbasp` programmer, connect the pins as follows:

```
  Signal             Programmer Pins   Board Pin   10-Pin Adapter Cable
  ------------------ ----------------- ----------- ----------------------
  **GND**            4, 6, 8, 10       1           1--2
  **SCK**            7                 2           3--4
  **MISO (PDO)**     9                 3           5--6
  **MOSI (PDI)**     1                 4           7--8
  **Reset**          5                 5           9--10
  **VCC (5V/3V3)**   2                 --          --
````

> **Note:** VCC is usually not supplied through the adapter cable unless
> you explicitly want to power the target controller.\
> Ensure correct operating voltage (5V vs. 3.3V).

## Required Tools (Installation)

-   `avr-gcc` and `avr-libc`
-   `avr-objcopy`
-   `avrdude`

### macOS Installation Example

``` bash
brew tap osx-cross/avr
brew install avr-gcc avrdude
```

## Build & Flash

``` bash
make            # builds microbdinterp.hex
make flash      # builds and flashes the hex (requires connected usbasp)
make fuse       # writes HFUSE/LFUSE as defined in the Makefile (use with care!)
```

## Makefile Explanation

-   Default MCU:

    ``` make
    MCU = atmega168
    DEVICE = m168
    ```

-   `make fuse` writes fuse bytes.\

-   `make flash` uses `avrdude -c usbasp -p m168`.

## Configuration / Build Options in the Code

-   `F_CPU = 16000000UL`
-   `MAX_SAM = 255`
-   Direct register access for ADC, PWM (`OCR0A`, `OCR1A`), and timers.

## Operation / Control (via Hardware)

```
  Control      Function
  ------------ -----------------------------------------------------
  **Left**     CPU step / instruction-set selection
  **Center**   Hardware / filter parameters
  **Right**    Plague step / process selection and filter modifier
```

## Key Concepts in the Code

### Instruction Sets

-   `instructionsetfirst`
-   `instructionsetplague`
-   `instructionsetbf` (Brainfuck)
-   `instructionsetSIR`
-   `instructionsetredcode`
-   `instructionsetbiota`

### Filter Modulation

`filtermod[]` operations on `OCR1A`:

-   left shift\
-   right shift\
-   multiply\
-   divide

### Plague / Cellular Algorithms

-   `hodge()`
-   `cel()`
-   `SIR()`
-   `life()`

### ADC Handling

-   `adc_init()`
-   `adcread(channel)` --- 8‑bit left-adjusted

## Safety & Notes

-   Ensure correct board voltage (5V / 3.3V).\
-   Fuse changes may brick the controller.\
-   Low-level register operations: test carefully.

## Development / Modification

-   Edit `microbdinterp.c`
-   Compile with `make`
-   Commented areas show modification points.

## Credits & License

-   Based on "The Plague Interpreter" / Microresearch.\
-   Modifications by Circuitnoise + contributors.\
-   No formal license included.

## Support / Contact

-   Check build errors and avrdude connectivity.\
-   Contact maintainer for deeper support.

## File

    microbdinterp.c
