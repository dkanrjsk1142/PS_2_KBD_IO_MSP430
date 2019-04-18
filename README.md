<BS># PS_2_KBD_IO_MSP430<br>

PS/2 Receiver & bluetooth keyboard controller driver

PS/2 keyboard : Cherry MX-G8113
MCU           : MSP430FR4133(launchpad)

cherry keyboard's PS/2 connection can be switched directly connect to PC by PS/2 or bluetoothed connect to PC by bluetooth keyboard controller.

cherry keyboard -(PS/2)---- PC
                        |
                        | <- switching by FET(drived by MCU)
                        |
                        --- MCU -- bluetooth keyboard controller -(bluetooth)-- PC
