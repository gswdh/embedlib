# peripherals

Standardised MCU peripheral interfaces: `gpio.h`, `uart.h`, `spi.h`, `i2c.h`, `can.h`.

These are **headers only**. There is no `.c` file here and there never will be — each
target supplies its own implementation (STM32 HAL, ESP-IDF, a register-level port, a
host stub for unit tests) and the device drivers elsewhere in this repository are
written against these prototypes alone. No header in this directory includes a vendor
header, so including one does not pull a silicon vendor into a portable driver.

## The shared contract

Every header follows the same rules, so knowing one is close to knowing all five.

**Errors.** Each peripheral has its own `<mod>_error_t` enum. `<MOD>_OK` is zero and
every failure is non-zero and distinct, so a caller can act on the specific failure
without the implementation printing anything. This matches the driver convention already
used in this repository (`stusb4500_error_t`, `ssd1309z_error_t`, and so on).

**Instances.** Hardware is named by a board-defined index — `uart_id_t`, `spi_id_t`,
`i2c_id_t`, `can_bus_t`, and `gpio_pin_t` for pins — never by a vendor handle or a
`(port, mask)` pair. The board publishes its indices as named constants and owns the
mapping to physical hardware:

```c
#define BOARD_UART_DEBUG  (0U)
#define BOARD_I2C_SENSORS (0U)
#define BOARD_CS_ADXL355  (0U)
#define BOARD_LED         (3U)
```

Driver and application code then names a role, not a peripheral, and moving a signal to
a different port is a board-file edit.

**Signatures.** Value parameters are `const`-qualified; outputs are trailing pointers.
Anything that can fail returns `<mod>_error_t`, including reads — hence
`gpio_read(pin, &state)` rather than a bare `bool`.

**Timeouts.** Always milliseconds. `<MOD>_TIMEOUT_NONE` (0) polls once without blocking;
`<MOD>_TIMEOUT_FOREVER` waits indefinitely.

**Blocking and background.** Blocking calls return when the transfer completes or the
deadline passes. The `_async` variants hand off to interrupt or DMA and report
completion through a callback; the caller's buffers are **not** copied and must stay
valid until the callback fires.

**Callbacks** run in interrupt context. Keep them short, do not block, and do not call a
blocking API from inside one.

**Optional features.** An implementation that cannot offer something returns
`<MOD>_ERR_UNSUPPORTED`. It never pretends to honour a request it silently ignored.

## Per-peripheral notes

- **gpio** — Pins are board-defined indices. Edge interrupts are optional as a group.
- **uart** — `uart_rx_start()` receives continuously in the background; that, not
  repeated `uart_read()`, is how a framed protocol should be received.
- **spi** — Master only. Bus (`spi_id_t`) and chip select (`spi_cs_t`) are separate, so
  one bus carries several devices without the device driver knowing the select GPIO.
  Pass `SPI_CS_NONE` between `spi_select()`/`spi_deselect()` to hold a select across
  several transfers.
- **i2c** — Addresses are always 7-bit right-aligned (`0x28`, never the shifted `0x50`).
  `i2c_mem_read()`/`i2c_mem_write()` do the pointer write and data transfer as one
  transaction with a repeated START; a separate write then read is not equivalent.
  `i2c_recover()` clocks out a slave holding SDA low after a mid-read reset.
- **can** — The controller is `can_bus_t`, not `can_id_t`, because "id" on CAN means the
  arbitration identifier. Bring-up is two-stage (`can_init()`, filters, `can_start()`)
  because acceptance filters can only be programmed off the bus. Classic and FD share
  the interface; frames carry their own format flags. Define `CAN_MAX_PAYLOAD` as 8
  before including for a classic-only build to shrink `can_frame_t` by 56 bytes.

## Implementing a target

Create the implementation outside this directory — the interfaces stay platform-clean:

```
peripherals/i2c.h            <- this interface
targets/stm32g0/i2c.c        <- one implementation
targets/host/i2c.c           <- another, for tests
```

An implementation need not cover everything. Return `<MOD>_ERR_UNSUPPORTED` from what
the hardware cannot do and the interface stays honest.
