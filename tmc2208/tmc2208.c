#include "tmc2208.h"

#include <math.h>

static uint8_t tmc_uart_calc_crc(const uint8_t *datagram, const uint32_t len)
{
    uint8_t crc = 0;

    for (uint32_t i = 0; i < (len - 1); i++)
    {
        uint8_t current = datagram[i];

        for (uint32_t j = 0; j < 8; j++)
        {
            if ((crc >> 7) ^ (current & 0x01))
            {
                crc = (crc << 1) ^ 0x07;
            }
            else
            {
                crc = (crc << 1);
            }
            current = current >> 1;
        }
    }

    return crc;
}

static tmc_error_t tmc_write_datagram(const tmc_write_datagram_t *tx)
{
    return tmc_uart_tx((const uint8_t *)tx, (uint32_t)sizeof(tmc_write_datagram_t));
}

static tmc_error_t tmc_read_datagram(const tmc_read_datagram_t *tx, tmc_write_datagram_t *rx)
{
    tmc_error_t error = tmc_uart_tx((const uint8_t *)tx, (uint32_t)sizeof(tmc_read_datagram_t));
    if (error != TMC_ERROR_OK)
    {
        return error;
    }

    // Get rid of the bytes we read from the TX
    tmc_uart_rx((uint8_t *)rx, (uint32_t)sizeof(tmc_read_datagram_t), TMC_UART_TIMEOUT_MS);

    // Now, get the real RXd ones
    return tmc_uart_rx((uint8_t *)rx, (uint32_t)sizeof(tmc_write_datagram_t), TMC_UART_TIMEOUT_MS);
}

static tmc_error_t tmc_write_reg(const uint8_t node, const uint8_t addr, const uint32_t value)
{
    // Init the datagram
    tmc_write_datagram_t tx = {
        .sync       = TMC_SYNC_WORD,
        .slave      = node,
        .addr.idx   = addr,
        .addr.write = TMC_WRITE_BIT,
        .payload    = value,
    };

    // Swap the edianess of the payload for the controller
    tx.payload = __builtin_bswap32(tx.payload);

    // Calc the CRC
    tx.crc =
        tmc_uart_calc_crc((const uint8_t *)&tx, (uint32_t)(uint32_t)sizeof(tmc_write_datagram_t));

    // Send on the UART
    return tmc_write_datagram((const tmc_write_datagram_t *)&tx);
}

static tmc_error_t tmc_read_reg(const uint8_t node, const uint8_t addr, uint32_t *value)
{
    // Init the datagram
    tmc_read_datagram_t tx = {
        .sync       = TMC_SYNC_WORD,
        .slave      = node,
        .addr.idx   = addr,
        .addr.write = TMC_READ_BIT,
    };

    // Calc the CRC
    tx.crc =
        tmc_uart_calc_crc((const uint8_t *)&tx, (uint32_t)(uint32_t)sizeof(tmc_read_datagram_t));

    // Send on the UART
    tmc_write_datagram_t rx    = {0};
    tmc_error_t          error = tmc_read_datagram((const tmc_read_datagram_t *)&tx, &rx);
    if (error != TMC_ERROR_OK)
    {
        return error;
    }

    // is the CRC ok?
    uint8_t crc =
        tmc_uart_calc_crc((const uint8_t *)&rx, (uint32_t)(uint32_t)sizeof(tmc_write_datagram_t));
    if (crc != rx.crc)
    {
        return TMC_ERROR_CRC;
    }

    // Is what we got for us?
    if ((rx.slave != 0xFF) || (rx.addr.idx != tx.addr.idx))
    {
        return TMC_ERROR_INVALID_RX;
    }

    // Let's get the data...
    *value = __builtin_bswap32(rx.payload);

    // now we can fuck off
    return TMC_ERROR_OK;
}

#include <stdio.h>

tmc_error_t tmc_init(const tmc_microstepping_t steps_per_notch, const uint32_t mtr_steps_per_turn)
{
    (void)steps_per_notch;
    (void)mtr_steps_per_turn;

    // Set defaults

    uint32_t    value = 0;
    tmc_error_t error = tmc_read_reg(0x00, 0x01, &value);
    printf("error = %u, value = %08lX\n", error, value);

    return TMC_ERROR_OK;
}

tmc_error_t tmc_start(const float turns_per_second)
{
    (void)turns_per_second;

    // // Make sure the microstepping is correct
    // tmc_set_microstepping(tmc_microstepping);

    // // Set the direction
    // tmc_set_dir((speed_rpm > 0) ? true : false);

    // // Calculate the output PWM freuqency
    // uint32_t frequency =
    //     (uint32_t)(fabs(speed_rpm) * tmc_steps_per_rev * pow(2, tmc_microstepping) / 60.0);

    // // Set the output frequency
    // tmc_set_stp(frequency);

    // // Enable the driver (it's inverted)
    // tmc_set_enable(false);

    return TMC_ERROR_OK;
}

tmc_error_t tmc_stop(void)
{
    // // Disable the driver (it's inverted)
    // tmc_set_enable(true);

    return TMC_ERROR_OK;
}

tmc_error_t tmc_set_microstepping(const tmc_microstepping_t steps_per_notch)
{
    (void)steps_per_notch;
    return TMC_ERROR_OK;
}
