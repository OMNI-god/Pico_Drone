#include <cstdio>
#include <cstdint>

#include "pico/stdlib.h"

#include "Elrs.h"
#include "CrsfParser.h"
#include "RCInput.h"

// ============================================================
// ELRS / CRSF configuration
// ============================================================

#define CRSF_UART uart1
#define CRSF_BAUDRATE 420000

#define UART_TX_PIN 4
#define UART_RX_PIN 5

// ============================================================
// CRSF configuration
// ============================================================

constexpr uint8_t CRSF_CHANNEL_COUNT = 16;

// ============================================================
// Print all 16 raw CRSF channels
// ============================================================

static void printRawChannels(const CrsfParser &crsf)
{
    const uint16_t *channels = crsf.channels();

    printf("\n");
    printf("------------------------------------------------------------\n");
    printf("                    CRSF RAW CHANNELS\n");
    printf("------------------------------------------------------------\n");

    for (uint8_t i = 0; i < CRSF_CHANNEL_COUNT; ++i)
    {
        printf(
            "CH%02d: %4u%s",
            i + 1,
            channels[i],
            ((i + 1) % 4 == 0) ? "\n" : " | ");
    }

    printf("------------------------------------------------------------\n");
}

// ============================================================
// Main
// ============================================================

int main()
{
    stdio_init_all();

    sleep_ms(2000);

    printf("\n");
    printf("============================================================\n");
    printf("                 ELRS RC INPUT TEST\n");
    printf("============================================================\n");

    printf("UART       : UART1\n");
    printf("Baudrate   : 420000\n");
    printf("Format     : 8N1\n");
    printf("TX         : GPIO %d\n", UART_TX_PIN);
    printf("RX         : GPIO %d\n", UART_RX_PIN);
    printf("Channels   : %d\n", CRSF_CHANNEL_COUNT);

    printf("============================================================\n\n");

    // ========================================================
    // Initialize ELRS UART
    // ========================================================

    Elrs elrs(
        CRSF_UART,
        CRSF_BAUDRATE,
        UART_RX_PIN,
        UART_TX_PIN);

    if (!elrs.initialize())
    {
        printf("ERROR: ELRS initialization failed!\n");

        while (true)
        {
            sleep_ms(1000);
        }
    }

    printf("ELRS UART initialized.\n");

    // ========================================================
    // CRSF parser
    // ========================================================

    CrsfParser crsf;

    printf("CRSF parser initialized.\n");

    // ========================================================
    // RC input processing
    // ========================================================

    RCInput rcInput;

    printf("RCInput initialized.\n");
    printf("Waiting for receiver data...\n\n");

    // ========================================================
    // Runtime variables
    // ========================================================

    uint8_t byte = 0;

    uint32_t lastPrintTime = 0;

    uint32_t lastFrameTime = 0;

    uint32_t frameCount = 0;

    // ========================================================
    // Main loop
    // ========================================================

    while (true)
    {
        // ----------------------------------------------------
        // Read all available UART bytes
        // ----------------------------------------------------

        while (elrs.available())
        {
            if (elrs.read(&byte, 1) == 1)
            {
                // --------------------------------------------
                // Feed byte into CRSF parser
                // --------------------------------------------

                if (crsf.processByte(byte))
                {
                    uint32_t now =
                        to_ms_since_boot(
                            get_absolute_time());

                    // ----------------------------------------
                    // Valid CRSF frame received
                    // ----------------------------------------

                    rcInput.update(
                        crsf.channels(),
                        now);

                    lastFrameTime = now;

                    frameCount++;
                }
            }
        }

        // ----------------------------------------------------
        // Current time
        // ----------------------------------------------------

        uint32_t now =
            to_ms_since_boot(
                get_absolute_time());

        // ====================================================
        // Print every 100 ms
        // ====================================================

        if (now - lastPrintTime >= 100)
        {
            lastPrintTime = now;

            // ------------------------------------------------
            // Raw CRSF channels
            // ------------------------------------------------

            const uint16_t *channels = crsf.channels();

            printf("\n");
            printf(
                "RAW: "
                "CH1=%4u "
                "CH2=%4u "
                "CH3=%4u "
                "CH4=%4u\n",

                channels[0],
                channels[1],
                channels[2],
                channels[3]);

            printf(
                "     "
                "CH5=%4u "
                "CH6=%4u "
                "CH7=%4u "
                "CH8=%4u\n",

                channels[4],
                channels[5],
                channels[6],
                channels[7]);

            printf(
                "     "
                "CH9=%4u "
                "CH10=%4u "
                "CH11=%4u "
                "CH12=%4u\n",

                channels[8],
                channels[9],
                channels[10],
                channels[11]);

            printf(
                "     "
                "CH13=%4u "
                "CH14=%4u "
                "CH15=%4u "
                "CH16=%4u\n",

                channels[12],
                channels[13],
                channels[14],
                channels[15]);

            // ------------------------------------------------
            // RC processed state
            // ------------------------------------------------

            /*
             * Do not use the old RCChannels type here.
             *
             * Your current RCInput implementation exposes
             * its state through its current API.
             */

            printf(
                "Frames: %lu | "
                "Last frame: %lu ms ago | "
                "Failsafe: %d\n",

                static_cast<unsigned long>(frameCount),

                static_cast<unsigned long>(
                    now - lastFrameTime),

                rcInput.isFailsafe());
        }

        // ----------------------------------------------------
        // Small CPU yield
        // ----------------------------------------------------

        sleep_us(100);
    }
}