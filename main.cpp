
#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>

// ======================================================
// PINOUT: TTGO LoRa32 T3 v1.6.1, SX1276
// ======================================================

#define BUTTON_PIN 13

#define LORA_SCK   5U
#define LORA_MISO  19U
#define LORA_MOSI  27U
#define LORA_CS    18U
#define LORA_RST   23U
#define LORA_DIO0  26U
#define LORA_DIO1  33U

#define LORA_FREQUENCY 915.0

// ======================================================
// BUTTON SETTINGS
// ======================================================

const uint32_t DEBOUNCE_MS = 30;
const uint32_t DOUBLE_CLICK_MS = 400;

// ======================================================
// BUTTON EVENTS
// ======================================================

enum ButtonEvent : uint8_t {
    BUTTON_SINGLE = 1,
    BUTTON_DOUBLE = 2
};

// SPI bus and LoRa radio
SPIClass loraSPI(VSPI);

SX1276 radio = new Module(
    LORA_CS,
    LORA_DIO0,
    LORA_RST,
    LORA_DIO1,
    loraSPI
);

// Queues
QueueHandle_t buttonQueue = nullptr;
QueueHandle_t radioQueue = nullptr;

// ======================================================
// BUTTON TASK
// Reads the button and detects single/double clicks.
// Sends the result to buttonQueue.
// ======================================================

void buttonTask(void *parameter) {
    Serial.println("[ButtonTask] Started");

    int rawState = digitalRead(BUTTON_PIN);
    int stableState = rawState;

    uint32_t lastDebounceTime = millis();

    bool clickPending = false;
    uint32_t firstReleaseTime = 0;

    while (true) {
        int reading = digitalRead(BUTTON_PIN);
        uint32_t now = millis();

        // Detect a possible change in the physical button state
        if (reading != rawState) {
            rawState = reading;
            lastDebounceTime = now;
        }

        // Accept the change only after the debounce interval
        if ((now - lastDebounceTime >= DEBOUNCE_MS) &&
            (rawState != stableState)) {

            stableState = rawState;

            // HIGH means the button has been released
            if (stableState == HIGH) {
                now = millis();

                if (clickPending &&
                    (now - firstReleaseTime <= DOUBLE_CLICK_MS)) {

                    // Second click detected
                    ButtonEvent event = BUTTON_DOUBLE;

                    if (xQueueSend(buttonQueue, &event, 0) == pdPASS) {
                        Serial.println(
                            "[ButtonTask] Double click -> Queue"
                        );
                    } else {
                        Serial.println(
                            "[ButtonTask] ERROR: buttonQueue full"
                        );
                    }

                    clickPending = false;
                } else {
                    // If an earlier click expired, report it as single
                    if (clickPending) {
                        ButtonEvent event = BUTTON_SINGLE;

                        if (xQueueSend(buttonQueue, &event, 0) == pdPASS) {
                            Serial.println(
                                "[ButtonTask] Single click -> Queue"
                            );
                        }
                    }

                    // Start waiting for a possible second click
                    clickPending = true;
                    firstReleaseTime = now;
                }
            }
        }

        // If no second click arrived in time, report a single click
        now = millis();

        if (clickPending &&
            (now - firstReleaseTime > DOUBLE_CLICK_MS)) {

            ButtonEvent event = BUTTON_SINGLE;

            if (xQueueSend(buttonQueue, &event, 0) == pdPASS) {
                Serial.println(
                    "[ButtonTask] Single click -> Queue"
                );
            } else {
                Serial.println(
                    "[ButtonTask] ERROR: buttonQueue full"
                );
            }

            clickPending = false;
        }

        // Yield CPU to other FreeRTOS tasks
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ======================================================
// RADIO TASK
// Waits for commands in radioQueue and transmits packets.
// ======================================================

void radioTask(void *parameter) {
    Serial.println("[RadioTask] Started");

    ButtonEvent command;

    while (true) {
        // Wait here until the Main Loop sends a command
        if (xQueueReceive(
                radioQueue,
                &command,
                portMAX_DELAY
            ) == pdPASS) {

            String packet;

            if (command == BUTTON_SINGLE) {
                packet = "BUTTON SINGLE";
            } else if (command == BUTTON_DOUBLE) {
                packet = "BUTTON DOUBLE";
            } else {
                continue;
            }

            Serial.print("[RadioTask] Transmitting: ");
            Serial.println(packet);

            // Radio transmission happens only in this task
            int state = radio.transmit(packet);

            if (state == RADIOLIB_ERR_NONE) {
                Serial.println("[RadioTask] Transmission successful");
            } else {
                Serial.print("[RadioTask] Transmission failed, code ");
                Serial.println(state);
            }
        }
    }
}

// ======================================================
// SETUP
// Initializes the button, radio, queues and tasks.
// ======================================================

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("=== FreeRTOS Button + LoRa TX ===");

    // Button connects between GPIO32 and GND
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    // Create queues before starting tasks
    buttonQueue = xQueueCreate(5, sizeof(ButtonEvent));
    radioQueue = xQueueCreate(5, sizeof(ButtonEvent));

    if (buttonQueue == nullptr || radioQueue == nullptr) {
        Serial.println("[ERROR] Failed to create queues");

        while (true) {
            delay(1000);
        }
    }

    Serial.println("[Setup] Queues created");

    // Initialize SPI for the SX1276 radio
    loraSPI.begin(
        LORA_SCK,
        LORA_MISO,
        LORA_MOSI,
        LORA_CS
    );

    Serial.println("[Setup] Initializing LoRa...");

    int state = radio.begin(LORA_FREQUENCY);

    if (state != RADIOLIB_ERR_NONE) {
        Serial.print("[ERROR] LoRa initialization failed, code ");
        Serial.println(state);

        while (true) {
            delay(1000);
        }
    }

    Serial.println("[Setup] LoRa initialized");

    // Button task has higher priority to keep button polling responsive
    BaseType_t buttonCreated = xTaskCreate(
        buttonTask,
        "ButtonTask",
        3072,
        nullptr,
        2,
        nullptr
    );

    // Radio task waits on its queue; transmission cannot block ButtonTask
    BaseType_t radioCreated = xTaskCreate(
        radioTask,
        "RadioTask",
        4096,
        nullptr,
        1,
        nullptr
    );

    if (buttonCreated != pdPASS || radioCreated != pdPASS) {
        Serial.println("[ERROR] Failed to create FreeRTOS task");

        while (true) {
            delay(1000);
        }
    }

    Serial.println("[Setup] Tasks created");
    Serial.println("[Setup] Press the button to transmit");
}

// ======================================================
// MAIN LOOP
// Receives button events and forwards them to radioQueue.
// No button detection or radio transmission is done here.
// ======================================================

void loop() {
    ButtonEvent event;

    // Block until ButtonTask places an event in buttonQueue
    if (xQueueReceive(
            buttonQueue,
            &event,
            portMAX_DELAY
        ) == pdPASS) {

        if (event == BUTTON_SINGLE) {
            Serial.println("[MainLoop] Received SINGLE event");
        } else if (event == BUTTON_DOUBLE) {
            Serial.println("[MainLoop] Received DOUBLE event");
        }

        // Forward the event to RadioTask
        if (xQueueSend(
                radioQueue,
                &event,
                pdMS_TO_TICKS(100)
            ) == pdPASS) {

            Serial.println("[MainLoop] Command sent to RadioTask");
        } else {
            Serial.println("[MainLoop] ERROR: radioQueue full");
        }
    }
}
