/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * gpio_adc_bridge.c - App layer application to send output on a GPIO and read ADC values, sending the data to the cfclient.
 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"

#include "log.h"
#include "param.h"
#include "deck_constants.h"
#include "deck_digital.h"

#define DEBUG_MODULE "GPIO2ADC"

#define LATERAL_OUT DECK_GPIO_IO1
#define VERTICAL_OUT DECK_GPIO_IO2

#define LATERAL_IN DECK_GPIO_IO3
#define VERTICAL_IN DECK_GPIO_IO4

#define POLL_PERIOD_MS 10
#define DEBOUNCE_SAMPLES 5 // state must be stable this many polls (50 ms) before it is accepted

typedef struct
{
  const char *name;
  deckPin_t pin;
  uint8_t connected;
  uint8_t candidate;
  uint8_t stableCount;
} connection_t;

static connection_t lateral = {.name = "Lateral"};
static connection_t vertical = {.name = "Vertical"};

static void updateConnection(connection_t *conn)
{
  uint8_t raw = (digitalRead(conn->pin) == HIGH) ? 1 : 0;

  if (raw != conn->candidate)
  {
    conn->candidate = raw;
    conn->stableCount = 0;
    return;
  }

  if (conn->stableCount < DEBOUNCE_SAMPLES)
  {
    conn->stableCount++;
    return;
  }

  if (conn->candidate != conn->connected)
  {
    conn->connected = conn->candidate;
    DEBUG_PRINT("%s %s\n", conn->name, conn->connected ? "connected" : "disconnected");
  }
}

void appMain()
{
  lateral.pin = LATERAL_IN;
  vertical.pin = VERTICAL_IN;

  pinMode(LATERAL_OUT, OUTPUT);
  pinMode(VERTICAL_OUT, OUTPUT);
  digitalWrite(LATERAL_OUT, HIGH);
  digitalWrite(VERTICAL_OUT, HIGH);

  pinMode(LATERAL_IN, INPUT_PULLDOWN);
  pinMode(VERTICAL_IN, INPUT_PULLDOWN);

  while (1)
  {
    vTaskDelay(M2T(POLL_PERIOD_MS));

    updateConnection(&lateral);
    updateConnection(&vertical);
  }
}

LOG_GROUP_START(aerolink)
LOG_ADD(LOG_UINT8, lateral, &lateral.connected)
LOG_ADD(LOG_UINT8, vertical, &vertical.connected)
LOG_GROUP_STOP(aerolink)
