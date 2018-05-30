/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
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
 * externalComputer.c - Connecting a external computer over UART
 */
#define DEBUG_MODULE "EXTPC"

#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "stm32fxxx.h"
#include "config.h"
#include "console.h"
#include "uart1.h"
#include "debug.h"
#include "deck.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "log.h"
#include "crc_bosch.h"
#include "quatcompress.h"
#include "commander.h"

static bool isInit;
static crc crcTable[256];

#define SEND_RATE 100
#define MAX_COMMAND_SIZE 64
static const uint8_t header[] = {'E', 'X', 'T', 'C'};

// TODO: this is hacky, but much more efficient than using the logging system...
extern state_t state;
extern setpoint_t setpoint;

static uint8_t cmdbuffer[MAX_COMMAND_SIZE];

enum EXTPCCOMMANDS
{
  EXTPC_SET_FULLSTATE_SETPOINT = 0,
};

struct fullStatePacket_s {
  int16_t x;         // position - mm
  int16_t y;
  int16_t z;
  int16_t vx;        // velocity - mm / sec
  int16_t vy;
  int16_t vz;
  int16_t ax;        // acceleration - mm / sec^2
  int16_t ay;
  int16_t az;
  int32_t quat;      // compressed quaternion, see quatcompress.h
  int16_t rateRoll;  // angular velocity - milliradians / sec
  int16_t ratePitch; //  (NOTE: limits to about 5 full circles per sec.
  int16_t rateYaw;   //   may not be enough for extremely aggressive flight.)
} __attribute__((packed));

static void send(const uint8_t* data, uint16_t size, crc* crcValue)
{
  uart1SendData(size, data);
  *crcValue = crcByByte(data, size, *crcValue, 0x0, crcTable);
}

// taking a copy ensures that the value isn't change between sending
// and checksum computation
static void sendFloat(float value, crc* crcValue)
{
  send((const uint8_t*)&value, sizeof(float), crcValue);
}

// static uint8_t recvUint8(crc* crcValue)
// {
//   uint8_t result;
//   uart1Getchar((char*)&result);
//   *crcValue = crcByByte((const uint8_t*)&result, sizeof(uint8_t), *crcValue, 0x0, crcTable);
//   return result;
// }

static uint16_t recvUint16(crc* crcValue)
{
  uint16_t result;
  uart1Getchar(((char*)&result) + 0);
  uart1Getchar(((char*)&result) + 1);
  *crcValue = crcByByte((const uint8_t*)&result, sizeof(uint16_t), *crcValue, 0x0, crcTable);
  return result;
}

void externalComputerSendTask(void *param)
{
  TickType_t lastWakeTime = xTaskGetTickCount();
  while (1) {
    vTaskDelayUntil(&lastWakeTime, F2T(SEND_RATE));

    crc crcValue = INITIAL_REMAINDER;
    // send(header, sizeof(header), &crcValue);
    uart1SendData(sizeof(header), (const uint8_t*)header);

    uint16_t length = 4 + 12 * sizeof(float);//4*sizeof(float);
    send((const uint8_t*)&length, sizeof(length), &crcValue);

    send((const uint8_t*)&lastWakeTime, sizeof(TickType_t), &crcValue);

    sendFloat(state.position.x, &crcValue);
    sendFloat(state.position.y, &crcValue);
    sendFloat(state.position.z, &crcValue);

    sendFloat(state.velocity.x, &crcValue);
    sendFloat(state.velocity.y, &crcValue);
    sendFloat(state.velocity.z, &crcValue);

    sendFloat(state.acc.x, &crcValue);
    sendFloat(state.acc.y, &crcValue);
    sendFloat(state.acc.z, &crcValue);

    // sendFloat(state.attitudeQuaternion.x, &crcValue);
    // sendFloat(state.attitudeQuaternion.y, &crcValue);
    // sendFloat(state.attitudeQuaternion.z, &crcValue);
    // sendFloat(state.attitudeQuaternion.w, &crcValue);

    sendFloat(setpoint.position.x, &crcValue);
    sendFloat(setpoint.position.y, &crcValue);
    sendFloat(setpoint.position.z, &crcValue);

    /* negate crc value */
    crcValue = ~(crcValue^FINAL_XOR_VALUE);
    uart1SendData(sizeof(crcValue), (const uint8_t*)&crcValue);

  }
}

void externalComputerReceiveTask(void *param)
{
  static uint8_t ringbuffer[sizeof(header)];
  static setpoint_t setpoint;

  uint8_t ringbufferIdx = 0;

  while (1) {
    uart1Getchar((char*)&ringbuffer[ringbufferIdx]);
    // DEBUG_PRINT("Recv: %d\n", ringbuffer[ringbufferIdx]);
    ++ringbufferIdx;
    bool headerMatch = true;
    for (int8_t i = 0; i < sizeof(header); ++i) {
      // DEBUG_PRINT("chk: %d %d %d\n", i, ringbuffer[(ringbufferIdx + i) % sizeof(header)], header[i]);
      if (ringbuffer[(ringbufferIdx + i) % sizeof(header)] != header[i]) {
        headerMatch = false;
        break;
      }
    }

    if (headerMatch) {
      crc crcValue = INITIAL_REMAINDER;

      uint16_t length = recvUint16(&crcValue);
      // DEBUG_PRINT("Match: %d\n", length);
      if (length <= sizeof(cmdbuffer)) {
        for (int i = 0; i < length; ++i) {
          uart1Getchar((char*)&cmdbuffer[i]);
        }
        crcValue = crcByByte((const uint8_t*)&cmdbuffer, length, crcValue, 0x0, crcTable);
        crcValue = ~(crcValue^FINAL_XOR_VALUE);

        uint32_t crcMsg;
        uart1Getchar(((char*)&crcMsg) + 0);
        uart1Getchar(((char*)&crcMsg) + 1);
        uart1Getchar(((char*)&crcMsg) + 2);
        uart1Getchar(((char*)&crcMsg) + 3);

        if (crcValue == crcMsg) {
          // DEBUG_PRINT("CRC OK.\n");

          uint8_t cmd = cmdbuffer[0];
          if (   cmd == EXTPC_SET_FULLSTATE_SETPOINT
              && length == sizeof(struct fullStatePacket_s) + 1) {
            const struct fullStatePacket_s* packet = (const struct fullStatePacket_s*)&cmdbuffer[1];

            setpoint.mode.x = modeAbs;
            setpoint.position.x = packet->x / 1000.0f;
            setpoint.velocity.x = packet->vx / 1000.0f;
            setpoint.acceleration.x = packet->ax / 1000.0f;

            setpoint.mode.y = modeAbs;
            setpoint.position.y = packet->y / 1000.0f;
            setpoint.velocity.y = packet->vy / 1000.0f;
            setpoint.acceleration.y = packet->ay / 1000.0f;

            setpoint.mode.z = modeAbs;
            setpoint.position.z = packet->z / 1000.0f;
            setpoint.velocity.z = packet->vz / 1000.0f;
            setpoint.acceleration.z = packet->az / 1000.0f;

            float const millirad2deg = 180.0f / ((float)M_PI * 1000.0f);
            setpoint.attitudeRate.roll = millirad2deg * packet->rateRoll;
            setpoint.attitudeRate.pitch = millirad2deg * packet->ratePitch;
            setpoint.attitudeRate.yaw = millirad2deg * packet->rateYaw;

            quatdecompress(packet->quat, (float *)&setpoint.attitudeQuaternion.q0);
            setpoint.mode.quat = modeAbs;
            setpoint.mode.roll = modeDisable;
            setpoint.mode.pitch = modeDisable;
            setpoint.mode.yaw = modeDisable;

            commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_CRTP);

          } else {
            DEBUG_PRINT("UNKNOWN CMD.\n");
          }


        } else {
          DEBUG_PRINT("CRC FAIL. %lu %lu\n", crcMsg, crcValue);
        }

      } else {
        DEBUG_PRINT("CMD too long!\n");
      }
    }
  }
}

static void externalComputerInit(DeckInfo *info)
{
  if(isInit)
    return;

  uart1Init(115200);

  // TODO: it seems stupid that there can be multiple tables per firmware...
  crcTableInit(crcTable);

  xTaskCreate(externalComputerSendTask, "EXTCSend",
              configMINIMAL_STACK_SIZE, NULL, /*priority*/0, NULL);
  xTaskCreate(externalComputerReceiveTask, "EXTCRecv",
              configMINIMAL_STACK_SIZE, NULL, /*priority*/0, NULL);

  isInit = true;
}

static bool externalComputerTest()
{
  bool status = true;

  if(!isInit)
    return false;

  return status;
}

static const DeckDriver externalComputer_deck = {
  // .vid = 0xBC,
  // .pid = 0x07,
  .name = "alEXTC",

  .usedPeriph = DECK_USING_UART1,
  .usedGpio = DECK_USING_RX1 | DECK_USING_TX1,

  .init = externalComputerInit,
  .test = externalComputerTest,
};

DECK_DRIVER(externalComputer_deck);
