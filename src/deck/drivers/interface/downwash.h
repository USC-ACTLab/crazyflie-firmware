/**
 * @file downwash.h
 * @brief Downwash deck driver
 * @details Downwash deck driver: expansion deck for detecting properller downwash in a stacked flight formation
 * @author Will Wu, ACT Lab @ Brown University
 * @date 2025-04-29
 */


#ifndef _DOWNWASH_H_
#define _DOWNWASH_H_

#include "deck_core.h"

void downwashDeckInit(DeckInfo* info);

bool downwashDeckTest(void);
void downwashDeckTask(void* arg);

#endif // _DOWNWASH_H_