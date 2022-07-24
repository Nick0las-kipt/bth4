#ifndef __WATCHDOG_H
#define __WATCHDOG_H

#ifdef __cplusplus
extern "C" {
#endif

int wdt_configure(void);
int wdt_prolongate(void);

#ifdef __cplusplus
}
#endif

#endif