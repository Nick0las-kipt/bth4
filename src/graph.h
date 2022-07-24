#ifndef GRAPH_H
#define GRAPH_H

#include <zephyr.h>
#include <kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

int graphProcessData(int pres, int temp, int rh);
bool drawGraphFast(bool force);

bool graphHistKeys(int key, int count);
bool drawGraphHist(bool force);
bool graphFastKeys(int key, int count);

#ifdef __cplusplus
}
#endif
#endif