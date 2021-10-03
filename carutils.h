#ifndef CARUTILS_H
#define CARUTILS_H

#include <stdint.h>

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#endif

struct can_signal {
  const char *name;
  const char *units;
  float (*decode)(uint64_t data);
};

struct can_object {
  uint32_t id;
  const char *name;
  const struct can_signal *signals;
  uint16_t sigcnt;
};

#error "CAR SPECIFIC HEADER SHOULD BE INCLUDED HERE"

#endif /* CARUTILS_H */
