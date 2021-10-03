/* Copyright (C) 2021 Aliaksei Katovich. All rights reserved.
 *
 * This source code is licensed under the BSD Zero Clause License found in
 * the 0BSD file in the root directory of this source tree.
 */

#ifndef DECODE64_H
#define DECODE64_H

#include <stdint.h>
#include <arpa/inet.h>

static inline uint8_t isle(void)
{
	uint16_t n = 1;
	return !!*((char *)&n);
}

static float decode64(uint8_t pos, uint8_t size, float scale, float offset,
 uint8_t lsb, uint8_t sign, uint64_t data)
{
	int64_t tmp;
	int8_t shift;

	tmp = (uint64_t) htonl(data) << 32 | htonl(data >> 32);

	if (lsb)
		shift = pos;
	else
		shift = 64 - ((pos >> 3) * 8) - size + pos % 8 - 7;

	if (shift < 0)
		return 0./0.;

	tmp = (tmp >> shift) & ((1 << size) - 1);
	if (sign && (tmp >> (size - 1)))
		tmp -= (1 << size);

	if (lsb && size > 8 && size <= 16)
		tmp = htons(tmp);
	else if (lsb && size > 16 && size <= 32)
		tmp = htonl(tmp);

	return tmp * scale + offset;
}

#endif /* DECODE64_H */
