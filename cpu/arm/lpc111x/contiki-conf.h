/*
 * Copyright (c) 2005, Swedish Institute of Computer Science
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 * @(#)$Id: contiki-conf.h,v 1.12 2009/09/09 21:11:24 adamdunkels Exp $
 */

#ifndef __CONTIKI_CONF_H__
#define __CONTIKI_CONF_H__

//#include <inttypes.h>
#include <stdint.h>
#include "driver_config.h"
//define platform for hal.h

#define CC_CONF_REGISTER_ARGS          1
#define CC_CONF_FUNCTION_POINTER_ARGS  1
#define CC_CONF_FASTCALL
#define CC_CONF_VA_ARGS                1
/*#define CC_CONF_INLINE                 inline*/

typedef uint8_t   u8_t;
typedef uint16_t u16_t;
typedef uint32_t u32_t;
typedef  int32_t s32_t;

#define CCIF
#define CLIF

typedef unsigned short uip_stats_t;

#define UIP_CONF_UDP             1
#define UIP_CONF_MAX_CONNECTIONS 8
#define UIP_CONF_MAX_LISTENPORTS 8
#define UIP_CONF_BUFFER_SIZE     100
#define UIP_CONF_BYTE_ORDER      UIP_LITTLE_ENDIAN
#define UIP_CONF_TCP       1
#define UIP_CONF_TCP_SPLIT       0
#define UIP_CONF_LOGGING         1
#define UIP_CONF_UDP_CHECKSUMS   1
#define UIP_CONF_IP_FORWARD      1

#if UIP_CONF_IPV6
#define UIP_CONF_IPV6_CHECKS     1
#define UIP_CONF_IPV6_QUEUE_PKT  1
#define UIP_CONF_IPV6_REASSEMBLY 0
#define UIP_CONF_NETIF_MAX_ADDRESSES  3
#define UIP_CONF_ND6_MAX_PREFIXES     3
#define UIP_CONF_ND6_MAX_NEIGHBORS    4
#define UIP_CONF_ND6_MAX_DEFROUTERS   2
#define UIP_CONF_ICMP6           1
#endif /* UIP_CONF_ICMP6 */

#define MMEM_CONF_SIZE 1

typedef unsigned long clock_time_t;

#define CLOCK_CONF_SECOND 1000

#define LOG_CONF_ENABLED 1

#define SMALLSIGNALS 1

#define NETSTACK_CONF_RADIO       rf230_driver
#define NETSTACK_CONF_NETWORK     rime_driver
#define NETSTACK_CONF_MAC         nullmac_driver
#define NETSTACK_CONF_FRAMER      framer_802154

#define CHANNEL_802_15_4          26
/* The radio needs to interrupt during an rtimer interrupt */
#define RTIMER_CONF_NESTED_INTERRUPTS 1
#define RF230_CONF_AUTOACK        0
#define RF230_CONF_AUTORETRIES    0
#define RF230_CONF_CSMARETRIES    1
#define RF230_CONF_RX_BUFFERS 1
/* RF230_CONF_CHECKSUM=0 for automatic hardware checksum */
#define RF230_CONF_CHECKSUM 0

#define CONTIKIMAC_CONF_RADIO_ALWAYS_ON  1

/* Not part of C99 but actually present */
int strcasecmp(const char*, const char*);

#define ADDRESSLOCAL 243
//#define GATEWAY
//#define PUTS(...) puts(__VA_ARGS__)
#define PUTS(...)
#endif /* __CONTIKI_CONF_H__ */
