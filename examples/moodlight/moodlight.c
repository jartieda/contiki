/*
 * Copyright (c) 2006, Swedish Institute of Computer Science.
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
 */

/**
 * \file
 *         A very simple Contiki application showing how Contiki programs look
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "contiki.h"

#include <stdio.h> /* For printf() */
/*---------------------------------------------------------------------------*/
PROCESS(moodlight_process, "Moddlight process");
AUTOSTART_PROCESSES(&moodlight_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(moodlight_process, ev, data)
{
//  static struct etimer timer_send_image;
//  static struct etimer timer_send_packet;
//  static int sd;
//  static int connected;
//  static sockaddr addr;
//  static unsigned short port;
//  static int ipkg;
//  char wifi_buff[] = "POST /recibeimg.php HTTP/1.1\n\
//Host: 192.168.1.2\n\
//User-Agent: my custom client v.1\n\
//Content-Type: application/octet-stream\n\
//Content-Length: 38400\n\
//\n";
//  int wifi_buff_leng = sizeof("POST /recibeimg.php HTTP/1.1\n\
//Host: 192.168.1.2\n\
//User-Agent: my custom client v.1\n\
//Content-Type: application/octet-stream\n\
//Content-Length: 38400\n\
//\n");

  PROCESS_BEGIN();
//  sd = -1;
//  connected = -1;
//  addr.sa_family = AF_INET;
//  // port
//  port = 81;
//  addr.sa_data[0] = (port & 0xFF00) >> 8;
//  addr.sa_data[1] = (port & 0x00FF);
//  //ip
//  addr.sa_data[2] = 192;
//  addr.sa_data[3] = 168;
//  addr.sa_data[4] = 1;
//  addr.sa_data[5] = 2;
//
//  while (1)
//  {
//      etimer_set(&timer_send_image, CLOCK_SECOND * 15);
//      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer_send_image));
//      sd = socket(AF_INET,SOCK_STREAM, IPPROTO_TCP);
//      if (sd != -1)
//      {
//          connected = connect(sd,&addr,sizeof(addr));
//          if (connected == 0)
//          {
//              GPIO_SetBits(GPIOD, GPIO_Pin_14);
//              //header
//              send(sd, wifi_buff, wifi_buff_leng, 0);
//              etimer_set(&timer_send_packet, CLOCK_SECOND * 2);
//              //image divided in 16 packets of 1200: 16 = 160 *120 /1200
//              for (ipkg = 0; ipkg < 32; ipkg++)
//              {
//                  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer_send_packet));
//                  send(sd, frame_buffer+(ipkg*1200),1200, 0);
//                  etimer_reset(&timer_send_packet);
//              }
//          }
//          PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer_send_packet));
//
//          closesocket(sd);
//      }
//      GPIO_ResetBits(GPIOE, GPIO_Pin_11);
//      //GPIO_SetBits(GPIOE, GPIO_Pin_12);
//      //GPIO_ResetBits(GPIOE, GPIO_Pin_12);
//
//      etimer_reset(&timer_send_image);
//  }
  
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
