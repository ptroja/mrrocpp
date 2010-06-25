/* 
 *  libplayerc : a Player client library
 *  Copyright (C) Andrew Howard 2002-2003
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 */
/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) Andrew Howard 2003
 *                      
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
/*  Speech Proxy for libplayerc library. 
 *  Structure based on the rest of libplayerc. 
 */
#include <assert.h>
#include <stdlib.h>
#include <string.h>

#include "playerc.h"
#include "libplayerc_error.h"

// Local declarations
void playerc_speech_recognition_putdata (playerc_speech_recognition_t *device, player_msghdr_t *header,
			    player_speech_recognition_data_t *data, size_t len);

// Create a new speech recognition proxy
playerc_speech_recognition_t *playerc_speech_recognition_create(playerc_client_t *client, int index)
{
  playerc_speech_recognition_t *device;

  device = malloc(sizeof(playerc_speech_recognition_t));
  memset(device, 0, sizeof(playerc_speech_recognition_t));
  playerc_device_init(&device->info, client, PLAYER_SPEECH_RECOGNITION_CODE, index,
                      (playerc_putdata_fn_t) playerc_speech_recognition_putdata);
    
  return device;
}

// Destroy a speech recognition proxy
void playerc_speech_recognition_destroy(playerc_speech_recognition_t *device)
{
  playerc_device_term(&device->info);
  free(device);
}

// Subscribe to the speech recognition device
int playerc_speech_recognition_subscribe(playerc_speech_recognition_t *device, int access)
{
  return playerc_device_subscribe(&device->info, access);
}

// Un-subscribe from the speech recognition device
int playerc_speech_recognition_unsubscribe(playerc_speech_recognition_t *device)
{
  return playerc_device_unsubscribe(&device->info);
}

// Process incoming data
void playerc_speech_recognition_putdata (playerc_speech_recognition_t *device, player_msghdr_t *header,
			    player_speech_recognition_data_t *data, size_t len)
{
    strncpy(device->data.text, data->text, strlen(data->text));
}

