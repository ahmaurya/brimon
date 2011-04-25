/**
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * Written (W) 2011 Abhinav Maurya
 * Copyright (C) 2011 Abhinav Maurya
 */

/** interface provided by CommandC */

interface Command
{
  command void execCommand(uint16_t commType);
  event void execCommandDone(error_t error, uint16_t commType);
}
