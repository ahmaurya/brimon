/**
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * Written (W) 2011 Jeet Patani
 * Copyright (C) 2011 Jeet Patani
 */

/*
	Transfer Interface provides interface to collect data from all nodes.
	It will be executed by head node.

	command collect() will send one by one request to eacch node to start data transfer.
	event collectDone() will be signaled at the end entire data transfer.
*/

interface Collect
{
	async command void collect();
	event void collectDone(error_t error);
}
