#include "Routing.h"

module RoutingP {
	provides interface Routing;
  uses {
		interface Leds;
    		interface Receive;
		interface AMSend;
    		interface Timer<TMilli> as MilliTimer;
    		interface SplitControl as AMControl;
    		interface Packet;
		interface Alarm <T32khz,uint32_t> as Timer32KHz;
		interface Alarm <T32khz,uint32_t> as SendTimer;
		interface CC2420Packet;
		interface ParameterInit<uint16_t> as SeedInit;
		interface PacketAcknowledgements;
		interface CC2420Config;
	}
}

implementation {
	message_t packet[QUEUE_LENGTH];
	bool locked=FALSE;
	uint16_t counter=0;
	int8_t tree_parent[NODES];
	uint8_t hello_start = 0;
	int num_hello;
	int avg_RSSI[NODES];
	int no_of_packets[NODES];
	int tree_build=0;
	int msg_dest[QUEUE_LENGTH];
	int msg_length[QUEUE_LENGTH];
	bool ack_req[QUEUE_LENGTH];
	int send_req = 0;
	int send_done = 0;
	int bad_tree[NODES];
	int routing_started = 0;
	int retcnt = 0;
	void sendHello(void){
		tree_build = 0;
		routing_started = 0;
		if(num_hello > 0){
			// Send 10 Hello Packets
			num_hello--;
			call Timer32KHz.start(640);
		}
		else if(TOS_NODE_ID == 1){
			num_hello--;
			// Begin routing after all nodes have transmitted their Hello Packets
			call Timer32KHz.start(320000);
		}
	}

	void print_tree(void){
		int i;
		printf("Good Tree : {");
		for(i=0;i<NODES;i++){
			printf("%d,",tree_parent[i]);
		}
		printf("}\n");
		printfflush();

		printf("Bad Tree : {");
		for(i=0;i<NODES;i++){
			printf("%d,",bad_tree[i]);
		}
		printf("}\n");
	}


	void start_routing(void);
	void send_link_state(int,int);
	void build_sub_tree(int,int);
	bool check_routing_done(void);
	void send_routing_done(int);

	command int Routing.getNumNodes(){
		return NODES;
	}

	command int Routing.getParent(int child){
		return tree_parent[(child-1)];
	}


	command int8_t* Routing.getRoutingTree(){
		return tree_parent;
	}


	command void Routing.route()
	{

			int i;
			for(i=0;i<NODES;i++){
				no_of_packets[i]=0;
				tree_parent[i]=0;
				bad_tree[i]=0;
				avg_RSSI[i]=-100;
			}


			num_hello = 10;
			hello_start = 1;
			sendHello();

			call CC2420Config.setAddressRecognition(TRUE,TRUE);
			call CC2420Config.setAutoAck(TRUE,TRUE);
			call CC2420Config.sync();  //Synchronize new setting with Hardware
			call MilliTimer.startOneShot(2000);
  	}


	async event void Timer32KHz.fired(){

		if(num_hello >=0 ){

		if(locked){
			return;
		}
		else {
  			routing_msg_t* rcm = (routing_msg_t*)call AMSend.getPayload(&packet[send_done], sizeof(routing_msg_t));



      			if (rcm == NULL) {
				return;
      			}

			rcm->type = 0;
			rcm->src=TOS_NODE_ID;
			if(call PacketAcknowledgements.noAck(&packet[send_done])){
				ack_req[send_done]=FALSE;
			}
			msg_dest[send_done]=AM_BROADCAST_ADDR;
			msg_length[send_done]=sizeof(routing_msg_t);
			if (call AMSend.send(msg_dest[send_done], &packet[send_done],msg_length[send_done]) == SUCCESS) {
				locked = TRUE;
			}
    		}
		sendHello();
		}
		else if(TOS_NODE_ID == 1){
			start_routing();
			hello_start = 0;

		}
		else{
			hello_start = 0;
		}

	}


	event void MilliTimer.fired() {
		int i,j,k;
		if(TOS_NODE_ID == 1 && routing_started==0){
			start_routing();
		}
		else if(tree_build == 0){
		if(check_routing_done()){
			print_tree();
			if(TOS_NODE_ID == 1){
				for(i=0;i<NODES;i++){
					if(tree_parent[i] == TOS_NODE_ID){
						send_routing_done(i+1);
					}
				}
			}
		}
		else{
			for(i=1;i<NODES;i++){
				if(tree_parent[i]==0 && bad_tree[i]!=0){
					tree_parent[i]=bad_tree[i];
					for(j=0;j<NODES;j++){
						if(tree_parent[j]==TOS_NODE_ID){
							build_sub_tree(i+1,j+1);
						}
					}
				}
			}
		}
		}
		print_tree();

	}

	async event void SendTimer.fired(){

		if(locked){
		}
		else {

			// Ask for link states to the nodes connected to the node.
			if(call PacketAcknowledgements.requestAck(&packet[send_done]) == SUCCESS){
				ack_req[send_done]=TRUE;
			}
			else{
				ack_req[send_done]=FALSE;
			}
  			if (call AMSend.send(msg_dest[send_done], &packet[send_done], msg_length[send_done]) == SUCCESS) {
				locked = TRUE;

			}
			else{
				call SendTimer.start(10);
			}

    		}

	}

	event message_t* Receive.receive(message_t* bufPtr,void* payload, uint8_t len) {

		int rssi;
		int i;
    		if (len == sizeof(routing_msg_t)){
			// If routing message is received
			routing_msg_t* rcm = (routing_msg_t*)payload;

			if(rcm->type == 0){ // type = 0 for hello packet

				if(rcm->type == 0 && hello_start == 0){ // Begin sending Hello packets if first Hello packet is received.
					num_hello=10;
					hello_start=1;
					call Timer32KHz.start(640+10*TOS_NODE_ID);
				}

				rssi=(call CC2420Packet.getRssi(bufPtr))-45;
				avg_RSSI[(rcm->src-1)]=(((avg_RSSI[(rcm->src-1)]*no_of_packets[(rcm->src - 1)])+rssi)/(no_of_packets[(rcm->src - 1)]+1));
				no_of_packets[(rcm->src - 1)]++;
		    		return bufPtr;
			}
    		}
		else if(len == sizeof(tree_msg_t)){ // tree nessage

			tree_msg_t* rcm = (tree_msg_t*)payload;
			int j;
			call Leds.set(4);
			for(i=0;i<NODES;i++){
				tree_parent[i]=rcm->tree[i];
			}
			print_tree();
			if(rcm->type == 0){
			if(rcm->dest == TOS_NODE_ID){
				send_link_state(TOS_NODE_ID,rcm->src);
			}
			else{
				if(!(check_routing_done())){
					for(i=0;i<NODES;i++){
						if(tree_parent[i] == TOS_NODE_ID){
							build_sub_tree(rcm->dest,i+1);
						}
					}
				}
			}
			print_tree();
			}
			else if(rcm->type==1){
				signal Routing.routeDone(SUCCESS);
				if(tree_build == 0){
				for(i=0;i<NODES;i++){
					if(tree_parent[i]== TOS_NODE_ID){

						send_routing_done(i+1);
					}
				}

				}

				tree_build=1;

			}
			return bufPtr;

		}
		else if (len == sizeof(link_msg_t)){
			// Received a link state message.

			link_msg_t* lstat = (link_msg_t*)payload;
			call Leds.set(2);

			if(lstat->type == 0){
				if(TOS_NODE_ID == 1){
				// If head node, process the link state
					int j;

					for(i=1;i<NODES;i++){
						if(tree_parent[i] == 0 && lstat->link_rssi[i] > GOOD_LINK){
							tree_parent[i]=lstat->src;
							bad_tree[i]=TOS_NODE_ID;
						}
						else if(tree_parent[i] == 0 && lstat->link_rssi[i] > BAD_LINK){
							bad_tree[i]=TOS_NODE_ID;
						}
					}
					if(!(check_routing_done())){
						for(i=0;i<NODES;i++){
							if(tree_parent[i]==lstat->src){
								for(j=0;j<NODES;j++){
									if(tree_parent[j]==TOS_NODE_ID){
										build_sub_tree(i+1,j+1);
									}
								}
							}
						}
					}
					print_tree();
				}
				else if (lstat->dest == TOS_NODE_ID){

					// If not head node, pass it to parent node.
						link_msg_t* l_stat = (link_msg_t*)call AMSend.getPayload(&packet[send_req], sizeof(link_msg_t));

		      				if (l_stat == NULL) {
							return;
		      				}

						l_stat->type = lstat->type;
						l_stat->src=lstat->src; // Do not modify the source.
						l_stat->dest = tree_parent[TOS_NODE_ID-1];
						for(i=0;i<NODES;i++){
							l_stat->link_rssi[i]=lstat->link_rssi[i];
						}
						msg_dest[send_req]=l_stat->dest;
						msg_length[send_req]=sizeof(link_msg_t);

						send_req++;
						call SendTimer.start(10);

						if(send_req == QUEUE_LENGTH ){
							send_req = 0;
						}
			}
			return bufPtr;
			}
    		}
		else{
			return bufPtr;
		}
  	}

	event void AMSend.sendDone(message_t* bufPtr, error_t error) {
		if (&packet[send_done] == bufPtr) {
			if(ack_req[send_done]){
				if(call PacketAcknowledgements.wasAcked(&packet[send_done]) == TRUE){
					locked = FALSE;
					send_done++;
					retcnt=0;
					if(send_done == QUEUE_LENGTH){
						send_done=0;
					}
					if(send_req != send_done){
						call SendTimer.start(10);
					}

				}
				else{
					locked=FALSE;
					retcnt++;
					if(retcnt >=GIVE_UP){
						send_done++;
						if(send_done == QUEUE_LENGTH){
							send_done=0;
						}
						retcnt=0;
					}
					if(send_req != send_done){
					call SendTimer.start(10);
					}
					//if (call AMSend.send(msg_dest[send_done], &packet[send_done], msg_length[send_done]) == SUCCESS) {
					//	locked = TRUE;
					//}
				}
			}
			else{
				locked = FALSE;
				retcnt=0;
				if(send_req != send_done){
					call SendTimer.start(10);
				}

			}

    		}
  	}


	void start_routing(void){
		int i,j;
		for(i=0;i<NODES;i++){
			if(avg_RSSI[i] > GOOD_LINK){
				tree_parent[i]=TOS_NODE_ID;
				bad_tree[i]=TOS_NODE_ID;
			}
			else if(avg_RSSI[i] > BAD_LINK){
				bad_tree[i]=TOS_NODE_ID;
			}
		}
		call MilliTimer.startPeriodic(20000);
		print_tree();
		routing_started=1;
		if(!(check_routing_done())){
			for(i=0;i<NODES;i++){
				if(tree_parent[i]==1){
					build_sub_tree(i+1,i+1);

				}
			}
		}
		call Leds.set(1);
	}


	void build_sub_tree(int destination,int next_hop){
		int i;

			// Ask for link states to the nodes connected to the node.
  			tree_msg_t* rcm = (tree_msg_t*)call AMSend.getPayload(&packet[send_req], sizeof(tree_msg_t));

      			if (rcm == NULL) {
				return;
      			}

			rcm->type = 0;
			rcm->src=TOS_NODE_ID;
			rcm->dest = destination;
			for(i=0;i<NODES;i++){
				rcm->tree[i]=tree_parent[i];
			}
			msg_dest[send_req]=next_hop;
			msg_length[send_req]=sizeof(tree_msg_t);

			call SendTimer.start(10);
			send_req++;
			if(send_req == QUEUE_LENGTH){
				send_req=0;
			}

	}

	void send_link_state(int src,int dest){

		// Send link state to the destination.

		int i;
  			link_msg_t* lstat = (link_msg_t*)call AMSend.getPayload(&packet[send_req], sizeof(link_msg_t));

      			if (lstat == NULL) {
				return;
      			}

			lstat->type = 0;
			lstat->src=src;
			lstat->dest = dest;

			for(i=0;i<NODES;i++){
				lstat->link_rssi[i]=avg_RSSI[i];
			}
			msg_dest[send_req]=dest;
			msg_length[send_req]=sizeof(link_msg_t);

			call SendTimer.start(10);
			send_req++;
			if(send_req == QUEUE_LENGTH){
				send_req=0;
			}

	}



	bool check_routing_done(void){
		int i;
		for(i=1;i<NODES;i++){
			if(tree_parent[i]==0){
				return FALSE;
			}
		}
		signal Routing.routeDone(SUCCESS);
		call MilliTimer.stop();
		if(tree_build == 0){
			for(i=0;i<NODES;i++){
				if(tree_parent[i] == TOS_NODE_ID){
					send_routing_done(i+1);
				}
			}
		}

		tree_build = 1;
		return TRUE;
	}

	void send_routing_done(int dest){
		int i;

		//if(tree_build==0){
			// Ask for link states to the nodes connected to the node.
  			tree_msg_t* rcm = (tree_msg_t*)call AMSend.getPayload(&packet[send_req], sizeof(tree_msg_t));

      			if (rcm == NULL) {
				return;
      			}

			rcm->type = 1;
			rcm->src=TOS_NODE_ID;
			rcm->dest = dest;
			for(i=0;i<NODES;i++){
				rcm->tree[i]=tree_parent[i];
			}
			msg_dest[send_req]=dest;
			msg_length[send_req]=sizeof(tree_msg_t);

			send_req++;
			if(send_req == QUEUE_LENGTH){
				send_req=0;
			}
			call SendTimer.start(10);

		print_tree();
		//}
	}

	event void AMControl.startDone(error_t err){}
	event void AMControl.stopDone(error_t err) {}
	event void CC2420Config.syncDone(error_t error) {
		if(error == SUCCESS){
		}
		else{
			call CC2420Config.sync();
		}

	}
}
