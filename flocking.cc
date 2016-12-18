
/**
* Flocking.cc 
* Wie Hsing Li 20016
* g++ -c communicate.c
*
*g++ -I/usr/local/include -L/usr/local/lib -o flocking `pkg-config --cflags playerc++` flocking.cc `pkg-config --libs playerc++` -lnsl communicate.o
*
*
**/


#include "flocking.h"



int port;
int proxyport;
int d_sense;
int dist;
double dfc, ndfc;

#define RAYS 32
using namespace PlayerCc;
using namespace std;

double mtoi(double meters){

	double inches;
	inches = meters * MPI;
	return inches;
}

double itom(double inches){

	double meters;
	meters = inches / MPI;
	return meters;
}

double radtod(double radians){

	double degree;
	degree = radians * 57.2958;
	if(degree < 0) {
		degree = degree + 360;
	}
	if(degree >= 360) degree = 0;

	return degree; 
}
double safewalkTimer(){

	clock_t startTime = clock();
	double firstPass = 0;
	
		//250 is to correct for safewalking delay to timer 
	return firstPass = ((250*clock()) - startTime) / CLOCKS_PER_SEC;
}


void send_cmd(int bfd, double x, double y, double sx, double sy, double yaw)
{

	// create a message
	int nbytes = 0;
	char *msg = NULL;
	char *char_from = NULL;
	char *char_x = NULL;
	char *char_y = NULL;
	char *speed_x = NULL;
	char *speed_y = NULL;
	char *char_yaw = NULL;

	msg = (char *) malloc(60*sizeof(char));
	char_from = (char *) malloc(10*sizeof(char));
	char_x = (char*) malloc(sizeof(float));
	char_y = (char*) malloc(sizeof(float));
	speed_x = (char*) malloc(sizeof(float));
	speed_y = (char*) malloc(sizeof(float));
	char_yaw = (char*) malloc(sizeof(float));


	itoa((int)port, char_from);
	sprintf(char_x, "%f",  x);
	sprintf(char_y, "%f", y);
	sprintf(speed_x, "%f", sx);
	sprintf(speed_y, "%f", sy);
	sprintf(char_yaw, "%f" , yaw);

	strcpy(msg, "C");
	strcat(msg, char_from);
	strcat(msg, "$");
	strcat(msg, char_x);
	strcat(msg, "&");
	strcat(msg, char_y);
	strcat(msg, "#");
	strcat(msg, speed_x);
	strcat(msg, "@");
	strcat(msg, speed_y);
	strcat(msg, "Y");
	strcat(msg, char_yaw);
	strcat(msg, "!");
//	printf("sending message: %s\n", msg);
	nbytes = talk_to_all(bfd, msg, H);
}

int randomNumber(int radius){

	int isecret, iguess;
	isecret = rand() & ((radius)*2) + (-radius);
	return isecret;
}


void parse_msg(char *msg, float *x, float *y, float *sx, float *sy, float *ryaw)
{
	//passes by reference to set position, speed, and yaw to objects in main using the parse method
  char *ptr, *token;

  token = strtok(msg, "!");
  while (token != NULL) {
    if (token[0] == 'C') {
    }
    ptr = strstr(token, "C");
    ptr++;
    ptr = strstr(token, "$");
    ptr++;
    *x = atof(ptr);
    ptr = strstr(token, "&");
    ptr++;
    *y = atof(ptr);
    ptr = strstr(token, "#");
    ptr++;
    *sx = atof(ptr);
    ptr = strstr(token, "@");
    ptr++;
    *sy = atof(ptr);
    ptr = strstr(token, "Y");
    ptr++;
    *ryaw = atof(ptr);
    token = strtok(NULL, "!");
  }
}

void safewalk(double *newspeed, double *newturnrate, LaserProxy &lp, Position2dProxy &pp, string command, double cx, double cy)
{

	double minR = lp.GetMinRight();
	double minL = lp.GetMinLeft();

	double l = (1e5*minR)/500-100;
	double r = (1e5*minL)/500-100;

	if (l > 100)
		l = 100;
	if (r > 100)
		r = 100;


	if(command == "a"){
		if(dfc <= dist){ 
			*newspeed = 0;
//			printf("Less than...\n");
		}
		// if(ndfc < dfc){ 		//seen in reverse.  Distance from centroid is getting greater
		// 	*newturnrate = *newturnrate + 100;
		// 	if(l < 10 || r < 10){ 
		// 		*newspeed = *newspeed*(-1);		//to avoid collision
		// 	}
		// }else *newturnrate = r-l;
		if(dfc > dist){
//			printf("Greater than...\n");
			if(r<20 || l <20){
				*newspeed = 0;
				*newturnrate = (r-l);
				*newturnrate = limit(*newturnrate, -40.0, 40.0);	//newturnrate has to be inside the limit (-40 to 40)
			}else{
//				printf("CX: %f  CY: %f\n", cx, cy);
				pp.GoTo(cx, cy, pp.GetYaw());
			}
		}
	}
	if(command == "d"){
		if(dfc > dist || dfc == 0.0){
			*newspeed = 0;
		}
		// if(ndfc > dfc){
		// 	*newturnrate = *newturnrate + 60;
		// 	if(l < 10 || r < 10){ 
		// 		*newspeed = *newspeed*(-1);		//to avoid collision
		// 	}
		// 	else *newturnrate = r-l;
		// }
		// else *newturnrate = r-l;

		if(r<20 || l <20){
			*newturnrate = (r-l);
			*newturnrate = limit(*newturnrate, -40.0, 40.0);	//newturnrate has to be inside the limit (-40 to 40)
		}else{
//				printf("CX: %f  CY: %f\n", cx, cy);
			pp.GoTo(cx, cy, pp.GetYaw());
		}		

		
		// printf("DISPERSE!!\n");
	}

	*newturnrate = dtor(*newturnrate);
}

double calcCentroidX(float arr[], int size, bool counted[]){

	double average = 0.0;

	int i = 0;
	while(i < 6){

		if(counted[i] == false) {i++;}
		else{
			average = average + arr[i];
			i++;
		}
	}

	return average/size;
}

double calcCentroidY(float arr[], int size, bool counted[]){

	double average = 0;

	int i = 0;
	while(i < 6){

		if(counted[i] == false) {i++;}
		else{
			average = average + arr[i];
			printf("arr[%d]: %f\n", i, arr[i]);
			i++;
		}
	}

	return average/size;
}

double calcDistance(float x1, float x2, float y1, float y2){

	double distance;

	distance = sqrt( ((x2-x1) * (x2-x1)) + ((y2-y1) * (y2-y1)) );
	return distance;
}

/* main function */
int main(int argc, char** argv)
{
	parse_args(argc,argv);
 	int nbytes = 0;
  	char msg[MAXBUF];
  	char msg2[MAXBUF];
  	char msg3[MAXBUF];
  	char msg4[MAXBUF];
  	char msg5[MAXBUF];
  	char msg6[MAXBUF];
  	bool flag = true;
	bool timers = true;
	srand( (unsigned)time( 0 ) );


//if executed w/o port number

	port = atoi(argv[1]);

	

	string command = argv[2];
	int radius, randx = 0, randy = 0, rd = 0;

	if(command == "r"){

		if (argc < 3) {
			printf("mytest port d_sense\n");
	    	exit(1);
	  	}
		radius = atoi(argv[3]);
		for (int i = 0; i < 6; ++i){

 			randx = rand() % (2*(radius)) - radius;
 			randy = rand() % (2*(radius)) - radius;
 			rd = rand() % (360);
 			printf("\nRobot %d:  (%d in, %d in)  Rand Degree: %d\n", i+1, randx, randy, rd);
 			printf("pose [%f %f 0 %d]\n", itom(randx), itom(randy), rd);
		}
		exit(1);
	}else{
		if (argc < 4) {
		printf("mytest port command d_sense distance\n");
    	exit(1);
  	}
		d_sense = atoi(argv[3]);
		dist = atoi(argv[4]);
	}

/*********************************************************************************************************************************/

  
 //proxyport to avoid address already in use
  switch(port)
  {
  	case 6665: proxyport = 7777; break;
  	case 6666: proxyport = 7778; break;
  	case 6667: proxyport = 7779; break;
  	case 6668: proxyport = 7780; break;
  	case 6669: proxyport = 7781; break;
  	case 6670: proxyport = 7782; break;
  	default:   proxyport = 7783; break;
  }
   
// 6665 	7777

	int leaderport = 7777;

  // initiate communication

  	try
 	{

	 	PlayerClient robot(gHostname, port);
	    Position2dProxy pp(&robot, gIndex);
	    LaserProxy lp(&robot, gIndex);
//	    CameraProxy cp(&robot, gIndex);
//		RangerProxy     laserProxy(&robot,1);
//		RangerProxy     sonarProxy(&robot,0);

	    double ppx = 0, ppy = 0, ppyaw;
		char run_type;
		double newspeed = 0.1, newturnrate = 0.1;
		int br = create_broadcast(proxyport, H);
		int lr[6];
		float rx[6];
		float ry[6];
		float ryaw[6];
		int counter = 1;  //1 because we are comparing one robot against the others
		bool counted[6];
		double cx = 0 , cy = 0;
		float rxspeed[6];
		float ryspeed[6];
		bool allstop = false;
		double endtime;
		bool rSpeed[6];
//		double dfc, ndfc;		//distance from centroid
		bool alltrue = true;
		bool allfalse = true;
		//initializing all arrays
		for (int i = 0; i < 6; i++)
		{
			lr[i] = create_listen(7777 + i, H);
			rx[i] = 999.0;
			ry[i] = 999.0;
			ryaw[i] = 0.0;
			rxspeed[i] = 999.0;
			ryspeed[i] = 999.0;
			counted[i] = false;
			rSpeed[i] = false;
		}
		
		while(1){
			safewalkTimer();
			//safewalk loop	
			// this blocks until new data comes; 10Hz by default
			robot.Read();
			pp.SetSpeed(newspeed, newturnrate);
			ppyaw = radtod(pp.GetYaw());
			send_cmd(br, ppx, ppy, pp.GetXSpeed(), pp.GetYSpeed(), ppyaw);
			ppx = mtoi(pp.GetXPos());
			ppy = mtoi(pp.GetYPos());
			
			//If statements are used to see if distance between robots are with d_sense; if it is include in centroid calculations.

			switch(proxyport){

				case 7777:	//for r[0]
					rx[0] = mtoi(pp.GetXPos()); ry[0] = mtoi(pp.GetYPos()); counted[0]=true; rxspeed[0]=pp.GetXSpeed(); ryspeed[0]=pp.GetYSpeed(); ryaw[0] = ppyaw;
					nbytes = listen_to_robot(lr[1],msg2);
					
					for (int i = 0; i < 6; ++i)
					{
						nbytes = listen_to_robot(lr[i],msg);
						parse_msg(msg, &rx[i], &ry[i], &rxspeed[i], &ryspeed[i], &ryaw[i]);
						if(calcDistance(rx[i], rx[0], ry[i], ry[0]) <= d_sense && counted[i] == false) {counter++; counted[i] = true;}
						if(calcDistance(rx[i], rx[0], ry[i], ry[0]) > d_sense && counted[i] == true) {counter--; counted[i] = false;rx[i]=999.0; ry[i]=999.0;}
					}				
					if(counter > 1){
						cx = calcCentroidX(rx, counter, counted); 
						cy = calcCentroidY(ry, counter, counted);
						dfc = calcDistance(cx, rx[0], cy, ry[0]);
					} else {cx = 0; cy = 0; dfc = 0;}
					
					for (int i = 0; i < 6; ++i)
					{
						if(rxspeed[i] == 0 && ryspeed[i] == 0) rSpeed[i] = true;
						else rSpeed[i] = false;
					}

					break;
				case 7778:	//for r[1]
					rx[1] = mtoi(pp.GetXPos()); ry[1] = mtoi(pp.GetYPos()); counted[1] = true; rxspeed[1]=pp.GetXSpeed(); ryspeed[1]=pp.GetYSpeed();ryaw[1] = ppyaw;

					for (int i = 0; i < 6; ++i)
					{
						nbytes = listen_to_robot(lr[i],msg);
						parse_msg(msg, &rx[i], &ry[i], &rxspeed[i], &ryspeed[i], &ryaw[i]);
						if(calcDistance(rx[i], rx[1], ry[i], ry[1]) <= d_sense && counted[i] == false) {counter++; counted[i] = true;}
						if(calcDistance(rx[i], rx[1], ry[i], ry[1]) > d_sense && counted[i] == true) {counter--; counted[i] = false;rx[i]=999.0; ry[i]=999.0;}
					}					
					if(counter > 1){
						cx = calcCentroidX(rx, counter, counted); 
						cy = calcCentroidY(ry, counter, counted);
						dfc = calcDistance(cx, rx[1], cy, ry[1]);
					}else {cx = 0; cy = 0; dfc = 0;}

					for (int i = 0; i < 6; ++i)
					{
						if(rxspeed[i] == 0 && ryspeed[i] == 0) rSpeed[i] = true;
						else rSpeed[i] = false;
					}
					break;
				case 7779: //for r[2]
					rx[2] = mtoi(pp.GetXPos()); ry[2] = mtoi(pp.GetYPos()); counted[2] = true; rxspeed[2]=pp.GetXSpeed(); ryspeed[2]=pp.GetYSpeed(); ryaw[2] = ppyaw;

					for (int i = 0; i < 6; ++i)
					{
						nbytes = listen_to_robot(lr[i],msg);
						parse_msg(msg, &rx[i], &ry[i], &rxspeed[i], &ryspeed[i], &ryaw[i]);
						if(calcDistance(rx[i], rx[2], ry[i], ry[2]) <= d_sense && counted[i] == false) {counter++; counted[i] = true;}
						if(calcDistance(rx[i], rx[2], ry[i], ry[2]) > d_sense && counted[i] == true) {counter--; counted[i] = false;rx[i]=999.0; ry[i]=999.0;}
					}

					if(counter > 1){
						cx = calcCentroidX(rx, counter, counted); 
						cy = calcCentroidY(ry, counter, counted);
						dfc = calcDistance(cx, rx[2], cy, ry[2]);
					} else {cx = 0; cy = 0; dfc = 0;}

					for (int i = 0; i < 6; ++i)
					{
						if(rxspeed[i] == 0 && ryspeed[i] == 0) rSpeed[i] = true;
						else rSpeed[i] = false;
					}
					break;
				case 7780: //for r[3]
					rx[3] = mtoi(pp.GetXPos()); ry[3] = mtoi(pp.GetYPos()); counted[3] = true; rxspeed[3]=pp.GetXSpeed(); ryspeed[3]=pp.GetYSpeed();ryaw[3] = ppyaw;
				
					for (int i = 0; i < 6; ++i)
					{
						nbytes = listen_to_robot(lr[i],msg);
						parse_msg(msg, &rx[i], &ry[i], &rxspeed[i], &ryspeed[i], &ryaw[i]);
						if(calcDistance(rx[i], rx[3], ry[i], ry[3]) <= d_sense && counted[i] == false) {counter++; counted[i] = true;}
						if(calcDistance(rx[i], rx[3], ry[i], ry[3]) > d_sense && counted[i] == true) {counter--; counted[i] = false;rx[i]=999.0; ry[i]=999.0;}
					}
					if(counter > 1){
						cx = calcCentroidX(rx, counter, counted); 
						cy = calcCentroidY(ry, counter, counted);
						dfc = calcDistance(cx, rx[3], cy, ry[3]);
					} else {cx = 0; cy = 0; dfc = 0;}

					for (int i = 0; i < 6; ++i)
					{
						if(rxspeed[i] == 0 && ryspeed[i] == 0) rSpeed[i] = true;
						else rSpeed[i] = false;
					}
					break;
				case 7781: //for r[4]
					rx[4] = mtoi(pp.GetXPos()); ry[4] = mtoi(pp.GetYPos()); counted[4] = true; rxspeed[4]=pp.GetXSpeed(); ryspeed[4]=pp.GetYSpeed();ryaw[4] = ppyaw;
			
					for (int i = 0; i < 6; ++i)
					{
						nbytes = listen_to_robot(lr[i],msg);
						parse_msg(msg, &rx[i], &ry[i], &rxspeed[i], &ryspeed[i], &ryaw[i]);
						if(calcDistance(rx[i], rx[4], ry[i], ry[4]) <= d_sense && counted[i] == false) {counter++; counted[i] = true;}
						if(calcDistance(rx[i], rx[4], ry[i], ry[4]) > d_sense && counted[i] == true) {counter--; counted[i] = false;rx[i]=999.0; ry[i]=999.0;}
					}					
					if(counter > 1){
						cx = calcCentroidX(rx, counter, counted); 
						cy = calcCentroidY(ry, counter, counted);
						dfc = calcDistance(cx, rx[4], cy, ry[4]);
					} else {cx = 0; cy = 0; dfc = 0;}

					for (int i = 0; i < 6; ++i)
					{
						if(rxspeed[i] == 0 && ryspeed[i] == 0) rSpeed[i] = true;
						else rSpeed[i] = false;
					}

					break;
				case 7782: //for r[5]
					rx[5] = mtoi(pp.GetXPos()); ry[5] = mtoi(pp.GetYPos()); counted[5] = true; rxspeed[5]=pp.GetXSpeed(); ryspeed[5]=pp.GetYSpeed();ryaw[5] = ppyaw;
				
					for (int i = 0; i < 6; ++i)
					{
						nbytes = listen_to_robot(lr[i],msg);
						parse_msg(msg, &rx[i], &ry[i], &rxspeed[i], &ryspeed[i], &ryaw[i]);
						if(calcDistance(rx[i], rx[5], ry[i], ry[5]) <= d_sense && counted[i] == false) {counter++; counted[i] = true;}
						if(calcDistance(rx[i], rx[5], ry[i], ry[5]) > d_sense && counted[i] == true) {counter--; counted[i] = false;rx[i]=999.0; ry[i]=999.0;}
					}					
					if(counter > 1){
						cx = calcCentroidX(rx, counter, counted); 
						cy = calcCentroidY(ry, counter, counted);
						dfc = calcDistance(cx, rx[5], cy, ry[5]);
					} else {cx = 0; cy = 0; dfc = 0;}

					for (int i = 0; i < 6; ++i)
					{
						if(rxspeed[i] == 0 && ryspeed[i] == 0) rSpeed[i] = true;
						else rSpeed[i] = false;
					}
				
				default: break;		

			}  //end switch
			safewalk(&newspeed, &newturnrate, lp, pp, command, itom(cx), itom(cy));
			ndfc = dfc;		//setting old value of dfc to ndfc; this is used for aggregation and dispersion

			if(safewalkTimer()>5){
				
				for (int i = 0; i < 6; ++i)
				{
					if(rSpeed[i] == true) allfalse = false;
					else alltrue = false;
				}

			}
//			printf("alltrue:  %d; allfalse:  %d\n", alltrue, allfalse);
			if(alltrue == true && allfalse == false) { break;}

			alltrue = true; allfalse = true;
		}//end while
		endtime = safewalkTimer();
		printf("Time: %f\n", endtime);
		printf("PORT:  %d \t DFC: %f\n", port, dfc);
		sleep(2);
	}	//end try
	catch (PlayerCc::PlayerError & e)
	{
		std::cerr << e << std::endl;
		return -1;
	}

}//end main