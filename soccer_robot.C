#include <iostream>
#include <nemo/Nemo.H>
#include <Util/mongoose.h>
#include <Image/DrawOpsSimple.H>
#include <math.h>

using namespace std;

// State function prototypes
void DriveToBall();
void DriveWithBall();
void DriveToGoal();
void ShootBall();
void SearchBall();
void AvoidObstacle();
void Turn(float, string);

//####################################################
//Camera variables
static std::vector<byte> displayImage;
static pthread_mutex_t imageMutex;
static bool camera_init = false;
static PixRGB<byte> color_min = PixRGB<byte>(0,0,0);
static PixRGB<byte> color_max = PixRGB<byte>(255,255,255);
static int color_quality = 10;
int quality; char cquality[5];

//$$$ PID settings
float set_val, sensor_val, curr_error, rotation, M1, M2, integral_e, de;
//Set gains Kp, Ki, Kd:
float Kp = 3.0, Ki = 0.0, Kd = 0.0;

//Declare global variables
double currState = 1.1;
double prevState = 1.1;
float compass_angle[4] = {343.0, 91.0, 185.0, 269.0};
double exact_angle[4] = {270.0, 0.0, 90.0, 180.0};
int ball[6] = {130, 204, 163, 255, 82, 148}; 		//[rmin rmax gmin gmax bmin bmax]
int goal[6] = {166, 255, 87, 163, 145, 212}; 		//[rmin rmax gmin gmax bmin bmax]
int robot_direction = -1; 				//default compass heading
float sizeR = 0.0; 					//object size ratio updated by take image fxn
float trans = 0.0;
int tErr = 0; 						//object offset from image center updated by take image fxn
float prev_error_total = 0; 				//Ki PID parameter
float prev_error = 0; 					//Ki PID parameter
double opp_goal = 106.1; 				//Heading of the opponent's goal (assuming that it's east)
double own_goal = 249.5; 				//Heading of own goal (assuming that it's west)

int lsPin = 0; 		//left servo pin
int rsPin = 1; 		//right servo pin
int dribPin = 2; 	//dribbler servo pin
int kickPin = 3; 	//kicker servo pin
int frSonarPin = 4; 	//front sonar pin
int lftSonarPin = 5; 	//our left (when facing robot) sonar pin
int cPin = 6; 		//Compass pin
int BallSonarPin = 7;   //Sonar above ball pin
//int pressPin = 0; 	//Analog - Pressure sensor pin
int irPin = 1; 		//Analog - IR pin

//cout << "just initialized sizeR to 0" << endl;

//####################################################
//Declare key fxns
void Turn(float angleOffsetDeg, string dir) {
	//Turn the robot to a set direction
	float angleFROM = (Nemo::getCompass(cPin))*M_PI/180;
	float angleTO = angleOffsetDeg*M_PI/180;
	float angletoturn = atan2(sin(angleTO - angleFROM),cos(angleTO - angleFROM))*180/M_PI;
	while(abs(angletoturn) > 15) {
		if (dir == "CCW") {
			Nemo::setServo(lsPin, 67); // TURN LEFT/counter clockwise 63
			Nemo::setServo(rsPin, 78); // TURN LEFT/counter clockwise 74
			//wait(1);
		}
		else {
			Nemo::setServo(lsPin, 81);  // TURN RIGHT/clockwise 85
			Nemo::setServo(rsPin, 92);	// TURN RIGHT/clockwise 96
			//wait(1);
		}
		angleFROM = (Nemo::getCompass(cPin))*M_PI/180;
		angletoturn = atan2(sin(angleTO - angleFROM),cos(angleTO - angleFROM))*180/M_PI;
		cout << "angle to turn: " << angletoturn << ", and compass reading: " << Nemo::getCompass(cPin) << endl;
	}
	Nemo::setServo(lsPin,79); // STOP TURNING
	Nemo::setServo(rsPin,80); // STOP TURNING

	cout << "done turning" << endl;
}

int direction_update(int robot_direction) {
	//sense the robot direction and map to a cardinal compass direction
	float compass_direction = 0.0;
	for (int i = 0; i < 5; i++){
		compass_direction += Nemo::getCompass(cPin);
		usleep(100000);
	}
	compass_direction = compass_direction / 5;
	// *************************************************************************
	// TO DO 2: UPDATE THE COMPASS VALUES (TRANSITION VALUES BETWEEN NORTH, EAST, SOUTH, WEST)
	if (compass_direction <  37 || compass_direction >= 304)
		robot_direction = 0; // NORTH
	else if (compass_direction < 144 && compass_direction >= 37)
		robot_direction = 1; // EAST
	else if (compass_direction < 222 && compass_direction >= 144)
		robot_direction = 2; // SOUTH
	else if (compass_direction < 304 && compass_direction >= 222)
		robot_direction = 3; // WEST
	// *************************************************************************
	
	return robot_direction;
}

void wait(int dur) {
	//wait a set time then drop in a delay
	dur = dur*1e6;
	usleep(dur);
}

void stop_drive(){
	//stop the robot
	Nemo::setServo(lsPin,79);
	Nemo::setServo(rsPin,80);
}

int blind_drive(string dir, int dur) {
	//drive for a set amount of time and and then stop
	if (dir =="Forward") { //drive forward
		Nemo::setServo(lsPin,79-5);
		Nemo::setServo(rsPin,80+5);
	}
	else if (dir == "Backward") { //drive backward
		Nemo::setServo(lsPin,79+5);
		Nemo::setServo(rsPin,80-5);
	}
	wait(dur); //wait for a set time
	stop_drive(); //stop the robot
}

static void get_qsvar(const struct mg_request_info *request_info,
                      const char *name, char *dst, size_t dst_len) {
	const char *qs = request_info->query_string;
	mg_get_var(qs, strlen(qs == NULL ? "" : qs), name, dst, dst_len);
}

static void *callback(enum mg_event event,
					  struct mg_connection *conn) {
	const struct mg_request_info *request_info 
	= mg_get_request_info(conn);
	if (event == MG_NEW_REQUEST) {
		std::string uri(request_info->uri);
		std::string method(request_info->request_method);
		if (uri == "/image.jpg")
		{
			quality = 10; cquality[5];
			get_qsvar(request_info, "quality", cquality, sizeof(cquality));
			quality =  cquality[0] == '\0' ? 10 : atoi(cquality);
			
			std::vector<byte> imgData;
			
			pthread_mutex_lock( &imageMutex );
			imgData = displayImage;
			color_quality = quality;
			pthread_mutex_unlock( &imageMutex );
			
			
			mg_write(conn, &imgData[0], imgData.size());
			//LINFO("JPG Image Size %d quality %d",(int)imgData.size(),quality);
		
		}else if(uri == "/startCamera"){
			
			if(!camera_init){
				int width = -1; char cwidth[5];
				int height = -1;char cheight[5];
				get_qsvar(request_info, "width", cwidth, sizeof(cwidth));
				get_qsvar(request_info, "height", cheight, sizeof(cheight));
				width =  cwidth[0] == '\0' ? -1 : atoi(cwidth);
				height = cheight[0] == '\0' ? -1 : atoi(cheight);
				//camera initialization
				bool caminit = Nemo::startCamera(width, height);//160x120 or 320x240 or 640x480
				pthread_mutex_lock( &imageMutex );
				camera_init = caminit;
				pthread_mutex_unlock( &imageMutex );
				if(caminit)
					mg_printf(conn,"Satrt Camera with %3d %3d ",width,height);
				else
					mg_printf(conn,"Can't Find Camera ...Please Check /dev/video0");
				
			}else{
				mg_printf(conn,"Camera Is Running...");
				
			}

		}else if(uri == "/compass"){//for /compass?pin=5
      			int pin = -1;char cpin[3];
      			get_qsvar(request_info, "pin", cpin, sizeof(cpin));
      			pin = cpin[0] == '\0' ? -1 : atoi(cpin);
		
		}else{
			return NULL;
		}
		return (void *)"";  // Mark as processed
	} else {
		return NULL;
	}
}

void take_image(string object)
{
		Image<PixRGB<byte> > img = Nemo::grabImage();//Get image from usb camera
		int w = img.getWidth();
		int h = img.getHeight();
		int ImArr[w*h];
		
		//==================================================
		//============= Put Your Code Here =================
		//==================================================
		
		if (1) {
			for (int y=0; y<h; y++) {
				for(int x = 0; x < w; x++) {
					PixRGB<byte> px = img.getVal(Point2D<int> (x,y));
					byte r,g,b;
					px.getRGB(r,g,b);
					byte r_max, g_max, b_max;
					//cmax.getRGB(r_max, g_max, b_max);
					byte r_min, g_min, b_min;
					//cmin.getRGB(r_min, g_min, b_min);
					if (x==0 && y==0) {
						if (object == "Ball") {
							r_min = ball[0]; r_max = ball[1];
							g_min = ball[2]; g_max = ball[3];
							b_min = ball[4]; b_max = ball[5];
						}
						else if (object == "Goal") {
							r_min = goal[0]; r_max = goal[1];
							g_min = goal[2]; g_max = goal[3];
							b_min = goal[4]; b_max = goal[5];
						}
					}

					if(r >= r_min && r <= r_max && g >= g_min && g <= g_max && b >= b_min && b <= b_max)
					{
						ImArr[y*w + x] = 1;
						img.setVal(x,y,PixRGB<byte> (0,0,0));
					}
					else {ImArr[y*w + x] = 0;}
				}		
			}
		}
		/*
		 std::cout<<std::endl;
		 cout<<"Raw Image"<<endl;
		 for (int y=0; y < img.getHeight(); y++) {
		 for(int x = 0; x < img.getWidth(); x++) {
		 std::cout<<ImArr[y*w + x];			
		 }
		 std::cout<<std::endl;
		 }
		 std::cout<<std::endl;
		 //std::cin.get();
		 */	
		
		
		//****************************************************
		
		if (1) {
			//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$//
			
			//Two-step processing Algorithm
			//Set size of Equivalency Set Array, resize as needed for problem
			int rows = h;
			int cols = w;
			int EqSpan = 20;
			int EqSet[(rows*cols)*EqSpan]; //Equivalent set stored as array of 0's
			int EqCntr; //Counter used to traverse EqSet's rows during equivalancy set updates
			int ObjArr[rows*cols]; //Final output of the algorithm - Raw image array with values updated to account for object
			int lbl = 1; //intialize pixel label
			int W, NW, N, NE; //initialize temp neighbor values
			int adjMin; //temp var to hold the minimum neighbor values
			int PixVals[4]; //temp array to hold neighbor pixel values
			
			//Initialize of ObjArr to all zeros
			for (int m = 0; m<rows; m++) {
				for (int n=0; n<cols; n++) {
					ObjArr[m*cols + n] = 0;
				}
			}
			
			//Initialize of EqSet to all zeros
			for (int m = 0; m<rows*cols; m++) {
				for (int n=0; n<EqSpan; n++) {
					EqSet[m*EqSpan + n] = 0;
				}
			}
			
			//First pass
			for (int i = 0; i<rows; i++)
			{ //raster scan -> by col, then by row
				for (int j = 0; j<cols; j++)
				{
					if (lbl > rows*cols) {cout<<"lbl > EqSet row space!"<<endl;
						cout<<"lbl = "<<lbl<<", "<<"EqSetRowSpace = " <<rows*cols <<endl;
						cout << "There is too much variation in the image, please fix RGB bounds!"<<endl;
						//cin.get();
						//return 0;
					}
					
					//Check background value
					if (ImArr[i*cols + j] != 0)
					{
						//find neighbors [W, NW, N, NE] -> assuming 8-bit connectivity
						if(i == 0)
						{// along on the top-most row
							//check W neighbor
							//W = ImArr[i][j-1];
							W = ImArr[i*cols+j-1];
							if (j==0 || W == 0)
							{//at top left boundary or W neighbor is a background
								ObjArr[i * cols + j] = lbl;
								EqSet[lbl*EqSpan + 0] = lbl;
								lbl++; //step the pixel label entry
							}
							else
							{//W nieghbor has a label value already, thus current pixel is connected and has same label value
								ObjArr[i*cols + j] = ObjArr[i*cols + j -1];
							}
						}
						else if (j == 0 || j == cols -1)
						{// along the left-most or right-most boundary
							if (j==0)
							{//left-most, check N and NE neighbors
								//N = ImArr[i-1][j];
								//NE = ImArr[i-1][j+1];
								N = ImArr[(i-1)*cols + j];
								NE = ImArr[(i-1)*cols + j +1];
								if (N == 0 && NE == 0)
								{
									ObjArr[i*cols + j] = lbl;
									EqSet[lbl*EqSpan + 0] = lbl;
									lbl++; //step the pixel label entry
								} 
								else if (N != 0 && NE != 0)
								{
									//determine and choose the min label val among the neighbors
									adjMin = ObjArr[(i-1)*cols + j]; //assume to start that the N pixel has the min value
									adjMin < ObjArr[(i-1)*cols + j+1] ? adjMin = ObjArr[(i-1)*cols + j] : adjMin = ObjArr[(i-1)*cols + j+1];
									ObjArr[i*cols + j] = adjMin;
								}
								else
								{
									//choose the non-zero neighbor
									N != 0 ? ObjArr[i*cols + j] = ObjArr[(i-1)*cols + j] : ObjArr[i*cols + j] = ObjArr[(i-1)*cols + j+1];
								}
								//Check and update equivalency relationships, as needed							
								PixVals[0] = ObjArr[(i-1)*cols + j];
								PixVals[1] = ObjArr[(i-1)*cols + j+1];
								PixVals[2] = 0;
								PixVals[3] = 0;
								EqCntr = 0;
								while (1) {
									if (PixVals[EqCntr] != 0) {
										for (int m=0;m<3;m++) {
											//traverse the neighboring pixel values
											if (EqSet[(PixVals[EqCntr])*EqSpan + 0] == 0 && PixVals[m] != 0) {	
												EqSet[(PixVals[EqCntr])*EqSpan + 0] = PixVals[m];
												//adding value for first found eq relationship
											}
											for (int n=0; n<EqSpan; n++) {
												//fix row index by EqCntr, n traverses cols in EqSet									
												if (EqSet[(PixVals[EqCntr])*EqSpan + n] == PixVals[m]) {break;} //if eq relationship already exists, then skip
												else if (EqSet[(PixVals[EqCntr])*EqSpan + n] == 0) { //add eq. relationship
													EqSet[(PixVals[EqCntr])*EqSpan + n] = PixVals[m];
													break;} 
											}	
										}
									}
									EqCntr++; //step EqSet row counter
									if (EqCntr > 1) {break;} //break search once EqSet row counter is beyond the highest object label value
								}
							}
							else
							{//right most, check W, NW and N neighbors
								W = ImArr[i*cols + j-1];
								NW = ImArr[(i-1)*cols + j-1];
								N = ImArr[(i-1)*cols + j];
								if (W == 0 && NW == 0 && N ==0)
								{
									ObjArr[i*cols + j] = lbl;
									EqSet[lbl*EqSpan + 0] = lbl;
									lbl++; //step the pixel label entry
								} 
								else if (W != 0 && NW != 0 && N != 0)
								{
									//determine and choose the min label val among the neighbors
									adjMin = ObjArr[i*cols + j-1]; //assume to start that the W pixel has the min value
									for (int m = 0; m < 3; m++)
									{
										if (m==1) {adjMin < ObjArr[(i-1)*cols + j-1] ? adjMin = ObjArr[(i-1)*cols + j-1] : 1;} //compare to NW pixel
										else if (m==2) {adjMin < ObjArr[(i-1)*cols + j] ? adjMin = ObjArr[(i-1)*cols + j] : 1;} //compare to N pixel
									}
									ObjArr[i*cols + j] = adjMin;
								}
								else
								{
									//choose the non-zero and most minimum neighbor
									if (W != 0)
									{
										ObjArr[i*cols + j] == 0 || ObjArr[i*cols + j] > ObjArr[i*cols + j-1] ? ObjArr[i*cols + j] = ObjArr[i*cols + j-1] : 1;
									}
									if (NW != 0)
									{
										ObjArr[i*cols + j] == 0 || ObjArr[i*cols + j] > ObjArr[(i-1)*cols + j-1] ? ObjArr[i*cols + j] = ObjArr[(i-1)*cols + j-1] : 1;
									}
									if (N != 0)
									{
										ObjArr[i*cols + j] == 0 || ObjArr[i*cols + j] > ObjArr[(i-1)*cols + j] ? ObjArr[i*cols + j] = ObjArr[(i-1)*cols + j] : 1;
									}
								}
								//Check and update equivalency relationships, as needed
								PixVals[0] = ObjArr[i*cols + j-1];
								PixVals[1] = ObjArr[(i-1)*cols +j-1];
								PixVals[2] = ObjArr[(i-1)*cols + j];
								PixVals[3] = 0;
								EqCntr = 0;
								while (1) {
									if (PixVals[EqCntr] != 0) {
										for (int m=0;m<3;m++) {
											//traverse the neighboring pixel values
											if (EqSet[(PixVals[EqCntr])*EqSpan + 0] == 0 && PixVals[m] != 0) {	
												EqSet[(PixVals[EqCntr])*EqSpan + 0] = PixVals[m];
												//adding value for first found eq relationship
											}
											for (int n=0; n<EqSpan; n++) {
												//fix row index by EqCntr, n traverses cols in EqSet									
												if (EqSet[(PixVals[EqCntr])*EqSpan + n] == PixVals[m]) {break;} //if eq relationship already exists, then skip
												else if (EqSet[(PixVals[EqCntr])*EqSpan + n] == 0) { //add eq. relationship
													EqSet[(PixVals[EqCntr])*EqSpan + n] = PixVals[m];
													break;} 
											}	
										}
									}
									EqCntr++; //step EqSet row counter
									if (EqCntr > 2) {break;} //break search once EqSet row counter is beyond the highest object label value
								}
							}
							
						}
						else
						{//within the bulk of the image array
							W = ImArr[i*cols + j-1];
							NW = ImArr[(i-1)*cols + j-1];
							N = ImArr[(i-1)*cols + j];
							NE = ImArr[(i-1)*cols + j+1];
							
							if (W == 0 && NW == 0 && N ==0 && NE ==0)
							{
								ObjArr[i*cols + j] = lbl;
								EqSet[lbl*EqSpan + 0] = lbl;
								lbl++; //step the pixel label entry
							} 
							else if (W != 0 && NW != 0 && N != 0 && NE != 0)
							{
								//determine and choose the min label val among the neighbors
								adjMin = ObjArr[i*cols + j-1]; //assume to start that the W pixel has the min value
								for (int m = 0; m < 4; m++)
								{
									if (m==1) {adjMin < ObjArr[(i-1)*cols + j-1] ? adjMin = ObjArr[(i-1)*cols + j-1] : 1;} //compare to NW pixel
									else if (m==2) {adjMin < ObjArr[(i-1)*cols + j] ? adjMin = ObjArr[(i-1)*cols + j] : 1;} //compare to N pixel
									else if (m==3) {adjMin < ObjArr[(i-1)*cols + j+1] ? adjMin = ObjArr[(i-1)*cols + j+1] : 1;} //compare to NE pixel
								}
								ObjArr[i*cols + j] = adjMin;
							}
							else
							{
								//choose the non-zero and most minimum neighbor
								if (W != 0)
								{
									ObjArr[i*cols + j] == 0 || ObjArr[i*cols + j] > ObjArr[i*cols + j-1] ? ObjArr[i*cols + j] = ObjArr[i*cols + j-1] : 1;
								}
								if (NW != 0)
								{
									ObjArr[i*cols + j] == 0 || ObjArr[i*cols + j] > ObjArr[(i-1)*cols + j-1] ? ObjArr[i*cols + j] = ObjArr[(i-1)*cols + j-1] : 1;
								}
								if (N != 0)
								{
									ObjArr[i*cols + j] == 0 || ObjArr[i*cols + j] > ObjArr[(i-1)*cols + j] ? ObjArr[i*cols + j] = ObjArr[(i-1)*cols + j] : 1;
								}
								if (NE != 0)
								{
									ObjArr[i*cols + j] == 0 || ObjArr[i*cols + j] > ObjArr[(i-1)*cols + j+1] ? ObjArr[i*cols + j] = ObjArr[(i-1)*cols + j+1] : 1;
								}
							}
							//Check and update equivalency relationships, as needed
							PixVals[0] = ObjArr[i*cols + j-1]; 
							PixVals[1] = ObjArr[(i-1)*cols + j-1];
							PixVals[2] = ObjArr[(i-1)*cols + j];
							PixVals[3] = ObjArr[(i-1)*cols + j+1];
							EqCntr = 0;
							while (1) {
								if (PixVals[EqCntr] != 0) {
									for (int m=0;m<3;m++) {
										//traverse the neighboring pixel values
										if (EqSet[(PixVals[EqCntr])*EqSpan + 0] == 0 && PixVals[m] != 0) {	
											EqSet[(PixVals[EqCntr])*EqSpan + 0] = PixVals[m];
											//adding value for first found eq relationship
										}
										for (int n=0; n<EqSpan; n++) {
											//fix row index by EqCntr, n traverses cols in EqSet									
											if (EqSet[(PixVals[EqCntr])*EqSpan + n] == PixVals[m]) {break;} //if eq relationship already exists, then skip
											else if (EqSet[(PixVals[EqCntr])*EqSpan + n] == 0) { //add eq. relationship
												EqSet[(PixVals[EqCntr])*EqSpan + n] = PixVals[m];
												break;} 
										}	
									}
								}
								EqCntr++; //step EqSet row counter
								if (EqCntr > 3) {break;} //break search once EqSet row counter is beyond the highest object label value
							}
						}
					}
				}
			}//end for
			/*
			 cout << "Intermediate Proessing" << endl;
			 for (int ii=0; ii<rows; ii++)
			 {
			 for (int jj=0; jj<cols; jj++)
			 {
			 cout << ObjArr[ii*cols + jj];// << " ";
			 }
			 cout << endl;
			 }
			 */
			cout<<"Done with first pass!"<<endl;
			//Correlate Equivalence Set
			int EqLbl; //equivalent label's row in EqSet
			int EqVal; //equivalent label value
			//int check = 0; //error check
			//Search over the valid eq set entries
			for (int i=1; i<rows*cols; i++) { //first row corresponds to label = 0, trivial case
				for (int j=0; j<EqSpan; j++) {
					if (EqSet[i*EqSpan + j] != i && EqSet[i*EqSpan + j] != 0) {
						EqLbl = EqSet[i*EqSpan + j]; //set new label's row for eq search
						//for loop over eq label row to find new relationships
						for (int k=0; k<EqSpan; k++) {
							EqVal = EqSet[EqLbl*EqSpan + k];
							if (EqVal ==0) {break;}
							//for loop over current label row to add new eq val
							for (int m=0; m<EqSpan; m++) {
								if (EqSet[i*EqSpan + m] == EqVal){break;} //eq val exists
								else if (EqSet[i*EqSpan + m] == 0) {EqSet[i*EqSpan + m] = EqVal; break;} //add new eq val
								else if (m + 1 >= EqSpan) {
									//cout<<"Too many Equivalency Relationships to track!"<<endl;
									//cout<<"There is too much variation in the image, please fix RGB bounds!"<<endl;
									//cin.get();
									//return 0;
									//check=1; //break;
								}
							}
							//if (check==1) {break;}
						}
						//for loop over curr label row to add new relationships to the eq label row
						for (int k=0; k<EqSpan; k++) {
							EqVal = EqSet[i*EqSpan + k];
							if (EqVal ==0) {break;}
							//for loop over eq label row to add new eq val from curr label row
							for (int m=0; m<EqSpan; m++) {
								if (EqSet[EqLbl*EqSpan + m] == EqVal){break;} //eq val exists
								else if (EqSet[EqLbl*EqSpan + m] == 0) {EqSet[EqLbl*EqSpan + m] = EqVal; break;} //add new eq val
								else if (m + 1 >= EqSpan) {
									//cout<<"Too many Equivalency Relationships to track!"<<endl;
									//cout<<"There is too much variation in the image, please fix RGB bounds!"<<endl;
									//cin.get();
									//return 0;
									//check=1; //break;
								}
							}
							//if (check==1) {break;}
						}
					}
					//if (check==1) {break;}
					if (EqSet[i*EqSpan + j] == 0) {break;} //hit last valid entry in the row
				}
				if (EqSet[i*EqSpan + 0] == 0) {break;} //hit last valid entry in the EqSet matrix
			}
			
			// Debug -> show equivalency relationships stored as an array
			/*
			 cout <<endl<< "Eq List (Equivalency Correlation)" << endl;
			 for (int m=0; m<rows*cols; m++)
			 {
			 for (int n=0; n<EqSpan; n++)
			 {
			 cout << EqSet[m*EqSpan + n];// << " ";
			 //cin.get();
			 }
			 cout << endl;
			 if (EqSet[m*EqSpan + 0] == 0 && m>0) {break;}
			 }
			 */
			
			cout<<"Done with eq set!"<<endl;
			//Second pass
			//if (check==0) {
			for (int m=0; m<rows; m++)
			{
				for (int n=0; n<cols; n++)
				{
					if (ObjArr[m*cols + n] != 0) {
						int curval = ObjArr[m*cols + n];
						for (int i=0;i<EqSpan;i++) {
							if (EqSet[curval*EqSpan + i] > 0 && EqSet[curval*EqSpan + i] < curval) {
								ObjArr[m*cols + n] = EqSet[curval*EqSpan + i];
								curval = ObjArr[m*cols + n];
								i = 0; //reset it so the new current value's eq set can be scanned
							}
						}
					}
				}
			}
			/*
			 cout <<endl<< "Objects Identified within Image" << endl;
			 for (int ii=0; ii<rows; ii++)
			 {
			 for (int jj=0; jj<cols; jj++)
			 {
			 cout << ObjArr[ii*cols + jj]; //<< " ";
			 }
			 cout << endl;
			 }
			 */
			
			
			//@@@@@@@@@@@@@@@@@@@@@@@@@@@@
			//Vision-error based processing
			cout << endl;
			cout << "Highest label value = " << lbl << endl;
			int lblStats[lbl*6]; // | lbl val | count | Xmin | Xmax | Ymin | Ymax |
			//initialize stats tracking array
			for (int i=0; i<lbl; i++) {
				for (int j=0; j<6; j++) {
					if (j==0) {lblStats[i*6 + j] = i;}
					else if (j==1) {lblStats[i*6 + j] = 0;} //count
					else if (j==2) {lblStats[i*6 + j] = cols+1;} //Xmin (start high)
					else if (j==3) {lblStats[i*6 + j] = 0;} //Xmax (start low)
					else if (j==4) {lblStats[i*6 + j] = rows+1;} //Ymin (cols inc for low y)
					else if (j==5) {lblStats[i*6 + j] = 0;} //Ymax (cols dec for high y)
				}
			}
			//fill in stats tracking array
			int SVal;
			
			for (int m=0; m<rows; m++) {
				for (int n=0; n<cols; n++) {
					if (ObjArr[m*cols + n] != 0) {
						SVal = ObjArr[m*cols + n];
						lblStats[SVal*6 + 1]++; //increment the count
						if (n < lblStats[SVal*6 + 2]) { lblStats[SVal*6 + 2] = n; } //update Xmin
						if (n > lblStats[SVal*6 + 3]) { lblStats[SVal*6 + 3] = n; } //update Xmax
						if (m < lblStats[SVal*6 + 4]) { lblStats[SVal*6 + 4] = m; } //update Ymin
						if (m > lblStats[SVal*6 + 5]) { lblStats[SVal*6 + 5] = m; } //update Ymax
					}
				}
			}
			/*
			 cout<<"Object Tracking Stats"<<endl;
			 cout<<"| lbl val | lbl count | Xmin | Xmax | Ymin | Ymax |"<<endl;
			 for (int i=0; i<lbl; i++) {
			 for (int j=0; j<6; j++) {
			 cout<<lblStats[i*6 + j]<<" ";
			 }
			 cout<<endl;
			 } 
			 */
			
			//find x_mid
			int maxLbl = 0;
			for (int m=0; m<lbl; m++) {
				if (lblStats[m*6 + 1] > lblStats[maxLbl*6 + 1]) {maxLbl = m;}
			}
			cout<<"max label = "<<maxLbl<<", count = "<< lblStats[maxLbl*6 + 1]<<endl;
			
			float xMid;
			xMid = (lblStats[maxLbl*6 + 2] + lblStats[maxLbl*6 + 3])/2.0;
			cout<<"Xmid = "<<xMid<<endl;
			cout<<"Xmid (ceil) = "<<ceil(xMid)<<endl;
			
			//find Tracking error (rotation)
			//int tErr; //tErr<0 => turn right; tErr>0 => turn left
			tErr = cols/2 - ceil(xMid);
			cout<<"Tracking error = "<< tErr <<endl;
			
			//find Tracking error (translational)
			//int trans;
			//float sizeR; //object size ratio to full frame (to update speed)
			sizeR = static_cast<float>((lblStats[maxLbl*6 + 1]/float(rows*cols)) * 100.0);
			cout<<"size ratio = "<<sizeR<< "%" <<endl;
			
			//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$//	
		}//if	
		
		//****************************************************
		
		
		//==================================================
		//============== End Your Code =====================
		//==================================================
		
		//Send to Webserver For Display
		std::vector<byte> jpg = Nemo::compressImage(img,quality);
		
		//Copy image and RGB info to/from web server
		pthread_mutex_lock( &imageMutex );
		displayImage = jpg;//write to web
		//cmin = color_min;//read from web
		//cmax = color_max;//read from web
		quality = color_quality;
		pthread_mutex_unlock( &imageMutex );
		
}


void InitCamera() {
	pthread_mutex_init(&imageMutex, NULL);
	
	std::cout << "Start Vison server..." << std::endl;
	std::cout << "Please go to http://192.168.1.1X:8080/" << std::endl;
	
	//open a web server on http://192.168.1.X:8080
	struct mg_context *ctx;
	const char *options[] = {
		"listening_ports", "8080", 
		"document_root","html",
		NULL};
	ctx = mg_start(&callback, NULL, options);
	
	
	//wait until camera ready
	bool cameraReady = false;
	do{
		pthread_mutex_lock( &imageMutex );
		cameraReady = camera_init;
		pthread_mutex_unlock( &imageMutex );
	}while(!cameraReady);
	
	
	//Start streaming video image
	PixRGB<byte> cmin = PixRGB<byte>(0,0,0);
	PixRGB<byte> cmax = PixRGB<byte>(255,255,255);
	int quality = 10;
}


void PID_control(string object) {
	//desired goal sensor msmt
	set_val = 0.0; //rotation from center of objects in image

	if (object == "Ball") {
		cout << "in PID_control, so sizeR for Ball is " << sizeR << endl;
		if (sizeR >=0.10 && sizeR <= 2.75) {
			cout<<"speed up"<<endl;
			trans = 5 + sizeR*10;
		}
		else if (sizeR > 2.75 && sizeR < 10) {
			cout<<"nominal speed"<<endl;
			trans = 5 + sizeR;
		}
		else {
			cout<<"stop"<<endl;
			trans = 0;
		}
	}
	else if (object == "Goal") {
		cout << "in PID_control, so sizeR for Goal is " << sizeR << endl;
		if (sizeR >=0.01 && sizeR <= 10) {
			cout<<"speed up"<<endl;
			trans = 5 + sizeR*10;
		}
		else if (sizeR > 10 && sizeR < 75) {
			cout<<"nominal speed"<<endl;
			trans = 5 + sizeR;
		}
		else {
			cout<<"stop"<<endl;
			trans = 0;
		}
	}
	
	sensor_val = tErr; //get new sensor msmt
	cout << "sensor_val is " << sensor_val << ", which should be " << tErr << endl;
	//Updating PID error terms
	curr_error = set_val - sensor_val; //P
	integral_e = prev_error_total + curr_error; //I
	de = curr_error - prev_error; //D
	//Updating error history data
	prev_error = curr_error;
	prev_error_total += prev_error; //increment sum of errors
	//set turn magnitude
	rotation = static_cast<float>((Kp*curr_error + Ki*integral_e + Kd*de)/50.0);
	//rotation = 0; //special case for testing translational motion
	std::cout << "Rotation: " << rotation << std::endl;
	M1 = 79 - trans + rotation; // Corresponds to pin 0, the right servo
	M2 = 80 + trans + rotation; // Corresponds to pin 1, the left servo
	std::cout << "Right motor: " << M1 << "; Left Motor: " << M2 << std::endl;
	Nemo::setServo(0, M1); //set right servo
	Nemo::setServo(1, M2); //set left servo
	//usleep(500000);
}

void update_state(double new_state) {
	prevState = currState;
	currState = new_state;
}

void echo_state () {
cout << "Previous State = " << prevState << endl;	
cout << "Current State = " << currState <<endl;
}

bool GotBall() {
	float sonar_msmt;
	while(1) {
		sonar_msmt = Nemo::getSonar(BallSonarPin);
		cout << "value for ball detecting sonar is " << sonar_msmt << endl;
		if (sonar_msmt < 10 || sonar_msmt > 12) {
			cout<<"sonar found ball!!"<<endl;
			return true;
			break;
		}
		else {
			cout<<"lost the ball! Sonar did not detect!"<<endl;
			return false;
			break;
		}
		wait(0.25);
	}
}

void startDribbler () {
	Nemo::setServo(dribPin, 101); //start dribbler
}

void stopDribbler () {
	Nemo::setServo(dribPin, 85); //stop dribbler
}


//####################################################
//####################################################
//Declare states
void DriveToBall() { //State 1.1 - Drive to ball
	bool findPersist = false;
	while (1) {
		cout << "we're in state " << currState << endl;
		take_image("Ball");
		cout << "in DriveToBall(), and sizeR of Ball is " << sizeR << endl;
		if (sizeR > 0.10) {
			findPersist = true;
			PID_control("Ball");
		}
		else {
			if (findPersist == true) {
				blind_drive("Forward", 2);
				if (GotBall()) {
					update_state(1.2); //transition to drive with ball to opp's goal
					break;
				}
				else {
					findPersist = false;
					blind_drive("Backward", 0.7);
				} //end if
			}
			else {
				update_state(2); //transition to search for ball
				break; //break while loop
			}
		}
	}
}

void DriveWithBall() { //State 1.2 - Drive with ball to opponent's goal
	//bool pMsmt = false; //pressure sensor msmt boolean
	//AvoidObstacle();
	//better way to determine CW or CCW?? -> min(current heading - absolute heading)
	Turn(opp_goal, "CCW");
	wait(0.5);
	while (1) {
		cout << "we're in state " << currState << endl;
		if (GotBall()) {
			AvoidObstacle();
			if (GotBall()) {
				take_image("Goal");
				cout << "in DriveWithBall(), and sizeR of Goal is " << sizeR << endl;
				if (sizeR > 22.5 && sizeR < 50 && abs(tErr) < 20) {
					stop_drive();
					update_state(3); //Transition to shoot
					break;
				}
				else if (sizeR < 0.001) {
					//camera gamma mode protection
					blind_drive("Backward",4);
				}
				else {
					PID_control("Goal");
				}
			}
		}
		else {
			update_state(1.1); //transition to drive to ball
			break;
		}
		
	} //while
}

void DriveToGoal() { //State 1.3 - Drive to goal without ball
	AvoidObstacle();
	Turn(own_goal, "CW");
	wait(0.5);
	while(1) {
		cout << "we're in state " << currState << endl;
		take_image("Goal");
		cout << "in DriveToGoal(), home goal, and sizeR of Goal is " << sizeR << endl;
		if (abs(tErr) < 35 && sizeR > 35) {
			Turn(opp_goal,"CW");
			wait(0.5);
			update_state(2); //transition to search for ball
			break;
		}
		else if (sizeR < 0.001) {blind_drive("Backward",4);}
		else {
			PID_control("Goal");
		}
	} //while
}

void AvoidObstacle() { //Avoid obstacle - for use in states 1.2 & 1.3
	float rdg;
	rdg = Nemo:;getSonar(frSonarPin);
	if (rdg <= 5) {
		blind_drive("Backward", 3);
	}
	else if (rdg >5 && rdg <= 20){
		rotation = rotation - 5;
	}
}

void SearchBall() { //State 2 - Search for the ball
	bool ballFound = false;
	int rotationsLeft = 12;
	string dirToGo = "CW";

	take_image("Ball"); //take first straight ahead image
	
	// Assume that the ball is up for grabs
	while ( (rotationsLeft > 0) && (!ballFound) )
	{
		//int counter = 0;
		//while(counter < 5) {
		//take_image("Ball");
		//wait(0.5);
		//counter++;
		//}

		cout << "in SearchBall(), and sizeR of Ball is " << sizeR << endl;
		if(sizeR > 0.05 || GotBall()) {
			ballFound = true;
			break;
		}

		else {
			if(1) { // Nemo::getSonar() > ) { //there's no object in that direction
				Turn( (Nemo::getCompass(cPin) + 30), dirToGo);
				take_image("Ball");
				//if (dir == "CCW") {
					//Nemo::setServo(lsPin, 74); // TURN LEFT/counter clockwise 63
					//Nemo::setServo(rsPin, 78); // TURN LEFT/counter clockwise 74
					//wait(1);
				
				//}

				//else {
				//Nemo::setServo(lsPin, 81);  // TURN RIGHT/clockwise 85
				//Nemo::setServo(rsPin, 92);	// TURN RIGHT/clockwise 96
				//wait(1);
				//}

				wait(0.5);
			}
			else {
				if(dirToGo == "CW") {
					dirToGo = "CCW";
					cout << "changing the direction to " << dirToGo << endl;
				}
				else if(dirToGo == "CCW") {
					dirToGo = "CW";
					cout << "changing the direction to " << dirToGo << endl;
				}
			}
			rotationsLeft--;
		}
	}

	if(ballFound) {
		if (GotBall()) {update_state(1.2);}
		else {update_state(1.1);}
	}

	// If you get to this point, the opponent must have the ball, so go back to our goal to defend
	else {
		update_state(1.3);
	}		
}

void Shoot(){ //State 3 - Shoot the ball
	int value = 80;
	cout << "we're in state " << currState << endl;
	stop_drive();
	wait(1);
	stopDribbler(); //stop the dribbler
	wait(0.25);
	cout << "KICK!!!" << endl;
	Nemo::setServo(kickPin, 150); //deploy the kicker
	wait(2);
	Nemo::setServo(kickPin, 0); //reset the kicker
	wait(2);

	//startDribbler(); //restart the dribbler
	blind_drive("Backward", 2); //blind drive backwards
	update_state(1.1); // transition to drive to ball
}

//####################################################
//####################################################
//####################################################
int main()
{
	currState = 1.1; //always start at drive to ball for kick-off
	InitCamera(); //intialize camera

	/*startDribbler();
	while(1) {
	blind_drive("Forward", 3);
	}
	cin.get();
	*/

	// Take junk images.
	int counter = 0;
	while(counter < 5) {
		take_image("Ball");
		usleep(50000);
		counter++;
		cout << "took image" << endl;
	}

	/*while(1) {
		take_image("Goal");
	}*/

	/*trans = 5.0;
	M1 = 79 - trans; // Corresponds to pin 0, the right servo
	M2 = 80 + trans; // Corresponds to pin 1, the left servo
	//std::cout << "Right motor: " << M1 << "; Left Motor: " << M2 << std::endl;
	Nemo::setServo(0, M1); //set right servo
	Nemo::setServo(1, M2); //set left servo
	while(1) {AvoidObstacle();}
	*/

	//cout << "before SM; about to call SearchBall" << endl;
	/*Nemo::setServo(kickPin, 0); //reset kicker
	startDribbler();
	wait(1);
	//DriveToBall();
	//DriveWithBall();
	Shoot();
	echo_state();
	stop_drive();
	cin.get();*/
	//cout << "just searched for ball" << endl;

	/*while(1) {
		float rdg;
		//rdg = Nemo::getCompass(cPin);
		rdg = Nemo::getSonar(BallSonarPin);
		cout<<rdg<<endl;
	}*/

	/*while(1) {
		SearchBall();
		echo_state();
		cin.get();
	}*/

	//state machine
	if (1) {
		cout << "in state machine" << endl;
		Nemo::setServo(kickPin, 0); //reset kicker
		startDribbler(); //start dribbler
		while(1) {
			echo_state();
			if (currState == 1.1) {
				DriveToBall();
			}
			else if(currState == 1.2) {
				// Go to opponent's goal to shoot the ball.
				wait(2); //time to control dribble
				DriveWithBall();
			}
			else if(currState == 1.3) {
				// Go to home goal for defense.
				cout << "protecting goal" << endl;
				stop_drive();
				DriveToGoal();
			}
			else if(currState == 2) {
				//search for ball
				stop_drive();
				SearchBall();
			}
			else if(currState == 3) {	
				Shoot();
				//cin.get();
			}
			AvoidObstacle();
		}
		cout << "done with SM" << endl;
	}
	
	return 0;
}
