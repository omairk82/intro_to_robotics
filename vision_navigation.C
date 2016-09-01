#include <iostream>
#include <nemo/Nemo.H>
#include <Util/mongoose.h>
#include <Image/DrawOpsSimple.H>
#include <math.h>

#define red   PixRGB<byte>(255,0,  0)
#define green PixRGB<byte>(0,  255,0)
#define blue  PixRGB<byte>(0,  0, 255)

using namespace std;

//####################################################
static std::vector<byte> displayImage;
static pthread_mutex_t imageMutex;
static bool camera_init = false;
static PixRGB<byte> color_min = PixRGB<byte>(0,0,0);
static PixRGB<byte> color_max = PixRGB<byte>(255,255,255);
static int color_quality = 10;
// ######################################################################
static void get_qsvar(const struct mg_request_info *request_info,
                      const char *name, char *dst, size_t dst_len) {
  const char *qs = request_info->query_string;
  mg_get_var(qs, strlen(qs == NULL ? "" : qs), name, dst, dst_len);
}
//####################################################
static void *callback(enum mg_event event,
    struct mg_connection *conn) {
  const struct mg_request_info *request_info 
              = mg_get_request_info(conn);
  if (event == MG_NEW_REQUEST) {
    std::string uri(request_info->uri);
    std::string method(request_info->request_method);
    if (uri == "/image.jpg")
    {
      int quality = 10; char cquality[5];
      get_qsvar(request_info, "quality", cquality, sizeof(cquality));
      quality =  cquality[0] == '\0' ? 10 : atoi(cquality);

      std::vector<byte> imgData;

      pthread_mutex_lock( &imageMutex );
      imgData = displayImage;
      color_quality = quality;
      pthread_mutex_unlock( &imageMutex );


      mg_write(conn, &imgData[0], imgData.size());
      LINFO("JPG Image Size %d quality %d",(int)imgData.size(),quality);






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
    }else if(uri == "/servo"){//for /compass?pin=1&position=90
      int pin = -1;char cpin[3];
      int position = -1;char cposition[5];
      get_qsvar(request_info, "pin", cpin, sizeof(cpin));
      get_qsvar(request_info, "position", cposition, sizeof(cposition));
      pin = cpin[0] == '\0' ? -1 : atoi(cpin);
      position = cposition[0] == '\0' ? -1 : atoi(cposition);
      Nemo::setServo(pin,position);
      mg_printf(conn,"Set Servo position %3d in digital pin %2d ",position,pin);

    }else if(uri == "/compass"){//for /compass?pin=5
      int pin = -1;char cpin[3];
      get_qsvar(request_info, "pin", cpin, sizeof(cpin));
      pin = cpin[0] == '\0' ? -1 : atoi(cpin);

      LINFO("Get Compass Reading from pin %d",pin);
      float compass = Nemo::getCompass(pin);//return 0-359.99 deg
      LINFO("Compass Reading is %f",compass);

      mg_printf(conn,"Compass reading %6.2f degree",compass);

    }else if(uri == "/sonar"){//for /sonar?pin=5
      int pin = -1;char cpin[3];
      get_qsvar(request_info, "pin", cpin, sizeof(cpin));
      pin = cpin[0] == '\0' ? -1 : atoi(cpin);
      float distance = Nemo::getSonar(pin);//return 0.0-370.0 cm
      mg_printf(conn,"Sonar reading %6.2f cm",distance);

    }else if(uri == "/battery"){
      float battery1 = Nemo::getBatteryVoltage(BATTERY_ZERO);
      float battery2 = Nemo::getBatteryVoltage(BATTERY_ONE);
      mg_printf(conn,"PWR1:%5.2fV,PWR2:%5.2fV",battery1,battery2);

    }else if(uri == "/speaker"){//for /speaker?frequency=1000
      int freq = -1;char cfreq[5];
      get_qsvar(request_info, "frequency", cfreq, sizeof(cfreq));
      freq = cfreq[0] == '\0' ? -1 : atoi(cfreq);
      Nemo::setSpeaker(freq);
      mg_printf(conn,"Set Speaker Frequency to %4dHz",freq);

    }else if(uri == "/led"){//for /compass?led=1&state=true
      int led = -1;char cled[3];
      bool state = false;char cstate[8];
      get_qsvar(request_info, "led", cled, sizeof(cled));
      get_qsvar(request_info, "state", cstate, sizeof(cstate));
      led = cled[0] == '\0' ? -1 : atoi(cled);
      state = cstate[0] == 't' ? true : false;
      Nemo::setLED(led,state);
      mg_printf(conn,"Set Led:%d to [%s] ",led,cstate);



    }else if(uri == "/rgb"){     //for /rgb?rmin=1&rmax=255....
      int rmin = -1;char crmin[5];
      int rmax = -1;char crmax[5];
      int gmin = -1;char cgmin[5];
      int gmax = -1;char cgmax[5];
      int bmin = -1;char cbmin[5];
      int bmax = -1;char cbmax[5];

      get_qsvar(request_info, "rmin", crmin, sizeof(crmin));
      get_qsvar(request_info, "rmax", crmax, sizeof(crmax));
      get_qsvar(request_info, "gmin", cgmin, sizeof(cgmin));
      get_qsvar(request_info, "gmax", cgmax, sizeof(cgmax));
      get_qsvar(request_info, "bmin", cbmin, sizeof(cbmin));
      get_qsvar(request_info, "bmax", cbmax, sizeof(cbmax));

      rmin= crmin[0] == '\0' ? 0 : atoi(crmin);
      rmax= crmax[0] == '\0' ? 0 : atoi(crmax);
      gmin= cgmin[0] == '\0' ? 0 : atoi(cgmin);
      gmax= cgmax[0] == '\0' ? 0 : atoi(cgmax);
      bmin= cbmin[0] == '\0' ? 0 : atoi(cbmin);
      bmax= cbmax[0] == '\0' ? 0 : atoi(cbmax);

      pthread_mutex_lock( &imageMutex );
      color_min = PixRGB<byte>(rmin,gmin,bmin);
      color_max = PixRGB<byte>(rmax,gmax,bmax);
      pthread_mutex_unlock( &imageMutex );

      mg_printf(conn,"Got Min:[%d,%d,%d]",rmin,gmin,bmin);
      mg_printf(conn,"Got Max:[%d,%d,%d]",rmax,gmax,bmax);

    }else{
      return NULL;
    }
    return (void *)"";  // Mark as processed
  } else {
    return NULL;
  }
}
//####################################################
//####################################################
//####################################################
int main()
{
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

  int ox = 0,oy = 0,radius = 10; //for disk drawing code

//$$$ PID settings
float set_val, sensor_val, curr_error, rotation, M1, M2, integral_e, de;
float prev_error_total = 0;
float prev_error = 0;
//Set gains Kp, Ki, Kd:
float Kp = 3.0, Ki = 0.0, Kd = 0.0;
//desired goal sensor msmt
set_val = 0.0; //rotation from center of objects in image
//$$$

  while(1)
  {
    if(cameraReady)
    {
      Image<PixRGB<byte> > img = Nemo::grabImage();//Get image from usb camera
      int w = img.getWidth();
      int h = img.getHeight();
      int ImArr[w*h];


      //==================================================
      //============= Put Your Code Here =================
      //==================================================

	/*
      //Draw a disk and shift down every frame
      ox = (ox+radius/2) >= w ? 0 :ox + 1;
      oy = (oy+radius/2) >= h ? 0 :oy + 1;
      Point2D<int> center = Point2D<int>(ox, oy);

      //Check Image/DrawOpsSimple.H for more drawing function
      drawDisk(img,center,radius,cmax);
	*/
	if (1) {
	for (int y=0; y<h; y++) {
		for(int x = 0; x < w; x++) {
			PixRGB<byte> px = img.getVal(Point2D<int> (x,y));
			byte r,g,b;
			px.getRGB(r,g,b);
			byte r_max, g_max, b_max;
			cmax.getRGB(r_max, g_max, b_max);
			byte r_min, g_min, b_min;
			cmin.getRGB(r_min, g_min, b_min);
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
	
	if (0) {
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
		int tErr; //tErr<0 => turn right; tErr>0 => turn left
		tErr = cols/2 - ceil(xMid);
		cout<<"Tracking error = "<< tErr <<endl;
	
		//find Tracking error (translational)
		int trans;
		float sizeR; //object size ratio to full frame (to update speed)
		sizeR = (lblStats[maxLbl*6 + 1]/float(rows*cols)) * 100.0;
		cout<<"size ratio = "<<sizeR<< "%" <<endl;
		if (sizeR >=0.01 && sizeR <= 2.75) {
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
		//cin.get();
		cout<<"Done with second pass!"<<endl;

		//$$$PID implementation$$$
		sensor_val = tErr; //get new sensor msmt
		//usleep(50000);
		//Updating PID error terms
	  	curr_error = set_val - sensor_val; //P
	 	integral_e = prev_error_total + curr_error; //I
	 	de = curr_error - prev_error; //D
		//Updating error history data
	 	prev_error = curr_error;
		prev_error_total += prev_error; //increment sum of errors
		//set turn magnitude
		rotation = (Kp*curr_error + Ki*integral_e + Kd*de)/50;
		//rotation = 0; //special case for testing translational motion
		std::cout << "Rotation: " << rotation << std::endl;
		M1 = 79 - trans + rotation; // Corresponds to pin 0, the right servo
		M2 = 80 + trans + rotation; // Corresponds to pin 1, the left servo
		std::cout << "Right motor: " << M1 << "; Left Motor: " << M2 << std::endl;
	  	Nemo::setServo(0, M1); //set right servo
		Nemo::setServo(1, M2); //set left servo
		//usleep(500000);
	
	//std::cin.get();

	//}//if
	
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
      cmin = color_min;//read from web
      cmax = color_max;//read from web
      quality = color_quality;
      pthread_mutex_unlock( &imageMutex );

    }else{
      LINFO("No Camera");
    }
  }
  return 0;
}
