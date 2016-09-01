#include <iostream>
#include <fstream>
#include <nemo/Nemo.H>
#include <Image/DrawOpsSimple.H>
#include <Util/mongoose.h>
#include "Particle.H"
#include "Util.H"
 
#define NUM_PARTICLES 100
#define MAP_WIDTH  200
#define MAP_HEIGHT 200

PixRGB<byte> const red(255,0,  0);
PixRGB<byte> const green(0,  255,0);
PixRGB<byte> const blue(0,  0, 255);

using namespace std;

  // PIN SETTINGS
  // MOTOR LEFT PIN 0
  // MOTOR RIGHT PIN 1
  // SERVO PIN 2
  // SONAR PIN 3
  // COMPASS PIN 4 and 5,6,7

// *************************************************************************
// TO DO 1: UPDATE THE COMPASS VALUES OF YOUR ROBOT (NORTH, EAST, SOUTH, WEST)
float compass_angle[4] = {343.0, 91.0, 185.0, 269.0}; // FOR TURNING
// *************************************************************************
double exact_angle[4] = {270.0, 0.0, 90.0, 180.0};
int robot_direction = -1;

//####################################################
static std::vector<byte> displayImage;
static std::vector<byte> mapImage;
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

    if (uri == "/map.jpg"){
      //get map image
      std::vector<byte> imgData;
      pthread_mutex_lock( &imageMutex );
      imgData = mapImage;
      pthread_mutex_unlock( &imageMutex );

      //display on the web
      mg_write(conn, &imgData[0], imgData.size());


    } else if (uri == "/image.jpg") {
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



    }else if(uri == "/rgb"){//for /rgb?rmin=1&rmax=255....
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

// #####################################################################
// #####################################################################
// #####################################################################
// Low variance sampler
// Use a roulette wheel to probabilistically duplicate particles with high weights,
// and discard those with low weights. A Particle is some structure that has
// a weight element w. The sum of all weights in oldParticles should equal 1.
std::vector<Particle> resampleParticles(std::vector<Particle> oldParticles){
  	std::vector<Particle> newParticles;
  	//Calculate a Cumulative Distribution Function for our particle weights
  	std::vector<double> CDF;
  	CDF.resize(oldParticles.size());
  	CDF[0] = ((Particle)oldParticles[0]).getWeight();
  	for(uint i=1; i<CDF.size(); i++)
    		CDF[i] = CDF[i-1] + oldParticles[i].getWeight();
	//Loop through our particles as if spinning a roulette wheel.
	//The random u that we start with determines the initial offset
	//and each particle will have a probability of getting landed on and
	//saved proportional to its posterior probability. If a particle has a very large
	//posterior, then it may get landed on many times. If a particle has a very low
	//posterior, then it may not get landed on at all. By incrementing by
	// 1/(numParticles) we ensure that we don't change the number of particles in our
	// returned set.
	uint i = 0;
	double u = randomDouble()* 1.0/double(oldParticles.size());
	for(uint j=0; j < oldParticles.size(); j++){
		while(u > CDF[i])
			i++;
		Particle p = oldParticles[i];
		p.setWeight(1.0/double(oldParticles.size()));
		newParticles.push_back(p);
		u += 1.0/double(oldParticles.size());
	}
  return newParticles;
}
 
// #####################################################################
// #####################################################################
void direction_update() {

	float compass_direction = 0.0;
	for (int i = 0; i < 5; i++){
		compass_direction += Nemo::getCompass(4);
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
}

void updateProbability(std::vector<Particle> &particles,
                        int direction, double distance){

	float total_probabilities = 0.0;
	float new_weight = 0.0;
	float old_probabilities = 0.0;
	float new_probability = 0.0;
	double map_distance = 0.0;
	double sonar_variance = 10.0;

	// update all the particle probabilities
	for (uint i=0; i<particles.size(); i++){
		Point2D<int> pos = particles[i].getPosition();
		// use heading to calculate the map distance from particle to wall.
		map_distance =  distToEdge(direction,Point2D<double>(pos));
		printf("Sonar: %f, Map_Distance: %f, direction: %d, Point(%d,%d), Weight: %f\n", distance, map_distance, direction, pos.i, pos.j, particles[i].getWeight());
		// Compute new probability using measured distance , map distance and sonar variance
		new_probability =  getProbability(distance, sonar_variance, map_distance);
		// update each probability and compute total probabilities
		old_probabilities = particles[i].getWeight();
		new_weight = old_probabilities * new_probability;
		particles[i].setWeight(new_weight);
		total_probabilities += new_weight;
	}

	// Normalize all probabilities
	for (uint i=0; i<particles.size(); i++){
		//normalized probability = probability / total probabilities
		particles[i].setWeight(particles[i].getWeight()/total_probabilities);
	}
 
}

void sense(std::vector<Particle> &particles, int currentDirection){
 	
	// taking sonar reading
	float sonar = 0.0;
	for (int i = 0; i < 5; i++){
		sonar += (double)Nemo::getSonar(3);
		usleep(100000);
	}
	sonar = sonar/5.0;
	// Update Probability
	updateProbability(particles, currentDirection, sonar);
	// Resample Particles
	particles = resampleParticles(particles);
}
 
void turnleft() {
	// *************************************************************************
	// UPDATE THE LEFT TURN OF YOUR ROBOT
	robot_direction = robot_direction - 1;
	if (robot_direction < 0)
		robot_direction = 3;
	float angleFROM = (Nemo::getCompass(4))*M_PI/180;
	float angleTO = compass_angle[robot_direction]*M_PI/180;
	float angletoturn = atan2(sin(angleTO - angleFROM),cos(angleTO - angleFROM))*180/M_PI;
	while(abs(angletoturn) > 15) {
		Nemo::setServo(0, 63); // TURN LEFT
		Nemo::setServo(1, 74); // TURN LEFT
		angleFROM = (Nemo::getCompass(4))*M_PI/180;
		angletoturn = atan2(sin(angleTO - angleFROM),cos(angleTO - angleFROM))*180/M_PI;
		cout << "angle to turn: " << angletoturn << ", and compass reading: " << Nemo::getCompass(4) << endl;
	}
	Nemo::setServo(0,79); // STOP TURNING
	Nemo::setServo(1,80); // STOP TURNING
	// *************************************************************************
}

void turnright() {
	// *************************************************************************
	// UPDATE THE RIGHT TURN OF YOUR ROBOT
	robot_direction = robot_direction + 1;
	if (robot_direction > 3)
		robot_direction = 0;
	float angleFROM = (Nemo::getCompass(4))*M_PI/180;
	float angleTO = compass_angle[robot_direction]*M_PI/180;
	float angletoturn = atan2(sin(angleTO - angleFROM),cos(angleTO - angleFROM))*180/M_PI;
	while(abs(angletoturn) > 15) {
		Nemo::setServo(0, 85);  // TURN RIGHT
		Nemo::setServo(1, 96);	// TURN RIGHT
		angleFROM = (Nemo::getCompass(4))*M_PI/180;
		angletoturn = atan2(sin(angleTO - angleFROM),cos(angleTO - angleFROM))*180/M_PI;
		cout << "angle to turn: " << angletoturn << ", and compass reading: " << Nemo::getCompass(4) << endl;
	}
	Nemo::setServo(0,79); // STOP TURNING
	Nemo::setServo(1,80); // STOP TURNING
	// *************************************************************************
} 

void moveforward() {
	// *************************************************************************
	// UPDATE THE FORWARD MOVING VALUES OF YOUR ROBOT
	Nemo::setServo(0,74);
	Nemo::setServo(1,85);
	usleep(1500000);
	Nemo::setServo(0,79);	// STOP MOVING
	Nemo::setServo(1,80);	// STOP MOVING
}


// #####################################################################
// #####################################################################
// #####################################################################
int main()
{

	/*while(1) {
		cout << "Compass reading is: " << Nemo::getCompass(4) << endl;
		usleep(500000);
	}*/

  pthread_mutex_init(&imageMutex, NULL);

	std::cout << "Start Vison server..." << std::endl;
	std::cout << "Please go to http://192.168.1.1X:8080/" << std::endl;


  //============== Web Server Initialization =======================
  //open a web server on http://192.168.1.X:8080
  struct mg_context *ctx;
  const char *options[] = {
    "listening_ports", "8080", 
    "document_root","html",
    NULL};
  ctx = mg_start(&callback, NULL, options);


  //============== Camera Initialization ============================
  //wait until camera ready
  bool cameraReady = false;
  //Start streaming video image
  int quality = 10;
  PixRGB<byte> cmin = PixRGB<byte>(0,0,0);
  PixRGB<byte> cmax = PixRGB<byte>(255,255,255);
  
  
  //============== Particle Filter Initialization ===================
  Image<PixRGB<byte> > mapImg = Image<PixRGB<byte> >(Dims(MAP_WIDTH,MAP_HEIGHT), ZEROS);
  std::vector<Particle> particles;
  for(int i = 0 ; i < NUM_PARTICLES ; i++){
    Particle p = Particle(Point2D<int>(rand()%MAP_WIDTH,rand()%MAP_HEIGHT), 1/NUM_PARTICLES);
    //Particle p = Particle(Point2D<int>(MAP_WIDTH/2,MAP_HEIGHT/2), 1.0/NUM_PARTICLES);
    particles.push_back(p);
  }
  double angle = 0.0;
  float compass = 0.0;
  float sonar = 0.0;
  
  while(1)
  {
    if(cameraReady)
    {
      Image<PixRGB<byte> > img = Nemo::grabImage();//Get image from usb camera
      int w = img.getWidth();
      int h = img.getHeight();

      //Put camera vision code below
      //==================================================
      //============== Start Your Vision Code ============
      //==================================================

      //==================================================
      //============== End Your Vision Code ==============
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
      pthread_mutex_lock( &imageMutex );
      cameraReady = camera_init;
      pthread_mutex_unlock( &imageMutex );
    }

    //==================================================
    //=========== Start Your Particle Filter Code ======
    //==================================================
    
    // Take keyboard input to make robot move
    cout << endl << "Enter Choice (1=Left, 2=Right, 3=Forward, 5=Sense): ";
    char input;
    cin >> input;

    switch (input){

      case '1': // Rotate Left 90 degrees
		// angle-=90.0;
		turnleft();
		direction_update(); 
		angle = exact_angle[robot_direction];
        	break;
      case '2': // Rotate Right 90 degrees
        	// angle+=90.0;
		turnright();
		direction_update(); 
		angle = exact_angle[robot_direction];
        	break;
      case '3': // move forward 21 cm (1.5 sec)
        	moveforward();
		direction_update(); 
		angle = exact_angle[robot_direction];
        	// update particle movement models
        	for (uint i=0; i<particles.size(); i++){
          		// move particle given ANGLE, DISTANCE, VARIANCE
  			particles[i].moveParticle(angle,21.0,10.0);
        	}
        	break;
      case '5':  // take sensor readings and resample particles
		direction_update(); 
		angle = exact_angle[robot_direction];
		sense(particles, robot_direction);
        	break;
      default:
        	cout << "No valid input";
    }//end switch

    //compute current location
    double xx = 0.0,yy = 0.0;
    for (uint i=0; i<particles.size(); i++){
      // move particle given ANGEL, DISTANCE, VARIANCE
      Point2D<int> pos = particles[i].getPosition();
      double w = particles[i].getWeight();
      xx += pos.i*w;
      yy += pos.j*w;

    }
    // PRINT CURRENT POSITION
    LINFO("Current Pos(%f,%f)",xx,yy);
    // DRAW PARTICLES
    Image<PixRGB<byte> > img = drawParticle(mapImg,particles);
    // DRAW POSITION OF ROBOT
    Point2D<int> pos(xx,yy);
    double angRad = angle*M_PI/180.0;
    drawDisk(img,pos,10,blue);
    drawLine(img,pos,angRad,20,green,2);
    Point2D<int> head(pos.i+10*cos(angRad),pos.j+10*sin(angRad));
    drawDisk(img,head,2,red);


    //display map image on the web
    std::vector<byte> mapJpg = Nemo::compressImage(img,80);
    pthread_mutex_lock( &imageMutex );
    mapImage = mapJpg;//write to web
    pthread_mutex_unlock( &imageMutex );
    usleep(100000);//0.1 seconds
    

    //==================================================
    //=========== End Your Particle Filter Code ========
    //==================================================




  }






  return 0;
}

