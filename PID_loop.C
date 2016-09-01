#include <iostream>
#include <nemo/Nemo.H>

int main()
{
	//float Nemo::getSonar(0);
	std::cout << "Hello, world!" << std::endl;

  	float set_val, sensor_val, curr_error, rotation, M1, M2, integral_e, de;
  	float prev_error_total = 0;
 	float prev_error = 0;

	// Set gains Kp, Ki, Kd:
	float Kp = 2.2, Ki = 0, Kd = 0;

	//desired goal sensor msmt
	set_val = 30.0; //distance from wall measured by sonar in this case
	
  while(1)
  {
	sensor_val = Nemo::getSonar(3); //get new sensor msmt
	std::cout << "getSonar returns " << sensor_val << std::endl;

	//Updating PID terms
  	curr_error = set_val - sensor_val; //P
 	integral_e = prev_error_total + curr_error; //I
 	de = curr_error - prev_error; //D
	
	//Updating error history data
 	prev_error = curr_error;
	prev_error_total += prev_error; //increment sum of errors

	//set turn magnitude
	rotation = Kp*curr_error + Ki*integral_e + Kd*de;

	M1 = 79.0 - 4.0 - rotation; // Corresponds to pin 0, the right servo
	M2 = 80.0 + 4.0 + rotation; // Corresponds to pin 1, the left servo
	  
	/*
	if(M1 < 0)
	{
		M1 = 0;
	}

	if(M2 > 180)
	{
		M2 = 180;
	}
	std::cout << "Left motor: " << M1 << "; Right Motor: " << M2 << std::endl;
	*/
	  
  	Nemo::setServo(0, M1); //set right servo
	Nemo::setServo(1, M2); //set left servo
  }
	
	return 0;
}

