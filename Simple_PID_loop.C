#include <iostream>
#include <nemo/Nemo.H>
#include <math.h>

using namespace std;

int main()
{
	std::cout << "Hello, World!!" << std::endl;
	std::cout << "And with two motors!" << std::endl;


	//while(1)
	//{
    
  	Nemo::setServo(0, 79);    // Our right wheel
  	//usleep(5000000);        // Wait for .5 seconds
  	Nemo::setServo(1, 80);    // Our left wheel
	usleep(5000000);          // Wait for .5 seconds
  	/*Nemo::setServo(1, 0);  
	usleep(5000000);          // Wait for .5 seconds
  	Nemo::setServo(0, 90);  
	usleep(5000000);          // Wait for .5 seconds
  	Nemo::setServo(1, 90);  
	usleep(5000000);          // Wait for .5 seconds
	*/

	//}

	// Initializing...
	float left_measurement, right_measurement, differenceNS, differenceWE, distance_fromfront, distance_moved, left_meas, right_meas;
	float total_meas, total_measurement;
	bool keep_rotating = true;

	//cout << "Compass reads " << Nemo::getCompass(3) << endl;

	while(keep_rotating)
	{
		cout << "At the top of while loop" << endl;
		// Read sonar value.
		distance_fromfront = Nemo::getSonar(3);
		cout << "Distance away from front = " << distance_fromfront << endl;
		distance_moved = distance_fromfront - 5;

		while(distance_fromfront > distance_moved)
		{
			// Keep driving forward.
			Nemo::setServo(0, 74);    // Our right wheel
  			Nemo::setServo(1, 85);    // Our left wheel
			distance_fromfront = Nemo::getSonar(3);
		}

		// *** Time to check on L and R!
		cout << "car stops moving" << endl;
		Nemo::setServo(0, 79);
		Nemo::setServo(1, 80);

		// Rotate the sonar to its left.
		Nemo::setServo(2, 180);  
		usleep(1000000);          // Wait for .5 seconds

		left_measurement = Nemo::getSonar(3);
	
		// Rotate the sonar to its right.
		Nemo::setServo(2, 0);
		usleep(1000000);

		right_measurement = Nemo::getSonar(3);

		// Set the sonar to face the front again.
		Nemo::setServo(2, 82);
		usleep(1000000);

		// Determine which side (L or R) of the wall it's farthest from.
		differenceWE = left_measurement - right_measurement;

		cout << "differenceWE = " << differenceWE << endl;
		cout << "Left measurement = " << left_measurement << "; right measurement = " << right_measurement << endl;
		total_measurement = left_measurement + right_measurement;
		cout << "total distance = " << total_measurement << endl;
		if( abs(differenceWE) < (0.05*total_measurement) )
		{
			cout << "Left and right are equal!" << endl;
			// Turn the car 90 degrees and perform this again.
			Nemo::setServo(0, 90);
			usleep(1000000);
			Nemo::setServo(0, 80);
	
			// Rotate the sonar to its left.
			Nemo::setServo(2, 180);  
			usleep(1000000);          // Wait for .5 seconds

			left_meas = Nemo::getSonar(3);
	
			// Rotate the sonar to its right.
			Nemo::setServo(2, 0);
			usleep(1000000);

			right_meas = Nemo::getSonar(3);

			// Set the sonar to face the front again.
			Nemo::setServo(2, 82);
			usleep(1000000);

			// Determine which side (L or R) of the wall it's farthest from.
			differenceNS = left_meas - right_meas;

			cout << "differenceNS = " << differenceNS << endl;
			cout << "Left meas = " << left_meas << "; right meas = " << right_meas << endl;
			total_meas = left_meas + right_meas;
			cout << "total distance = " << total_meas << endl;

			if( abs(differenceNS) < (0.05*total_meas) )
			{
				cout << "We're done!" << endl;
				keep_rotating = false;
			}

			else
			{
				if(left_meas > right_meas)
				{
					// Turn to the left - 180 on left wheel and 0 on right wheel.
					cout << "turning left NS" << endl;
					Nemo::setServo(0, 69);
					usleep(1000000);
					Nemo::setServo(0, 79);
				}
				else
				{
					// Turn to the right - 180 on right wheel and 0 on left wheel.
					cout << "turning right NS" << endl;
					Nemo::setServo(1, 90);
					usleep(1000000);
					Nemo::setServo(1, 80);
				}

			}

		}

		else
		{
			// Find the direction with the greatest gap and turn that way.
			if(left_measurement > right_measurement)
			{
				// Turn left - 180 on left wheel and 0 on right wheel.
				cout << "turning left WE" << endl;
				Nemo::setServo(0, 69);
				usleep(1000000);
				Nemo::setServo(0, 79);
			}
			else
			{
				// Turn right - 180 on right wheel and 0 on left wheel.
				cout << "turning right WE" << endl;
				Nemo::setServo(1, 90);
				usleep(1000000);
				Nemo::setServo(1, 80);
			}
		}

	}

	// Our lovely buzzer will go off.
	cout << "Bzzzzzzzzz!" << endl;
	Nemo::setSpeaker(500);
	usleep(3000000);


	std::cout << "Goodbye, World!" << std::endl;

	return 0;
}
