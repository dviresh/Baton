	// Standard C headers
	#include <math.h>
	#include <stdio.h>
	#include <sys/socket.h>
	#include <netinet/in.h>
	#include <arpa/inet.h>
	#include <stdint.h>
	#include <unistd.h>
	#include <sys/time.h>
	#include <iostream>
	#include <pthread.h>
	#include <ctime>


	// Project specific headers
	#include "Navio/MS5611.h"
	#include "Navio/MPU9250.h"
	#include "Navio/LSM9DS1.h"
	#include "Navio/Util.h"
	#include "AHRS.hpp"
	#include "Navio/PWM.h"
	#include "Navio/RCInput.h"
	#include "Eigen/Dense"

	using namespace std;
	using namespace Eigen;

	////////////////////////////////////////
	// Matrix Objects
	///////////////////////////////////////

	// Body Fixed Frame(BFF) Variables
	VectorXd position_bff(3);
	VectorXd velocity_bff(3);

	// Inertial Frame(IF) Variables
	VectorXd position_if(3);

	// Rotation Matrices -- BFF to IF
	MatrixXd rotx(3,3); // Roll --
	MatrixXd roty(3,3); // Pitch
	MatrixXd rotz(3,3); // Yaw
	MatrixXd rot_bi(3,3); // Complete rotation matrix from BFF to IF

	////////////////////////////////////
	// Constants
	////////////////////////////////////
	#define G_SI 9.80665
	#define PI   3.14159

	///////////////////////////////////
	// Constants for Controller
	/////////////////////////////////
	// Auto

	// Manual
	float MBeta= 0.0001/1177;
	float MAlpha= 0.000465/1171;
	float CBeta= 0.0007;
	float CAlpha= 0.0003;

	#define PWM_OUTPUT_1 0 // 1
	#define PWM_OUTPUT_2 1 // 2
	#define PWM_OUTPUT_3 2 // 3

	#define MS_ZERO_ANGLE_1 1.37
	#define MS_ZERO_ANGLE_2 1.47

	#define PERIOD_CONTROL_LOOP 2500   // microseconds = 1 ms
	#define PERIOD_INPUT_LOOP   10000  // microseconds = 10 ms
	#define PERIOD_OUTPUT_LOOP  100000 // microseconds = 100 ms


	////////////////////////////////////
	// Sensor, input, and actuator objects
	////////////////////////////////////
	InertialSensor * imu;
	// NOTE: The IMU on this system only has angular rate, linear acceleration, and magnetometer


	AHRS        ahrs;
	//Mahony AHRS
	// NOTE: This doesn't use the magnetometer to get rid of bias drift. Plus, the magnetometer probably wouldn't work well inside our building.


	PWM         pwm1;
	PWM         pwm2;
	PWM         pwm3;
	RCInput     rcin;

	pthread_mutex_t rcInputMutex;
	pthread_mutex_t controllerStateMutex;
	pthread_mutex_t outputMutex;
	//MS5611 barometer;


	////////////////////////////////////
	//Sensor reading data
	////////////////////////////////////
	float       ax   , ay, az;
	float       gx   , gy, gz;
	float       mx   , my, mz;
	float       v_x = 0, v_y = 0;

	float       p    , p_ini[10], p1, p0 = 0, dh;

	//Variables
	float       t    , t_ini[10], t0;
	float       g = 9.80620, M = 0.02896, R = 8.314, a = 6.5;

	int                 count = 0;

	////////////////////////////////////////////////////////////////////////
	//Orientation data
	////////////////////////////////////////////////////////////////////////
	float       roll, pitch, yaw;
	//float     rotx[3][3],roty[3][3],rotz[3][3];

	////////////////////////////////////////////////////////////////////////
	// RC controller inputs (mutex protected)
	/////////////////////////////////////////////////////////////////////////
	int                 g_AlphaControlRad;
	int                 g_BetaControlRad;
	int                 g_ThrustControlRadPerSec;

	////////////////////////////////////////////////////////////////////////
	// Controller state variables (mutex protected)
	////////////////////////////////////////////////////////////////////////
	float g_state[12];   // this contains [x, y, z, phi, theta, psi, dx, dy, dz, dphi, dtheta, dpsi]
	float g_inputs[3];  // this contains [rotor_velocity, alpha, beta]

	//////////////////////////////////////////////////////////////////////
	// Elased Time Outputs (mutex protected)
	/////////////////////////////////////////////////////////////////////
	int elapsedOutput_t1;
	int elapsedOutput_t2;
	int elapsedOutput_t3;

	//process timing variables
	clock_t t1_i;
	clock_t     t1_f;
	float       t1_v;
	clock_t     t2_i;
	clock_t     t2_f;
	float       t2_v;
	clock_t     t3_i;
	clock_t     t3_f;
	float       t3_v;

	//Timing data

	float       offset[3];
	struct timeval  tv;
	float       dt   , maxdt;
	float       mindt = 0.01;
	unsigned long   previoustime, currenttime;
	float       dtsumm = 0;
	int                 isFirst = 1;

	//Network data
	int                 sockfd;
	struct sockaddr_in servaddr = {0};
	char        sendline[80];

	InertialSensor *create_inertial_sensor(char *sensor_name)
	{
	  InertialSensor *imu;

	  if (!strcmp(sensor_name, "mpu"))
	  {
	    printf("Selected: MPU9250\n");
	    imu = new MPU9250();
	  }
	  else if (!strcmp(sensor_name, "lsm"))
	  {
	    printf("Selected: LSM9DS1\n");
	    imu = new LSM9DS1();
	  }
	  else
	  {
	    return NULL;
	  }

	  return imu;
	}

	void        print_help()
	{
	  printf("Possible parameters:\nSensor selection: -i [sensor name]\n");
	  printf("Sensors names: mpu is MPU9250, lsm is LSM9DS1\nFor help: -h\n");
	  printf("If you want to visualize IMU data on another machine,\n");
	  printf("add IP address and port number (by default 7000):\n");
	  printf("-i [sensor name] ipaddress portnumber\n");

	}

	//=============================Initial setup == == == == == == == == == == == == == == == == =

	void        imuSetup()
	{

	  float       t  , p;
	  float       g = 9.8, p_0 = 1013.25;
	  //-----------------------MPU initialization-- -- --------------------------

	  imu->initialize();

	  //-------------------------------------------------------------------------



	}

	// Set up the servo motor objects for controlling the rotor velocity and the two linear actuators
	int servoSetup()
	{

	  if (check_apm())
	  {
	    return 1;
	  }

	  // Initialize the RC controller input object
	  rcin.init();


	  //barometer.initialize();

	  // Initialize the three PWM outputs

	  if (!pwm1.init(PWM_OUTPUT_1))                                           // alpha linear actuator
	  {
	    fprintf(stderr, "Output Enable not set for PWM1. Are you root?\n");
	    return 0;
	  }
	  if (!pwm2.init(PWM_OUTPUT_2))                                           // beta linear actuator
	  {
	    fprintf(stderr, "Output Enable not set for PWM2. Are you root?\n");
	    return 0;
	  }
	  if (!pwm3.init(PWM_OUTPUT_3))                                          // rotor velocity
	  {
	    fprintf(stderr, "Output Enable not set for PWM3. Are you root?\n");
	    return 0;
	   }

	  // Enable each of the PWM and set the initial period
	  pwm1.enable(PWM_OUTPUT_1);
	  pwm1.set_period(PWM_OUTPUT_1, 200);

	  pwm2.enable(PWM_OUTPUT_2);
	  pwm2.set_period(PWM_OUTPUT_2, 200);

	  pwm3.enable(PWM_OUTPUT_3);
	  pwm3.set_period(PWM_OUTPUT_3, 200);
	}


	//*****************************************************************************************************************************************************************************************
	//************************************************************************************************ -- CONTROL THREAD -- ********************************************************************
	//******************************************************************************************************************************************************************************************
	 void* controlThread(void *)
	{
	  struct timeval  tv;
	  unsigned long   startTime, endTime, elapsed;

	  float dt = (float)PERIOD_CONTROL_LOOP/1000000.0;

	  while (1)
	  {
	    // Get the time at the start of the loop
	    gettimeofday(&tv, NULL);
	    startTime = 1000000 * tv.tv_sec + tv.tv_usec;

	    // Read in the most recent RC controller inputs in a threadsafe manner
	    // FIXME: change l_period0 to something like l_periodRotorSpeed or l_period1 to l_periodTip
	    int l_periodAlphaAngle ;
	    int l_periodBetaAngle ;
	    int l_periodRotorSpeed ;

	    pthread_mutex_lock(&rcInputMutex);
	    l_periodAlphaAngle = g_AlphaControlRad;
	    l_periodBetaAngle = g_BetaControlRad;
	    l_periodRotorSpeed = g_ThrustControlRadPerSec;
	    pthread_mutex_unlock(&rcInputMutex);


	    //--------Read raw measurements from the MPU and update AHRS-- -- ----------
	    //Accel + gyro.
	    imu->update();
	    imu->read_accelerometer(&ax, &ay, &az);
	    imu->read_gyroscope(&gx, &gy, &gz);
	 ax /= G_SI;
	    ay /= G_SI;
	    az /= G_SI;
	    gx *= 180 / PI;
	    gy *= 180 / PI;
	    gz *= 180 / PI;

	    ahrs.updateIMU(ax, ay, az, gx * 0.0175, gy * 0.0175, gz * 0.0175, dt);

	    //------------------------Read Euler angles-- -- --------------------------
	    ahrs.getEuler(&roll, &pitch, &yaw);

	    //-------------------Discard the time of the first cycle-- -- -------------
	    if (!isFirst)
	    {
	      if (dt > maxdt)
	        maxdt = dt;
	      if (dt < mindt)
	        mindt = dt;
	    }
	    isFirst = 0;

	    //-------------Console and network output with a lowered rate-- -- --------

	    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	    // WARNING: THIS SECTION OF CODE BELOW NEEDS A TON OF WORK. I'M NOT EVEN SURE I FOLLOW. IT SHOULD BE CLEARLY DIVIDED INTO
	    //          SEVERAL SECTIONS. NOTE: (1) and (2) should never be running simultaneously
	    //               (1) A section that computes the desired thrust and rotor angles using Konstantin's controller
	    //               (2) A section that computes the desired thrust and rotor angles from the RC controller
	    //               (3) A section that converts the desired thrust and rotor angles into the corresponding servo period.
	    //                      This last part should definitely be done in another function for ease of code reading.x
	    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	    //cont
	    // Control the angle of the propeller.
	    float               K1 = 0.4 / 30;
	    //0.4 ms per 30 degrees
	    float               K2 = 0.4 / 30;
	    //0.4 ms per 30 degrees
	    float               K3 = 0.08 / 30;
	    //0.4 ms per 18 degrees
	    // float    K4 = 0.4 / 18;
	    //0.4 ms per 18 degrees


	    //2servo

	    float               m1 = 0.001;
	     float               c1 = -0.15;
	    float               rolld = 180 - roll;

	    //**************************************************************************************** -- Kontantin's Controller -- ************************************

	     //float            ms_2 = K1 * roll + MS_ZERO_ANGLE_2 + K3 * gx;
	    //up
	    // float            ms_1 = K2 * pitch + MS_ZERO_ANGLE_1 + K3 * gy;
	    //up

	    //**************************************************************************************** -- Controlling with RC -- ****************************************

	    // Computing Gains
	    float KBeta= MBeta*l_periodBetaAngle+CBeta;
	    float KAlpha= MAlpha*l_periodAlphaAngle+CAlpha;

	    // Control equations
	    float       ms_1 = MS_ZERO_ANGLE_2 + KBeta * (l_periodBetaAngle - 1501);
	    float       ms_2 = MS_ZERO_ANGLE_1 + KAlpha * (l_periodAlphaAngle - 1512);


	//************************************************************************************************** -- Thrust Computation -- **********************************

	    float   m = 0.001;
	    float   c = -0.15;
	    float   thrust = m * l_periodRotorSpeed + c;

	//******************************************************************************************* -- Output To Pins -- ********************************************
	    pwm1.set_duty_cycle(PWM_OUTPUT_2, ms_1);
	    pwm2.set_duty_cycle(PWM_OUTPUT_1, ms_2);
	    pwm3.set_duty_cycle(PWM_OUTPUT_3,thrust);


	    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	    // WARNING: END WARNING SECTION
	    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	    pthread_mutex_lock(&controllerStateMutex);
	    // FIXME: The g_state should be populated with as many of the state variables as you have access to.
	    //        This won't be all of them. Here are the ones I think you should have:
	    //            A) phi, theta, psi from AHRS. Compute the equivalent rotation matrix
	    //            B) linear accelerations in body-fixed frame (we could integrate/double-integrate these to get linear velocities/position with noise drift)
	    //                NOTE: These body-fixed frame velocities will need to be converted to inertial frame velocities using the rotation matrix from (A)
	    //                      Then you can integrate the inertial frame velocities to have a (bad) estimate of the inertial frame positions.
	    //            C) dphi, dtheta, dpsi - angular velocities from rate gyro
	    //        We will never have a good estimate of the [x,y,z] position without either running outside with a GPS or having a camera system indoors.
	    //        I think this will make it hard to get a stabilizing controller with our limited lateral workspace in the tent.
	    //        One solution is to create a stabilizing controller based on the noisy integral and double integral of the linear accelerations. Because we have
	    //        good rotational estimates, this should at least give us a rotationally stable system and we can use the control inputs from the controller to give
	    //        [x,y,z] setpoints that are relative to the current (bad) position.

	//******************************* -- Body Fixed Frame Parameters -- ************************************************************************************************************************

	// Velocities
	    velocity_bff(0)=ax*dt; // Along X axis
	    velocity_bff(1)=ay*dt; // Along y axis
	    velocity_bff(2)=az*dt; // Along Z axis

	    position_bff(0)=velocity_bff(0)*dt; // X coordinate
	    position_bff(1)=velocity_bff(1)*dt; // Y coordinate
	    position_bff(2)=velocity_bff(2)*dt; // Z coordinate

	//*********************************************************************** -- Rotational Matrices -- ****************************************************************************************

	    // Roll

	    rotx(0,0)=1.0;
	    rotx(0,1)=0.0;
	    rotx(0,2)=0.0;
	    rotx(1,0)=0.0;
	    rotx(1,1)=cos(-roll);
	    rotx(1,2)=sin(-roll);
	    rotx(2,0)=0.0;
	    rotx(2,1)=-sin(-roll);
	    rotx(2,2)=cos(-roll);

	    // Pitch

	    roty(0,0)=cos(-pitch);
	    roty(0,1)=0.0;
	    roty(0,2)=-sin(-pitch);
	    roty(1,0)=0.0;
	    roty(1,1)=1.0;
	    roty(1,2)=0.0;
	    roty(2,0)=sin(-pitch);
	    roty(2,1)=0.0;
	    roty(2,2)=cos(-pitch);

	    // Yaw

	    rotz(0,0)=cos(-yaw);
	    rotz(0,1)=sin(-yaw);
	    rotz(0,2)=0.0;
	    rotz(1,0)=-sin(-yaw);
	    rotz(1,1)=cos(-yaw);
	    rotz(1,2)=0.0;
	    rotz(2,0)=0.0;
	    rotz(2,1)=0.0;
	    rotz(2,2)=1.0;

	//************************************************** -- Complete Rotation Matrix From BFF to IF -- *****************************************************************************************

	    rot_bi=rotz*roty*rotx;

	//************************************************** -- Position Of Baton In If -- *********************************************************************************************************

	    position_if=rot_bi*position_bff;

	//*************************************************** -- Storing States of Baton -- *********************************************************************************************************
	    g_state[0]=position_if(0);
	    g_state[1]=position_if(1);
	    g_state[2]=position_if(2);
	    g_state[3]=roll;
	    g_state[4]=pitch;
	    g_state[5]= yaw;
	    g_state[6]=velocity_bff(0);
	    g_state[7]=velocity_bff(1);
	    g_state[8]=velocity_bff(2);
	    g_state[9]=gx;
	    g_state[10]=gy;
	    g_state[11]=gz;
	//*************************************************** -- Storing the inputs from RC Trasmitter -- ******************************************************************************************
	    g_inputs[0] = l_periodAlphaAngle;
	    g_inputs[1]= l_periodBetaAngle;
	    g_inputs[2]= l_periodRotorSpeed;
	    pthread_mutex_unlock(&controllerStateMutex);

	    // Get the time after the execution of the loop and sleep the appropriate number of microseconds
	    gettimeofday(&tv, NULL);
	    endTime = 1000000 * tv.tv_sec + tv.tv_usec;

	    elapsed = endTime-startTime;


	    if ( elapsed < PERIOD_CONTROL_LOOP )
	    {
	      elapsedOutput_t1=PERIOD_CONTROL_LOOP-elapsed;
	      usleep(elapsedOutput_t1);
	    }


	  }
	}

	//******************************************************************************************************************************************************************************************
	//*********************************************************************************************************** -- INPUTS THREAD -- **********************************************************
	//******************************************************************************************************************************************************************************************
	void* inputThread(void *)
	{
	  struct timeval  tv;
	  unsigned long   startTime, endTime, elapsed;
	  while (1)
	  {
	//**************************************************************************** Get the time at the start of the loop
	    gettimeofday(&tv, NULL);
	 startTime = 1000000 * tv.tv_sec + tv.tv_usec;


	    int l_periodAlphaAngle = rcin.read(0);
	    int l_periodBetaAngle = rcin.read(1);
	    int l_periodRotorSpeed = rcin.read(2);

	    pthread_mutex_lock(&rcInputMutex);
	    g_AlphaControlRad = l_periodAlphaAngle;
	    g_BetaControlRad = l_periodBetaAngle;
	    g_ThrustControlRadPerSec = l_periodRotorSpeed;
	    pthread_mutex_unlock(&rcInputMutex);

	//************************************************************************* Get the time after the execution of the loop and sleep the appropriate number of microseconds
	    gettimeofday(&tv, NULL);
	    endTime = 1000000 * tv.tv_sec + tv.tv_usec;

	    elapsed = endTime-startTime;

	    if ( elapsed < PERIOD_INPUT_LOOP )
	    {
	      //        elapsedOutput_t2=elapsed;
	     elapsedOutput_t2=PERIOD_INPUT_LOOP-elapsed;
	      usleep(elapsedOutput_t2);
	    }

	  }

	}

	//******************************************************************************************************************************************************************************************
	//****************************************************************************************** -- OUTPUT THREAD -- ***************************************************************************
	//******************************************************************************************************************************************************************************************
	void* outputThread(void *)
	{
	  struct timeval  tv;
	  unsigned long   startTime, endTime, elapsed;
	  while (1)
	  {
	   // Get the time at the start of the loop
	 gettimeofday(&tv, NULL);
	    startTime = 1000000 * tv.tv_sec + tv.tv_usec;
	    int l_periodAlphaAngle;
	    int l_periodBetaAngle;
	    int l_periodRotorSpeed;
	    float l_state[12];
	    float l_inputs[3];
	    int elapsedt1;
	    int elapsedt2;
	    int elapsedt3;

	    // Copy the RC controller inputs into a local variable (mutex protected)
	    pthread_mutex_lock(&rcInputMutex);
	    l_periodAlphaAngle = g_AlphaControlRad;
	    l_periodBetaAngle = g_BetaControlRad;
	    l_periodRotorSpeed = g_ThrustControlRadPerSec;
	    pthread_mutex_unlock(&rcInputMutex);


	    // Copy the controller states into a local variable (mutex protected)
	    pthread_mutex_lock(&controllerStateMutex);
	    for (int i = 0 ; i < 12 ; i++)
	      l_state[i] = g_state[i];

	    for (int i = 0 ; i < 3 ; i++)
	      l_inputs[i] = g_inputs[i];
	    pthread_mutex_unlock(&controllerStateMutex);

	//************************************************************************* -- Output Inputs from RC - Controller --*****************************************************************

	   // printf("RC thrust: %d\tRC alpha: %d\tRC beta: %d\n", l_period0, l_period1, l_period2);

	//*********************************************************************** -- Outputting the Baton State Variables -- ***********************************************************************
	    printf("System States\n\t");
	    for (int i = 0 ; i < 12 ; i++)
	    {
	      printf("%f, ", l_state[i]);
	    }
	    printf("\n\n");


	    // Get the time after the execution of the loop and sleep the appropriate number of microseconds
	    gettimeofday(&tv, NULL);
	    endTime = 1000000 * tv.tv_sec + tv.tv_usec;

	    elapsed = endTime-startTime;
	 ;
	     elapsedOutput_t3=PERIOD_OUTPUT_LOOP-elapsed;

	      pthread_mutex_lock(&outputMutex);
	      elapsedt1=elapsedOutput_t1;
	      elapsedt2=elapsedOutput_t2;
	      elapsedt3=elapsedOutput_t3;
	      pthread_mutex_unlock(&outputMutex);

	//********************************************************************* -- Output Sleep Times -- ******************************************************************************************

	       printf("Sleep Time:\t Control Thread:%d\t, Input Thread:%d\t, Output Thread:%d\t\n",elapsedt1,elapsedt2,elapsedt3 );


	    if ( elapsed < PERIOD_OUTPUT_LOOP )
	    {
	       usleep(elapsedOutput_t3);
	    }

	    }
	}



	//******************************************************************************************************************************************************************************************
	//******************************************************************************************************* -- MAIN -- ***********************************************************************
	//******************************************************************************************************************************************************************************************
	int                 main   (int argc, char *argv[])
	{

	  //1 main
	  pthread_t    thread1;
	  pthread_t    thread2;
	  pthread_t    thread3;

	//***************************************************************************** -- Initializing the nutex
	  pthread_mutex_init(&rcInputMutex, NULL);
	  pthread_mutex_init(&controllerStateMutex, NULL);
	  pthread_mutex_init(&outputMutex, NULL);



	  //pthread_t thread1;
	  int         parameter;
	  char        *sensor_name;

	  if (check_apm())
	  {
	    return 1;
	  }
	  if (argc < 2)
	  {
	    printf("Enter parameter\n");
	    print_help();

	    return EXIT_FAILURE;
	  }
	  //prevent the error message
	  opterr = 0;

	  while ((parameter = getopt(argc, argv, "i:h")) != -1)
	  {
	    switch (parameter)
	    {
	    case 'i':
	      sensor_name = optarg;
	      break;
	  case 'h':
	      print_help();
	      return EXIT_FAILURE;
	    case '?':
	      printf("Wrong parameter.\n");
	      print_help();
	      return EXIT_FAILURE;
	    }
	  }

	  imu = create_inertial_sensor(sensor_name);

	  if (!imu)
	  {
	    printf("Wrong sensor name. Select: mpu or lsm\n");
	    return EXIT_FAILURE;
	  }
	  if (!imu->probe())
	  {
	    printf("Sensor not enable\n");
	    return EXIT_FAILURE;
	  }
	  //---------------------------Network setup-- -- ---------------------------

	  sockfd = socket(AF_INET, SOCK_DGRAM, 0);
	  servaddr.sin_family = AF_INET;

	  if (argc == 5)
	  {
	    servaddr.sin_addr.s_addr = inet_addr(argv[3]);
	    servaddr.sin_port = htons(atoi(argv[4]));
	  } else
	  {
	    servaddr.sin_addr.s_addr = inet_addr("127.0.0.1");
	    servaddr.sin_port = htons(7000);
	  }

	  //--------------------IMU setup and main loop-- -- ------------------------
	  imuSetup();
	  servoSetup();

	 //********************************************************************************************* -- Calling the threads -- *****************************************************************
	  pthread_create(&thread1, NULL, controlThread, NULL);
	  pthread_detach(thread1);
	  pthread_create(&thread2, NULL, inputThread, NULL);
	  pthread_detach(thread2);
	  pthread_create(&thread3, NULL, outputThread, NULL);
	  pthread_detach(thread3);
	  pthread_exit(&thread1);
	  pthread_exit(&thread2);
	  pthread_exit(&thread3);

	}

	            
	     
	                        
	                        
