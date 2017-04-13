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


                    //#include "Navio/MS5611.h"
                    #include "Navio/MPU9250.h"
                    #include "Navio/LSM9DS1.h"
                    #include "Navio/Util.h"
                    #include "AHRS.hpp"
                    #include "Navio/PWM.h"
                    #include "Navio/RCInput.h"

                    #define G_SI 9.80665
                    #define PI   3.14159

                    #define PWM_OUTPUT_1 0 // 1
                    #define PWM_OUTPUT_2 1 // 2
                    #define PWM_OUTPUT_3 2 // 3

                    #define MS_ZERO_ANGLE_1 1.6
                    #define MS_ZERO_ANGLE_2 1.6



                    // Objects

                    InertialSensor *imu;
                    AHRS    ahrs;   // Mahony AHRS
                    PWM     pwm1;
                    PWM     pwm2;
                    PWM     pwm3;
                    RCInput rcin;
                    pthread_mutex_t m;
                    //MS5611 barometer;

                    // Sensor data

                    float ax, ay, az;
                    //4,0-1         Top
                    float gx, gy, gz;
                    float mx, my, mz;
                    float v_x=0,v_y=0;

                    float  p, p_ini[10],p1,p0=0,dh;                     // Variables
                    float t , t_ini[10],t0;
                    float g=9.80620,M=0.02896,R=8.314,a=6.5;


                    int count=0;
                    // Orientation data

                    float roll, pitch, yaw;

                    // inputs
                    int period0;
                    int period1;
                    int period2;

                    // process timing variables

                    clock_t t1_i;
                    clock_t t1_f;
                    float t1_v;
                    clock_t t2_i;
                    clock_t t2_f;
                    float t2_v;
                    clock_t t3_i;
                    clock_t t3_f;
                    //clock_t t1_i;
                    float t3_v;

                    // Timing data

                    float offset[3];
                    struct timeval tv;
                    float dt, maxdt;
                    float mindt = 0.01;
                    unsigned long previoustime, currenttime;
                    float dtsumm = 0;
                    int isFirst = 1;

                    // Network data
                    int sockfd;
                    struct sockaddr_in servaddr = {0};
                    char sendline[80];

                    InertialSensor* create_inertial_sensor(char *sensor_name)
                    {
                        InertialSensor *imu;

                        if (!strcmp(sensor_name, "mpu")) {
                            printf("Selected: MPU9250\n");
                            imu = new MPU9250();
                        }
                        else if (!strcmp(sensor_name, "lsm")) {
                            printf("Selected: LSM9DS1\n");
                            imu = new LSM9DS1();
                        }
                        else {
                            return NULL;
                        }

                        return imu;
                    }

                    void print_help()
                    //98,1          12%
                    {
                        printf("Possible parameters:\nSensor selection: -i [sensor name]\n");
                        printf("Sensors names: mpu is MPU9250, lsm is LSM9DS1\nFor help: -h\n");
                        printf("If you want to visualize IMU data on another machine,\n");
                        printf("add IP address and port number (by default 7000):\n");
                        printf("-i [sensor name] ipaddress portnumber\n");

                    }

                    //============================= Initial setup =================================

                    void imuSetup()
                    {

                        float t,p;
                        float g=9.8,p_0=1013.25;
                        //----------------------- MPU initialization ------------------------------

                        imu->initialize();

                        //-------------------------------------------------------------------------

                        printf("Beginning Gyro calibration...\n");
                        for(int i = 0; i<100; i++)
                        {
                            imu->update();
                            imu->read_gyroscope(&gx, &gy, &gz);

                            gx *= 180 / PI;
                            gy *= 180 / PI;
                            gz *= 180 / PI;

                            offset[0] += (-gx*0.0175);
                            offset[1] += (-gy*0.0175);
                            offset[2] += (-gz*0.0175);
                            usleep(10000);
                        }
                        offset[0]/=100.0;
                        offset[1]/=100.0;
                        offset[2]/=100.0;

                    //        printf("Offsets are: %f %f %f\n", offset[0], offset[1], offset[2]);
                        ahrs.setGyroOffset(offset[0], offset[1], offset[2]);
                    }

                    //============================== Main loop ====================================
                    //void motorSetup();
                    //void baro();
                     int servoSetup() {

                        if (check_apm()) {
                            return 1;
                        }
                        rcin.init();
                    //    barometer.initialize();
                        if (!pwm1.init(PWM_OUTPUT_1)) {
                            fprintf(stderr, "Output Enable not set. Are you root?\n");
                            return 0;
                        }

                        if (!pwm2.init(PWM_OUTPUT_2)) {

                        }
                        if (!pwm3.init(PWM_OUTPUT_3)) {
                            fprintf(stderr, "Output Enable not set. Are you root?\n");
                            return 0;
                        }

                        pwm3.enable(PWM_OUTPUT_3);
                        pwm3.set_period(PWM_OUTPUT_3, 200);

                        pwm1.enable(PWM_OUTPUT_1);
                        pwm1.set_period(PWM_OUTPUT_1, 200);

                        pwm2.enable(PWM_OUTPUT_2);
                        pwm2.set_period(PWM_OUTPUT_2, 200);
                    }
                    void* imuLoop(void*){
                    //147,1         24%

                    
                        while(1){

                        //----------------------- Calculate delta time ----------------------------
                            t1_i=clock();
                            gettimeofday(&tv,NULL);
                            previoustime = currenttime;
                            currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
                            dt = (currenttime - previoustime) / 1000000.0;
                            if(dt < 1/1300.0) usleep((1/1300.0-dt)*1000000);
                            gettimeofday(&tv,NULL);
                            currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
                            dt = (currenttime - previoustime) / 1000000.0;

                        //-------- Read raw measurements from the MPU and update AHRS --------------

                        // Accel + gyro.
                            imu->update();
                            imu->read_accelerometer(&ax, &ay, &az);
                            imu->read_gyroscope(&gx, &gy, &gz);

                            ax /= G_SI;
                            ay /= G_SI;
                            az /= G_SI;
                            gx *= 180 / PI;
                            gy *= 180 / PI;
                            gz *= 180 / PI;

                            ahrs.updateIMU(ax, ay, az, gx*0.0175, gy*0.0175, gz*0.0175, dt);

                        

                        //------------------------ Read Euler angles ------------------------------

                            ahrs.getEuler(&roll, &pitch, &yaw);

                    //------------------- Discard the time of the first cycle -----------------

                            if (!isFirst)
                            {
                                if (dt > maxdt) maxdt = dt;
                                if (dt < mindt) mindt = dt;
                            }
                            isFirst = 0;

                        //------------- Console and network output with a lowered rate ------------

                            dtsumm += dt;
                            if(dtsumm > 0.05)
                            {
                            // Console output
                    //        printf("ROLL: %+05.2f PITCH: %+05.2f YAW: %+05.2f PERIOD %.4fs RATE %dHz \n", roll, pitch, yaw * -1, dt, int(1/dt));

                            // Network output
                                sprintf(sendline,"%10f %10f %10f %10f %dHz\n", ahrs.getW(), ahrs.getX(), ahrs.getY(), ahrs.getZ(), int(1/dt));
                                sendto(sockfd, sendline, strlen(sendline), 0, (struct sockaddr *)&servaddr, sizeof(servaddr));

                                dtsumm = 0;
                            }
                        //cont
                        // Control the angle of the propeller.
                        float K1=0.4/30; // 0.4 ms per 30 degrees
                        float K2 =0.4/30;// 0.4 ms per 30 degrees
                        float K3 = 0.08/30;  // 0.4 ms per 18 degrees
                        //float K4 =  0.4/18; // 0.4 ms per 18 degrees

                    //2servo
                        int period1=rcin.read(1);
                        //194,0-1       48%
                        int period0=rcin.read(0);
                        float m1= 0.001;
                        float c1= -0.15;
                        float rolld=180-roll;

                        float ms_1 = K1*roll +1.47+ K3*gx;   // up
                        float ms_2 = K2*pitch +1.37 +K3*gy;  // up
                        
                        pwm1.set_duty_cycle(PWM_OUTPUT_2, ms_1);
                        pwm2.set_duty_cycle(PWM_OUTPUT_1, ms_2);

                    // manual control

                    //float ms_1=1.47+0.001*(period1-1500);
                    //float ms_2=1.37+0.001*(period0-1500);

                        //printf("roll:%f,pitch:%f\n",roll-180,pitch);
                        

                        float thrust;
                        //int  period = rcin.read(2);
                        float m= 0.001;
                        float c= -0.15;
                        thrust = m*period2 + c;
                        pwm1.set_duty_cycle(PWM_OUTPUT_3,thrust);
                        t1_f=clock();
                        t1_v=(float)(t1_f-t1_i)/CLOCKS_PER_SEC;
                        sleep(t1_v);

                    }
                    }
                    void* inputs(void*){
                        while(1){
                            t2_i=clock();
                            pthread_mutex_lock(&m);
                            period0=rcin.read(0);
                            period1=rcin.read(1);
                            period2=rcin.read(2);
                            //period3=rcin.read(3);
                            t2_f=clock();
                            t2_v=(float)(t2_f-t2_i)/CLOCKS_PER_SEC;
                            pthread_mutex_unlock(&m);
                            sleep(t2_v);
                        }
                        
                    }
                    void* outputs(void*){
                        while(1){
                        t3_i=clock();
                        printf("thread1 sleep time is\n",t1_v);
                        printf("thread2 sleep time is\n",t2_v);
                        t3_f=clock();
                        t3_v=(float)t3_f-t3_f/CLOCKS_PER_SEC;
                        printf("thread3 sleep time is\n",t3_v); 
                        sleep(t3_v);   
                        }
                        

                    }
                    /*
                    //1mot

                    void *motorSetup(void*){
                        float thrust;
                        while(1){
                            float thrust;
                            int  period = rcin.read(2);
                            float m= 0.001;
                            float c= -0.15;
                            thrust = m*period + c;
                            pwm1.set_duty_cycle(PWM_OUTPUT_3,thrust);
                        }
                    }
                    */
               
                    
                    int main(int argc, char *argv[])
                    {

                    //1main
                        pthread_t thread1;
                        pthread_t thread2;
                        pthread_t thread3;
                      pthread_mutex_init(&m,NULL);
                    //   pthread_t thread1;
                        int parameter;
                        char *sensor_name;

                        if (check_apm()) {
                            return 1;
                        }

                        if (argc < 2) {
                            printf("Enter parameter\n");
                            print_help();
                            
                            return EXIT_FAILURE;
                        }

                        // prevent the error message
                        opterr=0;

                        while ((parameter = getopt(argc, argv, "i:h")) != -1) {
                            switch (parameter) {
                                case 'i': sensor_name = optarg; break;
                                case 'h': print_help(); return EXIT_FAILURE;
                                case '?': printf("Wrong parameter.\n");
                                print_help();
                                return EXIT_FAILURE;
                            }
                        }

                        imu = create_inertial_sensor(sensor_name);

                        if (!imu) {
                            printf("Wrong sensor name. Select: mpu or lsm\n");
                            return EXIT_FAILURE;
                        }

                        if (!imu->probe()) {
                            printf("Sensor not enable\n");
                            return EXIT_FAILURE;
                        }
                    //--------------------------- Network setup -------------------------------

                        sockfd = socket(AF_INET,SOCK_DGRAM,0);
                        servaddr.sin_family = AF_INET;

                        if (argc == 5)  {
                            servaddr.sin_addr.s_addr = inet_addr(argv[3]);
                            servaddr.sin_port = htons(atoi(argv[4]));
                        } else {
                            servaddr.sin_addr.s_addr = inet_addr("127.0.0.1");
                            servaddr.sin_port = htons(7000);
                        }

                        //-------------------- IMU setup and main loop ----------------------------

                        imuSetup();

                        servoSetup();

                     //       pthread_create(&thread1,NULL,&baro,NULL);
                    //        pthread_detach(thread1);
                        
                        pthread_create(&thread1,NULL,input_i,NULL);
                        pthread_detach(thread1);
                        pthread_create(&thread2,NULL,imuLoop,NULL);
                        pthread_detach(thread2);
                        pthread_create(&thread3,NULL,outputs,NULL);
                        pthread_detach(thread3);
                        pthread_exit(&thread1);
                        pthread_exit(&thread2);
                        pthread_exit(&thread3);
                        //pthread_create(&thread,NULL,&motorSetup,NULL);
                        //pthread_detach(thread);
                        //pthread_create(&thread2,NULL,&imuLoop,NULL);
                        //pthread_detach(thread2);

                            //thread t2(baro);

                            //pthread_exit(&thread);
                            //pthread_exit(&thread1);
                        //pthread_exit(&thread);
                        //pthread_exit(&thread2);
                    }

                    //194,0-1       36%
