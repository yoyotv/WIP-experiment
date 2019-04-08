#include  <wiringPiI2C.h>
#include  <stdio.h>
#include  <stdlib.h>
#include  <math.h>
#include  <sys/time.h>


// PID parameters
double Kp = 2.5;   // 2.5
double Ki = 0.8;   // 1.0
double Kd = 8.0;   // 8.0
double K  = 1.9*1.12;


// Complimentary Filter parameters
double K0 = (double) 0.98;
double K1 = (double) 0.02;

int fd;
int acclX, acclY, acclZ;
int gyroX, gyroY, gyroZ;
double accl_scaled_x, accl_scaled_y, accl_scaled_z;
double gyro_scaled_x, gyro_scaled_y, gyro_scaled_z;


double gyro_offset_x, gyro_offset_y;
double gyro_total_x, gyro_total_y;
double gyro_x_delta, gyro_y_delta;
double rotation_x, rotation_y;
double last_x, last_y;

struct timeval tv, tv2;
unsigned long long      timer, t;

double deltaT;

int read_word_2c(int addr)
{
  int val;
  val = wiringPiI2CReadReg8(fd, addr);
  val = val << 8;
  val += wiringPiI2CReadReg8(fd, addr+1);
  if (val >= 0x8000)
    val = -(65536 - val);

  return val;
}

double dist(double a, double b)
{
  return sqrt((a*a) + (b*b));
}

double get_y_rotation(double x, double y, double z)
{
  double radians;
  radians = atan2(x, dist(y, z));
  return -(radians * (180.0 / M_PI));
}

double get_x_rotation(double x, double y, double z)
{
  double radians;
  radians = atan2(y, dist(x, z));
  return (radians * (180.0 / M_PI));
}

void read_all()
{
    acclX = read_word_2c(0x3B);
    acclY = read_word_2c(0x3D);
    acclZ = read_word_2c(0x3F);

    accl_scaled_x = acclX / 16384.0;
    accl_scaled_y = acclY / 16384.0;
    accl_scaled_z = acclZ / 16384.0;

    gyroX = read_word_2c(0x43);
    gyroY = read_word_2c(0x45);
    gyroZ = read_word_2c(0x47);

    gyro_scaled_x = gyroX / 131.0;
    gyro_scaled_y = gyroY / 131.0;
    gyro_scaled_z = gyroZ / 131.0;
}

unsigned long long  getTimestamp()
{
  gettimeofday(&tv, NULL);
  return (unsigned long long) tv.tv_sec * 1000000 + tv.tv_usec;
}


double constrain(double v, double min_v, double max_v)
{
  if (v <= min_v)
    return (double)min_v;
  else if (v >= max_v)
    return (double)max_v;
  else
    return (double)v;
}

double GUARD_GAIN = 100.0;
double error, last_error, integrated_error;
double pTerm, iTerm, dTerm;
double angle;
double angle_offset = 2.0;  //1.5

double speed;

//Brain parameter set===============



//double mean[10] ={-1.6,-1,-0.6,-0.2,0,0.2,0.6,1,1.5,2};
double var[10] = {3,3,3,3,3,3,3,3,3,3};
double s[10] = {0,0,0,0,0,0,0,0,0};
//double mean2[10] = { -1.6,-1,-0.6,-0.2,0,0.2,0.6,1,1.5,2 };
double mean[10] = { -2,-1,-0.6,-0.4,-0.1,0.1,0.4,0.6,1,2 };
double mean2[10] = { -2,-1,-0.6,-0.4,-0.1,0.1,0.4,0.6,1,2 };
double var2[10] = { 3,3,3,3,3,3,3,3,3,3 };
double s2[10] = { 0,0,0,0,0,0,0,0,0 };

double weight_a[10] = { 0,0,0,0,0,0,0,0,0 };
double weight_p[10] = { 0,0,0,0,0,0,0,0,0 };

double delta_weight_a[10];
double delta_weight_p[10];
double delta_mean[10];
double delta_var[10];
double delta_mean2[10];
double delta_var2[10];
double identicalmatrix[10]= { 1,1,1,1,1,1,1,1,1,1 };
double identicalmatrix2[10] = { 1,1,1,1,1,1,1,1,1,1 };
double a,b,ub;
int value = 10;
double tu_v;
double d_error;
double sliding,sliding2;
int i;
//Brain parameter set===============







double rate,rob=0;




void pid()
{
	error = last_y - angle_offset;
	/*
	pTerm = Kp * error;

	integrated_error = 0.95*integrated_error + error;
	iTerm = Ki * integrated_error;

	dTerm = Kd * (error - last_error);
	last_error = error;
	*/
	d_error = (error - last_error)/ deltaT;
	last_error = error;

	sliding = 2*error + d_error;
	sliding2 =d_error;

	//brain=======================================
	if (abs(sliding) <= 1) {
		rate = 0.1;
	}
	else{
		rate = 0.1;
	}
	for (i = 0; i < value; i++) {
		
		delta_weight_a[i] = -rate*sliding*s[i]; // (error converge much quickly as learning rate minuslearning)
		delta_weight_p[i] = 0.01*sliding*s[i];

		delta_mean[i] = -(0.002*sliding)*(weight_a[i] - weight_p[i]) * s[i] * (2 * (identicalmatrix[i] - mean[i])) / pow(var[i], 2);
		delta_var[i] = -(0.002*sliding)*(weight_a[i] - weight_p[i]) * s[i] * (2 * pow((identicalmatrix[i] - mean[i]), 2)) / pow(var[i], 3);

		delta_mean2[i] = -(0.002*sliding2)*(weight_a[i] - weight_p[i]) * s2[i] * (2 * (identicalmatrix2[i] - mean2[i])) / pow(var2[i], 2);
		delta_var2[i] = -(0.002*sliding2)*(weight_a[i] - weight_p[i]) * s2[i] * (2 * pow((identicalmatrix2[i] - mean2[i]), 2)) / pow(var2[i], 3);

		//update weight mean variance == == == == == == == == == ==


		weight_a[i] = weight_a[i] + delta_weight_a[i];
		//weight_p[i] = weight_p[i] + delta_weight_p[i];

		mean[i] = mean[i] + delta_mean[i];
		var[i] = var[i] + delta_var[i];

		mean2[i] = mean2[i] + delta_mean2[i];
		var2[i] = var2[i] + delta_var2[i];

	}

  //= == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == ==
	  //controller
	  //CMAC****************************************************************************************
	for (i = 0; i < value; i++) {
		identicalmatrix[i] = 1;
	}

	for (i = 0; i < value; i++) {
		identicalmatrix[i] =identicalmatrix[i]*sliding;
	}
	for (i = 0; i < value; i++) {
		identicalmatrix2[i] = 1;
	}

	for (i = 0; i < value; i++) {
		identicalmatrix2[i] = identicalmatrix2[i] * sliding2;
	}



  //sensory_cortex layer == == == == == =

	  for (i = 0; i < value; i++) {
		  if (sliding> mean[9]) {
			  sliding = mean[9];
		  }
		  if (sliding < mean[0]) {
			  sliding = mean[0];
		  }
			  s[i] = exp(-(pow((sliding- mean[i]),2)/(pow(var[i],2))));

		  }
	  for (i = 0; i < value; i++) {
		  if (sliding2> mean2[9]) {
			  sliding2 = mean2[9];
		  }
		  if (sliding2 < mean2[0]) {
			  sliding2 = mean2[0];
		  }

		  s2[i] = exp(-(pow((sliding2 - mean2[i]), 2) / (pow(var2[i], 2))));
	  }
		  

	  /*
	  for (i = 0; i < value; i++) {
		  s[i] =s[i]*s2[i] ;
	  }
	  */

	  //= == == == == == == =
	  //Receptive field
	  a = 0;
	  b = 0;
  for (i = 0; i < value; i++) {
	  a = a + s[i] * weight_a[i];
	  b = b + s[i] * weight_p[i];
  }

  ub = a - b*0;
	  

	  //= == == == == == == == == == == == == == == == == == == == == == == ==
	  //organize input of WIP system




  //brain=======================================
  //speed = constrain(tu_v, -GUARD_GAIN, GUARD_GAIN); 
  if (sliding > 5) {
	  rob = 5 * 8;
  }
  if (sliding < -5) {
	  rob = -5 * 8;
  }
  if (abs(sliding) <= 5) {
	  rob = sliding * 8;
  }
  
	  if (ub >= 100) {
		  ub = 100;
	  };
  if (ub <= -100) {
	  ub = -100;
  };
  if (abs(ub) <= 100) {
	  ub = ub;
  };
  tu_v = -ub;
  speed = tu_v + 0*rob;

}























int main()
{
  init_motors();
  delay(200);

  fd = wiringPiI2CSetup (0x68);
  wiringPiI2CWriteReg8 (fd,0x6B,0x00);//disable sleep mode 
//  printf("set 0x6B=%X\n",wiringPiI2CReadReg8 (fd,0x6B));

  timer = getTimestamp();

  deltaT = (double) (getTimestamp() - timer)/1000000.0;
  read_all();

  last_x = get_x_rotation(accl_scaled_x, accl_scaled_y, accl_scaled_z);
  last_y = get_y_rotation(accl_scaled_x, accl_scaled_y, accl_scaled_z);


  gyro_offset_x = gyro_scaled_x;
  gyro_offset_y = gyro_scaled_y;

  gyro_total_x = last_x - gyro_offset_x;
  gyro_total_y = last_y - gyro_offset_y;


  while(1) {
    t = getTimestamp();
    deltaT = (double) (t - timer)/1000000.0;
    timer = t;

    read_all();

    gyro_scaled_x -= gyro_offset_x;
    gyro_scaled_y -= gyro_offset_y;

    gyro_x_delta = (gyro_scaled_x * deltaT);
    gyro_y_delta = (gyro_scaled_y * deltaT);

    gyro_total_x += gyro_x_delta;
    gyro_total_y += gyro_y_delta;

    rotation_x = get_x_rotation(accl_scaled_x, accl_scaled_y, accl_scaled_z);
    rotation_y = get_y_rotation(accl_scaled_x, accl_scaled_y, accl_scaled_z);

//    printf("[BEFORE] gyro_scaled_y=%f, deltaT=%lf, rotation_y=%f, last_y= %f\n", (double)gyro_scaled_y, (double)deltaT, (double)rotation_y, (double) last_y);

//    printf("[1st part] = %f\n", (double) K0*(last_y + gyro_y_delta));
//    printf("[2nd part] = %f\n", (double) K1*rotation_y);
    last_x = K0 * (last_x + gyro_x_delta) + (K1 * rotation_x);
    last_y = K0 * (last_y + gyro_y_delta) + (K1 * rotation_y);

//    printf("[AFTER] gyro_scaled_y=%f, deltaT=%lf, rotation_y=%f, last_y=%f\n", (double)gyro_scaled_y, (double)deltaT, (double)rotation_y, (double) last_y);
    
    //if (last_y < -60.0 || last_y > 60.0) 
     // stop_motors();

    pid();
    printf("%lf\t%lf\t%lf\t%lf\t%lf\n", error, speed, pTerm, iTerm, dTerm);
	printf("%1f\t%1f\t%1f\n", d_error, sliding, tu_v);
	for (i = 0; i < value; i++) {
		printf("%1f  %1f\n", s[i], s2[i]);
	}

    motors(speed, 0.0, 0.0);
    
    delay(10);
  }

  stop_motors();
  return 0;
}
