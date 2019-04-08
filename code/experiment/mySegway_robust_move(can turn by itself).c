
#include  <wiringPiI2C.h>
#include  <stdio.h>
#include  <stdlib.h>
#include  <math.h>
#include  <sys/time.h>





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
double error_theta, last_error, integrated_error;
double pTerm, iTerm, dTerm;
double angle;
double angle_offset = 2.0;  //1.5

double speed_l,speed_r;

//Brain parameter set===============


//w part=======================
double mean[10] ={-1.6,-1,-0.6,-0.2,0,0.2,0.6,1,1.5,2};
double var[10] = {5,5,5,5,5,5,5,5,5,5};
double s[10] = {0,0,0,0,0,0,0,0,0};
double mean2[10] = { -1.6,-1,-0.6,-0.2,0,0.2,0.6,1,1.5,2 };
double var2[10] = { 5,5,5,5,5,5,5,5,5,5 };
double s2[10] = { 0,0,0,0,0,0,0,0,0 };
double recptive[10] = { 0,0,0,0,0,0,0,0,0 };


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
double a, b, uc;
int value = 10;
double error_w, d_error_w, last_error_w=0;
double delta_w;
double d_w = 0, p_w = 0;
double receptive[10] = { 0,0,0,0,0,0,0,0,0,0 };

//v part=======================
double vmean[10] = { -1.6,-1,-0.6,-0.2,0,0.2,0.6,1,1.5,2 };
double vvar[10] = { 5,5,5,5,5,5,5,5,5,5 };
double vs[10] = { 0,0,0,0,0,0,0,0,0 };
double vmean2[10] = { -1.6,-1,-0.6,-0.2,0,0.2,0.6,1,1.5,2 };
double vvar2[10] = { 5,5,5,5,5,5,5,5,5,5 };
double vs2[10] = { 0,0,0,0,0,0,0,0,0 };
double vrecptive[10] = { 0,0,0,0,0,0,0,0,0 };


double vweight_a[10] = { 0,0,0,0,0,0,0,0,0 };
double vweight_p[10] = { 0,0,0,0,0,0,0,0,0 };

double vdelta_weight_a[10];
double vdelta_weight_p[10];
double vdelta_mean[10];
double vdelta_var[10];
double vdelta_mean2[10];
double vdelta_var2[10];
double videnticalmatrix[10] = { 1,1,1,1,1,1,1,1,1,1 };
double videnticalmatrix2[10] = { 1,1,1,1,1,1,1,1,1,1 };
double vreceptive[10] = { 0,0,0,0,0,0,0,0,0,0 };


double error_v, d_error_v, last_error_v = 0;
double delta_v,vuc;
double vdelta=0,wdelta=0;
double error_theta, d_error_theta;

//v part=======================

double sliding,sliding2;
int i;
double tu_v,tu_w;


int lamda1 = 1;
int lamda2 = 1;
int lamda3 = 20;
int lamda4 = 5;

double pi = 3.14159265359;
double wd, vd, wr = 3.14159265359 / 45, vr;
//Brain parameter set===============
double delta = 0;
double xv = 0,v = 0,w = 0;
double d_x = 0, d_y = 0, d_delta = 0;
double x = 0, y = 0;
double xr = 0, yr = 0;
double deltar=0;

double error_xv;

double p_vd = 0,p_wd = 0;
double p_xr = 0, p_yr = 0;
double error_delta;
double error_x = 0, error_y = 0;
double xvd;
double T = 0;


int sign(double j) {
	int a;
	if (j >= 0) {
		a=1; //true is positve
	}
	else {
		a=-1; //False is negative
	}
	return a;
}


void posture_controller() {


	
	xr = -1* cos(wr*t)+1;
	yr = 1 * sin(wr*t);


	/*
	if (T < 5) {
		xr = 0;
		yr = 0;
		wr = 0;
	}
	else {
		xr = 0;
		yr = T;
		wr = 3.14159265359 / 45;
	}
	*/


	deltar = fmod((deltar + (wd)*deltaT ), (2 * pi));

	vr = abs(pow((xr - p_xr), 2) + pow(pow((yr - p_yr), 2),0.5)) / deltaT;
	//= == == == == == == == == == == == == == == == ==

	//= == == == == == == == == == == == == == == == ==



	//= == == == == == == == == == == == == == == == ==

	//obtain error_x, error_y, error_delta
    error_x = (xr - x)*cos(delta) + (yr - y)*sin(delta);
	error_y = (xr - x)*-sin(delta) + (yr - y)*cos(delta);
	error_delta = deltar - delta;

	//= == == == == == == == == == == == == == == == ==


	wd = wr + 2 * lamda3*vr*error_y*cos(error_delta / 2) + lamda4*sin(error_delta / 2);
	
	if (abs(wd) < pi/4) {
		wd = wd;
	}
	else if (wd > pi/4) {
		wd = pi / 4;
	}
	else {
		wd = -pi / 4;
	}
	
	vd = vr*cos(error_delta) + lamda1*tanh(w)*w*error_x - lamda1*tanh(w)*(vr*sin(error_delta) + lamda2*error_y) + lamda2*error_x - lamda1*(1 - pow(tanh(w) , 2))*d_w*error_y;
	
	if (abs(vd) < 40) {
		vd = vd;
	}
	else if(vd > 40) {
		vd = 40;
	}
	else {
		vd = -40;
	}

	xvd = pow((pow(error_x ,2) + pow(error_y, 2)), 0.5);

	error_w = -(wd - w);
	error_v = -(vd - v);
	error_xv = -(xvd - xv);
}






void pidw()
{



	d_error_w = (error_w - last_error_w);
	last_error_w = error_w;

	if (error_w > 5) {
		tu_w = 5 * 3;
	}
	if (error_w < -5) {
		tu_w = -5 * 3;
	}
	if (abs(error_w) <= 5) {
		tu_w = error_w * 3;
	}

	wdelta = error_w + d_error_w;
	//direction()
	//cmac////////////////////////////////////////////////////////////////


	//adative laws == == == == == == == == == == == =

	for (i = 1; i <= value; i++) {
		delta_weight_a[i] = -0.1*wdelta*receptive[i];
		//delta_weight_p(i, :) = 0.1*delta*receptive[i];

		delta_mean[i] = -0.01*wdelta*(weight_a[i])*receptive[i] * (2 * (identicalmatrix[i] - mean[i]) / pow(var[i],2));
		delta_var[i] = -0.01*wdelta*(weight_a[i])*receptive[i] * (2 * pow((identicalmatrix[i] - mean[i]), 2) / pow(var[i], 3));

		delta_mean2[i] = -0.01*wdelta*(weight_a[i])*receptive[i] * (2 * (identicalmatrix2[i] - mean2[i]) / pow(var2[i], 2));
		delta_var2[i] = -0.01*wdelta*(weight_a[i])*receptive[i] * (2 * pow((identicalmatrix2[i] - mean2[i]) ,2) / pow(var2[i], 3));
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


	double identicalmatrix[10] = { 1,1,1,1,1,1,1,1,1,1 };
	double identicalmatrix2[10] = { 1,1,1,1,1,1,1,1,1,1 };

	for (i = 1; i <= value; i++) {
		identicalmatrix[i] = identicalmatrix[i] * error_w;
		identicalmatrix2[i] = identicalmatrix2[i] * d_error_w;
	}

    //%receptive layer == == == == == =
	for (i = 1; i <= value; i++) {
		s[i] = exp(-pow(error_w - mean[i], 2) / pow(var[i], 2));
		s2[i] = exp(-pow(d_error_w - mean2[i], 2) / pow(var2[i], 2));
		receptive[i] = s[i]*s2[i];
		uc = uc + receptive[i] * weight_a[i];
	}

	//////////////////////////////////////////////////////////////////////



	tu_w = -tu_w*7-0*uc;

}




void pidv()
{


	error_theta = last_y - angle_offset;

	//d_error = (error_theta - last_error_theat) / deltaT;
	//last_error = error;



	if (error_theta > 3) {
		tu_v = error_theta * 10*2;

	}
	if (error_theta < -3) {
		tu_v = error_theta * 10*2;

	}
	if (abs(error_theta) <= 3) {
		tu_v = error_theta * 10*2;

	}
	
	if (vd < 0) {
		if (angle_offset > -2/*-1*/) {
			angle_offset = angle_offset - 0.001*abs(vd);
		}
	}
	

	d_error_v = error_v - last_error_v;
	vdelta = error_v + d_error_v;
	last_error_v = error_v;
	//direction()
	//cmac////////////////////////////////////////////////////////////////


	//adative laws == == == == == == == == == == == =

	for (i = 1; i <= value; i++) {
		vdelta_weight_a[i] = -0.1*vdelta*vreceptive[i];
		//delta_weight_p(i, :) = 0.1*delta*receptive[i];

		vdelta_mean[i] = -0.01*vdelta*(vweight_a[i])*vreceptive[i] * (2 * (videnticalmatrix[i] - vmean[i]) / pow(vvar[i], 2));
		vdelta_var[i] = -0.01*vdelta*(vweight_a[i])*vreceptive[i] * (2 * pow((videnticalmatrix[i] - vmean[i]) ,2) / pow(vvar[i], 3));

		vdelta_mean2[i] = -0.01*vdelta*(weight_a[i])*vreceptive[i] * (2 * (videnticalmatrix2[i] - vmean2[i]) / pow(vvar2[i], 2));
		vdelta_var2[i] = -0.01*vdelta*(weight_a[i])*vreceptive[i] * (2 * pow((videnticalmatrix2[i] - vmean2[i]), 2) / pow(vvar2[i], 3));
		//update weight mean variance == == == == == == == == == ==


		vweight_a[i] = vweight_a[i] + vdelta_weight_a[i];
		//weight_p[i] = weight_p[i] + delta_weight_p[i];

		vmean[i] = vmean[i] + vdelta_mean[i];
		vvar[i] = vvar[i] + vdelta_var[i];

		vmean2[i] = vmean2[i] + vdelta_mean2[i];
		vvar2[i] = vvar2[i] + vdelta_var2[i];
	}

	//= == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == ==
	//controller


	double videnticalmatrix[10] = { 1,1,1,1,1,1,1,1,1,1 };
	double videnticalmatrix2[10] = { 1,1,1,1,1,1,1,1,1,1 };

	for (i = 1; i <= value; i++) {
		videnticalmatrix[i] = videnticalmatrix[i] * error_v;
		videnticalmatrix2[i] = videnticalmatrix2[i] * d_error_v;
	}

	//%receptive layer == == == == == =
	for (i = 1; i <= value; i++) {
		vs[i] = exp(-pow(error_v - vmean[i], 2) / pow(vvar[i], 2));
		vs2[i] = exp(-pow(d_error_v - vmean2[i], 2) / pow(vvar2[i], 2));
		vreceptive[i] = vs[i] * vs2[i];
		vuc = vuc + vreceptive[i] * vweight_a[i];
	}

	//////////////////////////////////////////////////////////////////////


	tu_v = tu_v-0*vuc;

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


  while(T<=22.5) {
	  T = T + deltaT;
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
    
    if (last_y < -60.0 || last_y > 60.0) 
      stop_motors();


	posture_controller();
    pidw();
	pidv();



	speed_r = (tu_v + tu_w) / 2;
	speed_l = (tu_v - tu_w) / 2;

	if (speed_r > 100) {
		speed_r = 100;
	}
	if (speed_r < -100) {
		speed_r = -100;
	}
	if (speed_l > 100) {
		speed_l = 100;
	}
	if (speed_l < -100) {
		speed_l = -100;
	}

	//printf("\n%lf\tLeft:%lf\tRight:%lf\t\n", error_theta, speed_r, speed_l);
	//printf("%1f\t%1f\t%1f\n", d_error, sliding, tu_v);
	printf("T:%lf\tdeltaT:%lf\ttimer:%lf\n",T,deltaT,timer);
	printf("vd:%lf\n", vd);
	printf("v:%lf\n", v);
	printf("tu_w:%lf\n",tu_w);
	printf("offset:%lf\n", angle_offset);
	printf("R:%lf\tL:%lf\n\n", speed_r,speed_l);
    motors(speed_r,speed_l, 0.0, 0.0);
    
	//===========================================================

	delta = delta + deltaT*w;
	delta = fmod(delta, (2 * pi));

	v = -5.2726*0.2323*(speed_r + speed_l)*0.5;
	w = gyro_scaled_z*pi / 180;
	d_w = (w - p_w) / deltaT;
	p_w = w;
	xv = v*deltaT;

	//tracking error dynamics
	d_x = cos(delta)*v;
	d_y = sin(delta)*v;
	d_delta = w;

	x = x + d_x*deltaT;
	y = y + d_y*deltaT;
	//============================================================
    delay(10);


  }

  stop_motors();
  return 0;
}
