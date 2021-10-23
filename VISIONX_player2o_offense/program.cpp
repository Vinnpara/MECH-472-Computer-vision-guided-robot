
#include <cstdio>
#include <iostream>
#include <fstream>

#include <Windows.h>

using namespace std; 

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

#include "image_transfer5.h"

// include this header file for computer vision functions
#include "vision.h"

#include "robot.h"

#include "vision_simulation.h"

#include "timer.h"

extern robot_system S1;

int centroid_red(image &rgb, int &icr, int &jcr, int &f);
int centroid_green(image &rgb, int &icg, int &jcg, int &b);
int calculate_position(int ires, int jres, double &angle);
int centroid_blue_op(image &rgb, int &icb, int &jcb);
int centroid_orange_op(image &rgb, int &ico, int &jco);
int point_robot(int icg, int jcg, int ico, int jco, double beta, int &PW_L, int &PW_R, double &a, int &l, int &PW_Laser);
void move_robot(int &pw_l, int &pw_r);
int cover_object(image &grey, int is, int js);
int avoid_obstacles(int dis_front, int dis_back, double ob_angle, double beta, int &PW_L, int &PW_R);

const double pi = 3.14159;
const double pi_inv = 1 / pi;

int main()
{
	double x0, y0, theta0, max_speed, opponent_max_speed;
	int pw_l, pw_r, pw_laser, laser;
	double light, light_gradient, light_dir, image_noise;
	double width1, height1;
	int N_obs, n_robot;
	double x_obs[50], y_obs[50], size_obs[50];
	double D, Lx, Ly, Ax, Ay, alpha_max;
	double tc, tc0; // clock time
	int mode, level;
	int pw_l_o, pw_r_o, pw_laser_o, laser_o;
	
	// TODO: it might be better to put this model initialization
	// section in a separate function
	
	// note that the vision simulation library currently
	// assumes an image size of 640x480
	width1  = 640;
	height1 = 480;
	
	// number of obstacles
	N_obs  = 2;

	x_obs[1] = 270; // pixels
	y_obs[1] = 270; // pixels
	size_obs[1] = 1.0; // scale factor 1.0 = 100% (not implemented yet)	

	x_obs[2] = 135; // pixels
	y_obs[2] = 135; // pixels
	size_obs[2] = 1.0; // scale factor 1.0 = 100% (not implemented yet)	

	// set robot model parameters ////////
	
	D = 121.0; // distance between front wheels (pixels)
	
	// position of laser in local robot coordinates (pixels)
	// note for Lx, Ly we assume in local coord the robot
	// is pointing in the x direction		
	Lx = 31.0;
	Ly = 0.0;
	
	// position of robot axis of rotation halfway between wheels (pixels)
	// relative to the robot image center in local coordinates
	Ax = 37.0;
	Ay = 0.0;
	
	alpha_max = 3.14159/2; // max range of laser / gripper (rad)
	
	// number of robot (1 - no opponent, 2 - with opponent, 3 - not implemented yet)
	n_robot = 2;
	
	cout << "\npress space key to begin program.";
	pause();

	// you need to activate the regular vision library before 
	// activating the vision simulation library
	activate_vision();

	// note it's assumed that the robot points upware in its bmp file
	
	// however, Lx, Ly, Ax, Ay assume robot image has already been
	// rotated 90 deg so that the robot is pointing in the x-direction
	// -- ie when specifying these parameters assume the robot
	// is pointing in the x-direction.

	// note that the robot opponent is not currently implemented in 
	// the library, but it will be implemented soon.

	activate_simulation(width1,height1,x_obs,y_obs,size_obs,N_obs,
		"robot_A.bmp","robot_B.bmp","background.bmp","obstacle.bmp",D,Lx,Ly,
		Ax,Ay,alpha_max,n_robot);	

	// open an output file if needed for testing or plotting
//	ofstream fout("sim1.txt");
//	fout << scientific;
	
	// set simulation mode (level is currently not implemented)
	// mode = 0 - single player mode (manual opponent)
	// mode = 1 - two player mode, player #1
	// mode = 2 - two player mode, player #2	
	mode = 2;
	level = 1;
	set_simulation_mode(mode,level);	
	
	// set robot initial position (pixels) and angle (rad)
	x0 = 150;
	y0 = 375;
	theta0 = 0;
	set_robot_position(x0,y0,theta0);
	
	// set opponent initial position (pixels) and angle (rad)
//	x0 = 150;
//	y0 = 375;
//	theta0 = 3.14159/4;
//	set_opponent_position(x0,y0,theta0);

	// set initial inputs / on-line adjustable parameters /////////

	// inputs
	pw_l = 1500; // pulse width for left wheel servo (us)
	pw_r = 1500; // pulse width for right wheel servo (us)
	pw_laser = 1500; // pulse width for laser servo (us)
	laser = 0; // laser input (0 - off, 1 - fire)
	
	// paramaters
	max_speed = 100; // max wheel speed of robot (pixels/s)
	opponent_max_speed = 100; // max wheel speed of opponent (pixels/s)
	
	// lighting parameters (not currently implemented in the library)
	light = 1.0;
	light_gradient = 1.0;
	light_dir = 1.0;
	image_noise = 1.0;

	// set initial inputs
	set_inputs(pw_l,pw_r,pw_laser,laser,
		light,light_gradient,light_dir,image_noise,
		max_speed,opponent_max_speed);

	// NOTE: for two player mode you shouldn't set the opponent inputs

	// opponent inputs
//	pw_l_o = 1300; // pulse width for left wheel servo (us)
//	pw_r_o = 1600; // pulse width for right wheel servo (us)
//	pw_laser_o = 1500; // pulse width for laser servo (us)
//	laser_o = 0; // laser input (0 - off, 1 - fire)

	// manually set opponent inputs for the simulation
	// -- good for testing your program
//	set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o, 
//				opponent_max_speed);

	// regular vision program ////////////////////////////////
	
	// note that at this point you can write your vision program
	// exactly as before.
	
	// |||||||| ||      |||||||| ||  || ||||||  |||||||    |||||   ||||||| |||||| |||||| |||||| |||     ||	|||||| ||||||
	// ||    || ||      ||    || ||  || ||      ||    ||  ||   ||  ||   || ||     ||     ||     || ||   ||  ||     ||
	// |||||||| ||      |||||||| |||||| ||||||  |||||||       ||   ||   || |||||| |||||| |||||| ||  ||  ||  |||||| ||||||
	// ||       ||      ||    ||   ||   ||      ||    ||    |||    ||   || ||     ||     ||     ||   || ||      || ||
	// ||       ||||||| ||    ||   ||   ||||||  ||    ||  |||||||  ||||||| ||     ||     |||||| ||     |||  |||||| ||||||


	// in addition, you can set the robot inputs to move it around
	// the image and fire the laser.
	
	image rgb, greya, greyb, label, rgb0;
	int height, width, i, b, f, s;
	int nlabels;
	static int init = 0;
	int ig, jg, ir, jr, ibo, jbo, ioo, joo, l;
	int ires, jres, ires_op, jres_op;
	int PW_LP, PW_RP;
	double beta, beta_op;
	double iro, jro, a, dis_front[51], dis_back[51];
	double i_ob[51], j_ob[51], ob_angle[51];

	// note that the vision simulation library currently
	// assumes an image size of 640x480
	width = 640;
	height = 480;

	rgb.type = RGB_IMAGE;
	rgb.width = width;
	rgb.height = height;

	rgb0.type = RGB_IMAGE;
	rgb0.width = width;
	rgb0.height = height;

	greya.type = GREY_IMAGE;
	greya.width = width;
	greya.height = height;

	greyb.type = GREY_IMAGE;
	greyb.width = width;
	greyb.height = height;

	label.type = LABEL_IMAGE;
	label.width = width;
	label.height = height;

	// allocate memory for the images
	allocate_image(rgb);
	allocate_image(rgb0);
	allocate_image(greya);
	allocate_image(greyb);
	allocate_image(label);


	join_player();

	// measure initial clock time
	tc0 = high_resolution_time(); 

	while(1) {
		
		// simulates the robots and acquires the image from simulation
		acquire_image_sim(rgb);

		tc = high_resolution_time() - tc0;

		//this functions below moves the robot from the keyboard to find a certain coordinates or for fun
		//move_robot(pw_l, pw_r);//move the robot

		centroid_green(rgb, ig, jg, b);
		centroid_red(rgb, ir, jr, f);
		centroid_blue_op(rgb, ibo, jbo);
		centroid_orange_op(rgb, ioo, joo);

		//I COULD'NT PUT THE OBSTACLES CENTROID CALCULATIONS BELOW INTO A FUNCTION
		if (init<4) {
			copy(rgb, rgb0);//copy the original to use the color centroid functions on it
	

			copy(rgb0, greya);
			scale(greya, greyb);
			copy(greyb, greya); // put result back into image a

			cover_object(greya, ig, jg);
			cover_object(greya, ir, jr);
			cover_object(greya, ibo, jbo);
			cover_object(greya, ioo, joo);

			threshold(greya, greyb, 100);
			copy(greyb, greya);
			copy(greya, rgb0);
	
			invert(greya, greyb);
			copy(greyb, greya);

			label_image(greya, label, nlabels);
			for (i = 1; i<nlabels + 1; i++) centroid(greya, label, i, i_ob[i], j_ob[i]);

			init ++;//so that any error about the position of robots to recheck because IT HAPPENED IN ThIS ONE
		}//-N->end of if

		for (i = 1; i < nlabels + 1; i++) draw_point_rgb(rgb, (int)i_ob[i], (int)j_ob[i], 255, 255, 255);

		draw_point_rgb(rgb, ir, jr, 0, 255, 0);
		draw_point_rgb(rgb, ig, jg, 255, 0, 0);
		draw_point_rgb(rgb, ibo, jbo, 255, 255, 0);
		draw_point_rgb(rgb, ioo, joo, 0, 0, 255);

		ires = ig - ir;
		jres = jg - jr;
		calculate_position(ires, jres, beta);

		ires_op = ioo - ibo;
		jres_op = joo - jbo;
		calculate_position(ires_op, jres_op, beta_op);

		//cout << "\n"<<ig <<"\t"<< jg <<"\t"<< ir <<"\t" <<jr <<"\t" <<ibo <<"\t" <<jbo <<"\t" <<ioo <<"\t" <<joo<< "\t" << beta << "\t" << a;

		//this below is for obstacle avoidance
		for (i = 1; i < nlabels + 1; i++){
			dis_front[i] = (i_ob[i] - ig)*(i_ob[i] - ig) + (j_ob[i] - jg)*(j_ob[i] - jg);
			dis_back[i] = (i_ob[i] - ir)*(i_ob[i] - ir) + (j_ob[i] - jr)*(j_ob[i] - jr);

			ob_angle[i] = atan2((j_ob[i] - jg), (i_ob[i] - ig)) * 180 * pi_inv;
			if (ob_angle[i] < 0)ob_angle[i] = 360 - fabs(ob_angle[i]);
			//	cout << "\tdistance " << i << " = " << dis_front[i] << "\tangle " << i << " = " << ob_angle[i];

			if (dis_front[i] <= 10000) {

				avoid_obstacles(dis_front[i], dis_back[i], ob_angle[i], beta, pw_l, pw_r);
				break;
			}
			if (dis_back[i] <= 8000){
				pw_l = 1000;
				pw_r = 2000;
				break;
			}
			else 	point_robot(ig, jg, ioo, joo, beta, pw_l, pw_r, a, l, pw_laser);


		}//end of for loop
		if (l == 1)laser = 1;
		//else laser = 0;

		//the if below is for blind movement
		if (f == 1){
			pw_l = 1000;
			pw_r = 2000;
		}
		if (b == 1){
			pw_l = 2000;
			pw_r = 1000;
		}
		set_inputs(pw_l,pw_r,pw_laser,laser,
			light,light_gradient,light_dir,image_noise,
			max_speed,opponent_max_speed);

		// manually set opponent inputs for the simulation
		// -- good for testing your program
//		set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o, 
//					opponent_max_speed);

		// NOTE: only one player can use image_view()
		//view_rgb_image(rgb);

		// don't need to simulate too fast
		Sleep(10); // 100 fps max
	}

	// free the image memory before the program completes
	free_image(rgb);
	free_image(rgb0);
	free_image(greya);
	free_image(greyb);
	free_image(label);


	deactivate_vision();
	
	deactivate_simulation();	
	
	cout << "\ndone.\n";

	return 0;
}

//BELOW ARE MY FUNCTIONS USED FOR THE PROGRAM
int centroid_red(image &rgb, int &icr, int &jcr, int &f){
	ibyte *pr;
	int i, j, height, width;
	int mr, mir, mjr;
	int B, G, R;


	pr = rgb.pdata;

	width = rgb.width;
	height = rgb.height;

	mr = 0, mir = 0, mjr = 0;

	for (j = 0; j < height; j++){
		for (i = 0; i < width; i++){
			B = *pr;
			G = *(pr + 1);
			R = *(pr + 2);
			if (R > 200 && G < 100 && B < 100){//must check the picture with irfan view to get the perfect 3-color intensity
				mr += R;
				mir += R*i;
				mjr += R*j;
			}
			pr += 3;
		}//end of i loop
	}//end of j loop
	if (mr == 0) {
		cout << "\nCOLOR RED NOT FOUND";
		f = 1;
		return 1;
	}
	icr = mir / mr;
	jcr = mjr / mr;
	f = 0;
	return 0;
}//end of function

int centroid_green(image &rgb, int &icg, int &jcg, int &b){
	ibyte *pg;
	int i, j, height, width;
	int mg, mig, mjg;
	int B, G, R;


	pg = rgb.pdata;

	width = rgb.width;
	height = rgb.height;

	mg = 0, mig = 0, mjg = 0;

	for (j = 0; j < height; j++){
		for (i = 0; i < width; i++){
			B = *pg;
			G = *(pg + 1);
			R = *(pg + 2);
			if (R < 80 && G > 170 && B < 150){
				mg += G;
				mig += G*i;
				mjg += G*j;
			}
			pg += 3;
		}//end of i loop
	}//end of j loop
	if (mg == 0){
		cout << "\nCOLOR GREEN NOT FOUND";
		b = 1;
		return 1;
	}
	icg = mig / mg;
	jcg = mjg / mg;
	b = 0;
	return 0;


}//end of function


int calculate_position(int ires, int jres, double &angle){
	double rad;



	rad = atan2(jres, ires);

	angle = rad * 180 * pi_inv;

	if (angle < 0) angle = 360 - abs(angle);//takes the whole 360 degrees


	return 0;

}

int centroid_blue_op(image &rgb, int &icb, int &jcb){
	ibyte *pb;
	int i, j, height, width;
	int mb, mib, mjb;
	int B, G, R;


	pb = rgb.pdata;

	width = rgb.width;
	height = rgb.height;

	mb = 0, mib = 0, mjb = 0;

	for (j = 0; j < height; j++){
		for (i = 0; i < width; i++){
			B = *pb;
			G = *(pb + 1);
			R = *(pb + 2);
			if (R <60 && G > 150 && B > 200){//must check the picture with irfan view-N-> to get the perfect 3-color intensity
				mb += B;
				mib += B*i;
				mjb += B*j;
			}
			pb += 3;
		}//end of i loop
	}//end of j loop
	if (mb == 0) {
		cout << "\nCOLOR BLUE NOT FOUND";
		return 1;
	}
	icb = mib / mb;
	jcb = mjb / mb;

	return 0;
}//end of function

int centroid_orange_op(image &rgb, int &ico, int &jco){
	ibyte *po;
	int i, j, height, width;
	int mo, mio, mjo;
	int B, G, R;


	po = rgb.pdata;

	width = rgb.width;
	height = rgb.height;

	mo = 0, mio = 0, mjo = 0;

	for (j = 0; j < height; j++){
		for (i = 0; i < width; i++){
			B = *po;
			G = *(po + 1);
			R = *(po + 2);
			if (R > 200 && G > 180 && G<200 && B<130 && B > 110){
				mo += (G + R);
				mio += (G + R)*i;
				mjo += (G + R)*j;
			}

			po += 3;
		}//end of i loop
	}//end of j loop
	if (mo == 0) {
		cout << "\nCOLOR ORANGE NOT FOUND";
		return 1;
	}
	ico = mio / mo;
	jco = mjo / mo;

	return 0;


}//end of function

int point_robot(int icg, int jcg, int ico, int jco, double beta, int &PW_L, int &PW_R, double &a, int &l, int &PW_Laser){

	//double iro, jro;
	int angle1, angle2, target;

	angle1 = (int)beta;
	l = 0;

	//iro = (icb + ico) / 2;//thought so it targets-N-> the center of the opponent robot
	//jro = (jcb + jco) / 2;

	a = atan2((jco - jcg), (ico - icg)) * 180 * pi_inv;//angle of line between center of opponent robot and green
	target = ((jco - jcg)*(jco - jcg) + (ico - icg)*(ico - icg));
	if (a < 0)a = 360 - abs(a);//so it will range the whole 360 degrees

	angle2 = (int)a;



	if (angle1 / 10 == angle2 / 10) {
		PW_L = 1000;
		PW_R = 2000;//move forward
		if (target <= 20000) {
			PW_L = PW_R = 1500;
			l = 1;//so that it will be close 
			return 0;
		}
	}

	else if (angle1 < angle2) {// move to the left
		PW_L = 1900;
		PW_R = 1900;
	}

	else if (angle1 > angle2){ // move to the right
		PW_L = 1100;
		PW_R = 1100;
	}

	//this is needed because it will assume that angle2>angle1 and-N-> move to the left instead of right
	if (angle1 >= 0 && angle1 <= 90 && angle2 >= 270 && angle2 <= 359){
		PW_L = 1100;
		PW_R = 1100;

	}

	//this is needed because it will assume that angle2>angle1 and move to the right instead of left
	if (angle2 >= 0 && angle2 <= 90 && angle1 >= 270 && angle1 <= 359){
		PW_L = 1900;
		PW_R = 1900;

	}

	return 0;
}//end of function

void move_robot(int &pw_l, int &pw_r){

	if (KEY(VK_DOWN)){

		if (pw_l > 2000)pw_l = 2000;//limit the max speed of the motor
		if (pw_r < 1000) pw_r = 1000;
		pw_l = 2000;
		pw_r = 1000;
	}
	if (KEY(VK_UP)){

		if (pw_l < 1000)pw_l = 1000;//limit the reverse max speed
		if (pw_r > 2000)pw_r = 2000;
		pw_l = 1000;
		pw_r = 2000;
	}
	if (KEY(VK_SPACE)){
		pw_l = 1500;
		pw_r = 1500;
	}


	if (KEY(VK_LEFT)){

		if (pw_l > 2000)pw_l = 2000;//limit the max speed of the motor
		if (pw_r < 1000) pw_r = 1000;
		pw_l = 2000;
		pw_r = 2000;
	}
	if (KEY(VK_RIGHT)){

		if (pw_l < 1000)pw_l = 1000;//limit the reverse max speed
		if (pw_r > 2000)pw_r = 2000;
		pw_l = 1000;
		pw_r = 1000;
	}

	if (!KEY(VK_UP) && !KEY(VK_DOWN && !KEY(VK_LEFT) && !KEY(VK_RIGHT))){
		if (pw_l > 1500)pw_l -= 50;
		else if (pw_l < 1500)pw_l += 50;
		if (pw_r < 1500)pw_r += 50;
		else if (pw_r>1500)pw_r -= 50;
	}
}//end of functions

int cover_object(image &grey, int is, int js)
{
	ibyte *pa;
	double r, rmax, dr, s, smax, ds, theta;
	int i, j;

	pa = grey.pdata;


	rmax = 60.0; // maximum radius of that will turn white (pixels)
	dr = 0.5; // delta radius  (pixels)
	ds = 0.5; // delta arc-length (pixels)

	// -N->covering pixels in an outward concentic ring pattern
	for (r = 1.0; r <= rmax; r += dr) {
		smax = 2 * 3.1416*r; // maximum arc length
		for (s = 0; s <= smax; s += ds) {
			theta = s / r; // s = r*theta
			i = (int)(is + r*cos(theta));
			j = (int)(js + r*sin(theta));
			*(pa + j*grey.width + i) = 255;

			// limit (i,j) from going off the image
			if (i < 0) i = 0;
			if (i > grey.width - 1) i = grey.width - 1;
			if (j < 0) j = 0;
			if (j > grey.height - 1) j = grey.height - 1;

		}
	}

	return 0;
}//end of functions

int avoid_obstacles(int dis_front, int dis_back, double ob_angle, double beta, int &PW_L, int &PW_R){

	static int pw_lp, pw_rp;

	if (beta > 0 && beta <= 90){
		if (fabs(beta - ob_angle) >= 90 && ob_angle<180){
			PW_L = 1000;
			PW_R = 2000;
			return 0;
		}
		if (ob_angle >= 270)
		{
			//move to the left
			PW_L = 1700;
			PW_R = 1900;

			if (abs(beta + (360 - ob_angle)) >= 90){
				PW_L = 1000;
				PW_R = 2000;
				return 0;
			}
		}
		else if (ob_angle > beta){
			//move to the right
			PW_L = 1100;
			PW_R = 1300;
		}

		else{
			//move to the left
			PW_L = 1700;
			PW_R = 1900;
		}
		pw_lp = PW_L;
		pw_rp = PW_R;
	}//0-90 

	else if (beta > 90 && beta <= 180){
		if (fabs(beta - ob_angle) >= 90){
			PW_L = 1000;
			PW_R = 2000;
			return 0;
		}
		if (ob_angle > beta){
			//move to the right
			PW_L = 1100;
			PW_R = 1300;
		}
		else{
			//move to the left
			PW_L = 1700;
			PW_R = 1900;
		}
		pw_lp = PW_L;
		pw_rp = PW_R;
	}//up front90-180

	else if (beta > 180 && beta <= 270){
		if (fabs(beta - ob_angle) >= 90){
			PW_L = 1000;
			PW_R = 2000;
			return 0;
		}
		if (ob_angle > beta){
			//move to the right
			PW_L = 1100;
			PW_R = 1300;
		}
		else{
			//move to the left
			PW_L = 1700;
			PW_R = 1900;
		}
		pw_lp = PW_L;
		pw_rp = PW_R;
	}//left180-270

	else if (beta > 270 && beta <= 360){
		if (fabs(beta - ob_angle) >= 90){
			PW_L = 1000;
			PW_R = 2000;
			return 0;
		}
		if (ob_angle > beta){
			//move to the right
			PW_L = 1100;
			PW_R = 1300;
		}
		else if (ob_angle > 0 && ob_angle <= 90){
			//move to the right
			PW_L = 1100;
			PW_R = 1300;

		}
		else{
			//move to the left
			PW_L = 1700;
			PW_R = 1900;
		}
		pw_lp = PW_L;
		pw_rp = PW_R;

	}//down270-360

	//////////////////////////////////////////////////////////////////////////


	return 0;
}
