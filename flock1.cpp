/*
* Author: Alejandrio Vasay
* Course: Physically-Based Simulation and Modeling
* Program: This OpenGL application simulates flocking and prey-predator
*	       relationship.
*/
#include "cs649.h"

#include <sstream> 
/////////////////////////////////////
// DEFINES
////////////////////////////////////
#define BOXSIZE			40.0
#define OBJECTRADIUS	1.5
#define MAXTRIANGLES	60
#define MAXRAVERTICES	200

/////////////////////////////////////
// ACCELERATION ATTRIBUTES
/////////////////////////////////////

int fileIncr = 0;
vec3d windvelocity;
float dt			 = 0.01;
float e				 = 0.6;
float k				 = 0.9;
float xgravity		 = 0.0;
float ygravity		 = 0.01;
float zgravity		 = 0.0;
float raForce		 = 0.3;
float raMass		 = 5.0;
float raFHab		= 0.5;
float kmatch_init	 = 0.01;	float kmatching		= kmatch_init;
float kcenter_init   = 0.003;	float kcentering	= kcenter_init;
float kavoid		 = 0.5;
float objgrav_init   = 0.9;		float objgravity	= objgrav_init;
float predgravity	 = 3.5;
float food_init		 = 5.5;		float foodgravity	= food_init;
float love_init		 = 0.001; 

float habitatAttract = 0.01;
float habitatRepel   = 0.01;
float habitatMass    = 2.0;
float revball_init		= 0.5; float revballgravity = revball_init;
float revballmass    = 1.0;
float revpoint1grav	 = 0.4;
float revpoint1mass  = 1.0;
float attmass		 = 3.0;
float attgravity	 = 6.0;

////////////////////////////////////
// POLYGON, TRIANGLE, PLANE
////////////////////////////////////
tri3d triangle[MAXTRIANGLES]; 
int tricount = 0;
vec3d planevec1, planevec2;
int vert[][3][3] = { 
	                  { {-20, 0, -60}, {-20, 0, -40}, {-20, -50, -40} },
                      { {-20, -50, -40}, {-20, -50, -60}, {-20, 0, -60} },

					  { {-20, 0, -40}, {20, 0, -40}, {20, -50, -40} },
					  { {20, -50, -40}, {-20, -50, -40}, {-20, 0, -40} },

					  { {20, 0, -40}, {20, 0, -60}, {20, -50, -60} },
                      { {20, -50, -60}, {20, -50, -40}, {20, 0, -40} },
                    };
int num_triangles = 6;
//int num_vertices = 3;
//int num_points	 = 3;

vec3d RAVertices[MAXRAVERTICES];
int ravertcount = 0;

/////////////////////////////////////
// OBJECT ARRAY
////////////////////////////////////
int objcount =0;
obj3d boid[MAXOBJECTS]; 
obj3d leader;
int cube;

////////////////////////////////////
// FOOD ARRAY
////////////////////////////////////
int foodcount = 0;
food3d food[MAXOBJECTS];

////////////////////////////////////
// Stolen Food
////////////////////////////////////
int stolencount = 0;
obj3d stolen[MAXOBJECTS];
bool IsFoodPresent = false;

////////////////////////////////////
// PREDATOR ARRAY
////////////////////////////////////
int predcount = 0;
obj3d predator[MAXOBJECTS];
float linger_init = 0.5;
float linger = linger_init;
float angry = -1;
////////////////////////////////////
// Habitat
////////////////////////////////////
tri3d habitat[MAXTRIANGLES], lair[MAXTRIANGLES];
int habcount = 0;
int num_tri_hab = 26;
int habvert[][3][3] = { 
						// small center
	                  { {20, 0, -20}, {60, 0, -20}, {60, 0, -0} },
                      { {60, 0, -0}, {20, 0, -0}, {20, 0, -20} },

					   { {20, 0, -20}, {20, 0, -0}, {10, -50, -0} },
                      { {10, -50, -0}, {10, -50, -20}, {20, 0, -20} },
					  { {20, 0, -0}, {10, -50, 0}, {20, -50, 10} },

					  { {20, 0, -0}, {60, 0, -0}, {60, -50, 10} },
					  { {60, -50, 10}, {20, -50, 10}, {20, 0, -0} },
					  { {60, 0, -0}, {60, -50, 10}, {70, -50, -0} },

					  { {60, 0, -0}, {60, 0, -20}, {70, -50, -20} },
                      { {70, -50, -20}, {70, -50, -0}, {60, 0, -0} },

					  { {60, 0, -20}, {20, 0, -20}, {20, -50, -30} },
                      { {20, -50, -30}, {60, -50, -30}, {60, 0, -20} },


						// Big left
					  { {-70, 30, -110}, {-30, 30, -110}, {-30, 30, -70} },
                      { {-30, 30, -70}, {-70, 30, -70}, {-70, 30, -110} },

					   { {-70, 30, -110}, {-70, 30, -70}, {-90, -50, -70} },
                      { {-90, -50, -70}, {-90, -50, -110}, {-70, 30, -110} },
					  { {-70, 30, -70}, {-90, -50, -70}, {-70, -50, -50} },

					  { {-70, 30, -70}, {-30, 30, -70}, {-30, -50, -50} },
					  { {-30, -50, -50}, {-70, -50, -50}, {-70, 30, -70} },
					  { {-30, 30, -70}, {-30, -50, -50}, {-10, -50, -70} },

					  { {-30, 30, -70}, {-30, 30, -110}, {-10, -50, -110} },
                      { {-10, -50, -110}, {-10, -50, -70}, {-30, 30, -70} },

					  { {-30, 30, -110}, {-70, 30, -110}, {-70, -50, -130} },
                      { {-70, -50, -130}, {-30, -50, -130}, {-30, 30, -110} },

					   { {-100, -50, -100}, {100, -50, -100}, {100, -50, 100} },
                      { {100, -50, 100}, {-100, -50, 100}, {-100, -50, -100} },

					  // Big right
				/*	  { {40, 50, -150}, {80, 50, -150}, {80, 50, -110} },
                      { {80, 50, -110}, {40, 50, -110}, {40, 50, -150} },

					   { {40, 50, -150}, {40, 50, -110}, {20, -50, -110} },
                      { {20, -50, -110}, {20, -50, -150}, {40, 50, -150} },

					  { {40, 50, -110}, {80, 50, -110}, {80, -50, -90} },
					  { {80, -50, -90}, {40, -50, -90}, {40, 50, -110} },

					  { {80, 50, -110}, {80, 50, -150}, {100, -50, -150} },
                      { {100, -50, -150}, {100, -50, -110}, {80, 50, -110} },

					  { {80, 50, -150}, {40, 50, -150}, {40, -50, -160} },
                      { {40, -50, -160}, {80, -50, -160}, {80, 50, -150} },*/



                    };

int num_repelvert = 12;
int repelvert[][3] = { //{40, -40, -150}, {40, -40, -130}, {30, -40, -110},
					   //{70, -40, -110}, {50, 20, -85}, {50, 0, -85},
					   {-80, -40, -110}, {-80, -40, -70}, {-50, -40, -90},

					   //small box
						{10,-30, -20}, {60,-50,-30}, {30,-30,0},

};

/////////////////////////////////////
// Revolving paths that objects follow
//////////////////////////////////////
obj3d revball;
const long int num_of_revolution = 6000;
int ii	= 0;
float ballx, ballz, ymov=0;
float radius;
float revolvedt = 0.05;
obj3d revpoint1;
const long int num_of_revolution2 = 5000;
int ll	= 0;
float ptx, ptz, ymov2=0;
float radius2;
float revolvedt2 = 0.05;
int rightflap = 0.0;
int leftflap = 0.0;
GLfloat texture[5];
////////////////////////////////////
// CAMERA
////////////////////////////////////
float xpos = 0, ypos = 0, zpos = 0, xrot = 0, yrot = 0, zrot = 0, angle2=0.0;
// Move Camera
void camera (void) {
    glRotatef(xrot,1.0,0.0,0.0);	
    glRotatef(yrot,0.0,1.0,0.0);	
	glRotatef(zrot,0.0,0.0,1.0);
    glTranslated(-xpos,-ypos,-zpos);
}

void snapshot(char *fileName)
{
	GLint viewport[4]; 
	glGetIntegerv( GL_VIEWPORT, viewport ); 
	int width;
	int height;
	width  = viewport[2]; 
	height = viewport[3]; 
	width -= width%4; 
	GLubyte * bmpBuffer = NULL; 
	bmpBuffer = (GLubyte *)malloc(width*height*3*sizeof(GLubyte)); 
	if (!bmpBuffer) 
	return ; 
	glReadPixels((GLint)0, (GLint)0, (GLint)width, (GLint)height, 
	GL_BGR_EXT, GL_UNSIGNED_BYTE, bmpBuffer); 
	//	char fileName[500];
	//	strcpy(fileName,dataName);
	//	strcat(fileName,".bmp")
	FILE *filePtr=fopen(fileName, "wb"); 
	//	char *filename
	///(&filePtr, ; 
	if (!filePtr) 
	return ; 
	BITMAPFILEHEADER bitmapFileHeader; 
	BITMAPINFOHEADER bitmapInfoHeader; 
	bitmapFileHeader.bfType = 0x4D42; //"BM" 
	bitmapFileHeader.bfSize = width*height*3; 
	bitmapFileHeader.bfReserved1 = 0; 
	bitmapFileHeader.bfReserved2 = 0; 
	bitmapFileHeader.bfOffBits = sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER); 
	bitmapInfoHeader.biSize = sizeof(BITMAPINFOHEADER); 
	bitmapInfoHeader.biWidth = width; 
	bitmapInfoHeader.biHeight = height; 
	bitmapInfoHeader.biPlanes = 1; 
	bitmapInfoHeader.biBitCount = 24; 
	bitmapInfoHeader.biCompression = BI_RGB; 
	bitmapInfoHeader.biSizeImage = 0; 
	bitmapInfoHeader.biXPelsPerMeter = 0; 
	bitmapInfoHeader.biYPelsPerMeter = 0; 
	bitmapInfoHeader.biClrUsed = 0; 
	bitmapInfoHeader.biClrImportant = 0; 
	fwrite(&bitmapFileHeader, sizeof(bitmapFileHeader), 1, filePtr); 
	fwrite(&bitmapInfoHeader, sizeof(bitmapInfoHeader), 1, filePtr); 
	fwrite(bmpBuffer, width*height*3, 1, filePtr); 
	fclose(filePtr); 
	free(bmpBuffer); 

}

void addRAVertex(vec3d vert) {
	RAVertices[ravertcount] = vert;
	ravertcount++;
}

void buildRepelWalls(int num, int vert2[][3], vec3d &boidpos, vec3d &boidvel, float mass, float force, RepelAttract_t rep) {
	vec3d vrtx;
	for(int j=0; j<num; j++) {
			vrtx.x = vert2[j][0];
			vrtx.y = vert2[j][1];
			vrtx.z = vert2[j][2];
			
			if(distance3d(vrtx, boidpos) < 45.0) {
				repelAttractVelocity(vrtx, raMass, boidpos, raForce, rep, boidvel);
				repelAttractVelocity(vrtx, raMass, boidpos, raForce, rep, boidvel);
			}
	}

}

/////////////////////////////////////////////////////
// Compute attraction of to revolving point
// so they can fly around in a circular manner
/////////////////////////////////////////////////////
void handlePaths() {
	float theta = 2.0f * PI * float(ii) / float(num_of_revolution);
	ballx = 100 * cosf(theta) + 30;
	ballz = 100 * sinf(theta) - 20;
	revball.position.x = ballx;
	revball.position.y = 0;
	revball.position.z = ballz;
	for(int o=0; o<objcount; o++) {
		if(ballz < -30 && ballx < -50 && !IsFoodPresent) {
			//cout << "Should have attraction here" << endl;
			vec3d att; att.x = -60; att.y = 40; att.z = -80;
			repelAttractVelocity(att, 
							attmass, 
							boid[0].position, 
							attgravity, 
							ATTRACT,
							boid[0].velocity);
		}
		if(ballz < -60 && ballx < -40 && !IsFoodPresent) {
			//cout << "Should have attraction here" << endl;
			vec3d att; att.x = -60; att.y = 30; att.z = -110;
			repelAttractVelocity(att, 
							attmass, 
							boid[0].position, 
							attgravity, 
							ATTRACT,
							boid[0].velocity);
			//boid[o].bodyrot = 180;
			//boid[o].axis.x = 0;
			//boid[o].axis.y = 1;
			//boid[o].axis.z = 0;
		}
	}
	//else
	//	attgravity = 3.0;

	float gamma = 2.0f * PI * float(ll) / float(num_of_revolution2);
	ptx = 5 * cosf(gamma) + 40;
	ptz = 5 * sinf(gamma) -10;
	revpoint1.position.x = ptx;
	revpoint1.position.y = 6;
	revpoint1.position.z = ptz;

	repelAttractVelocity(revball.position, 
						revballmass, 
						boid[0].position, 
						revballgravity, 
						ATTRACT,
						boid[0].velocity);
	 for (int pr=0; pr<predcount; pr++)
		repelAttractVelocity(revpoint1.position, 
							revpoint1mass, 
							predator[pr].position, 
							revpoint1grav, 
						    ATTRACT,
							predator[pr].velocity);
	ii++;
    ll++;
    ymov += 0.00;
}

void questionMark() {

}

void drawHex (int num)
{
	GLdouble theta;
	//glLineWidth(2.0);
	glPushMatrix();
	glTranslatef(50, 0, 50);
	glBegin(GL_POLYGON);
	for(int i=0; i<num; i++) {
		glColor3f(0.5,0.5,0.5); 
		theta = 50+ i/num * (2*PI);		
		glVertex3f(sin(theta), cos(theta),0.0);
	}		
    glEnd();
	glPopMatrix();
}

void createHabitat() {
	//drawHex(6);

	habitat[0].topleft.x = -20;
	habitat[0].topleft.y = 0;
	habitat[0].topleft.z = -60;
	habitat[0].topright.x = 20;
	habitat[0].topright.y = 0;
	habitat[0].topright.z = -60;
	habitat[0].botright.x = 20;
	habitat[0].botright.y = 0;
	habitat[0].botright.z = -40;

	habitat[1].topleft.x = 20;
	habitat[1].topleft.y = 0;
	habitat[1].topleft.z = -40;
	habitat[1].topright.x = -20;
	habitat[1].topright.y = 0;
	habitat[1].topright.z = -40;
	habitat[1].botright.x = -20;
	habitat[1].botright.y = 0;
	habitat[1].botright.z = -60;


	glPushMatrix();
	glBegin(GL_TRIANGLES);
		glColor4f(0.2, 0.2, 0.2, 0.40);
		glVertex3f(habitat[0].topleft.x , habitat[0].topleft.y, habitat[0].topleft.z);	
		glVertex3f(habitat[0].topright.x, habitat[0].topright.y, habitat[0].topright.z);	
		glVertex3f(habitat[0].botright.x, habitat[0].botright.y, habitat[0].botright.z);	
    glEnd();
	glBegin(GL_TRIANGLES);
		glColor4f(0.2, 0.2, 0.2, 0.40);
		glVertex3f(habitat[1].topleft.x , habitat[1].topleft.y, habitat[1].topleft.z);	
		glVertex3f(habitat[1].topright.x, habitat[1].topright.y, habitat[1].topright.z);	
		glVertex3f(habitat[1].botright.x, habitat[1].botright.y, habitat[1].botright.z);	
    glEnd();
	glPopMatrix();

}

void createPredator(int num) {
	for(int i=0; i<num; i++) {
		predcount++;
		predator[predcount-1].position.x = 40;
		predator[predcount-1].position.y = 5;
		predator[predcount-1].position.z = -8;

		predator[predcount-1].velocity.x = 0;
		predator[predcount-1].velocity.y = 0;
		predator[predcount-1].velocity.z = 0;

		predator[predcount-1].mass = 5;
		predator[predcount-1].acceleration.x = 0;
		predator[predcount-1].acceleration.y = 0;
		predator[predcount-1].acceleration.z = 0;
	}
}

void createFood(int num) {
	IsFoodPresent = true;
	revballgravity = 0.0;
	for(int i=0; i<num; i++) {
		foodcount++;
		food[foodcount-1].position.x = 25;
		food[foodcount-1].position.y = 1.0;
		food[foodcount-1].position.z = -5;

		food[foodcount-1].velocity.x = 0;
		food[foodcount-1].velocity.y = 0;
		food[foodcount-1].velocity.z = 0;

		food[foodcount-1].acceleration.x = 0;
		food[foodcount-1].acceleration.y = 0;
		food[foodcount-1].acceleration.z = 0;

		food[foodcount-1].color.r = 0;
		food[foodcount-1].color.g = 1;
		food[foodcount-1].color.b = 0;
		food[foodcount-1].alpha	= 1.0;

		food[foodcount-1].mass = 5;
		food[foodcount-1].gravity = food_init;
		food[foodcount-1].love = love_init;
		food[foodcount-1].thief = -1;
		food[foodcount-1].stolen = false;
	}
}

void addObject(int num) {

	for(int i=0; i < num; i++) {
	objcount++;
	boid[objcount-1].acceleration.x = 0.0;
	boid[objcount-1].acceleration.y = 0.0;
	boid[objcount-1].acceleration.z = 0.0;
	boid[objcount-1].mass = 1.0;

	boid[objcount-1].position.x = (rand()%20 + 40);
	boid[objcount-1].position.y = -(rand()%15);
	boid[objcount-1].position.z =  (rand()%20 + 80);

	boid[objcount-1].velocity.x = 0;
	boid[objcount-1].velocity.y = 0;
	boid[objcount-1].velocity.z = -(rand()%5);

	boid[objcount-1].limbvel = 1;
	boid[objcount-1].leftflap  = rand()%50;
	boid[objcount-1].rightflap = rand()%50;

	boid[objcount-1].color.r = 1;
	boid[objcount-1].color.g = 0;
	boid[objcount-1].color.b = 0;
	boid[objcount-1].alpha	= 1.0;
	}

	////////////////////////////////////////////
	// Just initialize the next one to 0
	// because I have a routine that does something
	// like boid[i+1]
	///////////////////////////////////////////
	boid[objcount].acceleration.x = 0.0;
	boid[objcount].acceleration.y = 0.0;
	boid[objcount].acceleration.z = 0.0;
	boid[objcount].mass = 0.0;

	boid[objcount].position.x = 0;
	boid[objcount].position.y = 0;
	boid[objcount].position.z = 0;

	boid[objcount].velocity.x = 0;
	boid[objcount].velocity.y = 0;
	boid[objcount].velocity.z = 0;

	boid[objcount].color.r = 0;
	boid[objcount].color.g = 0;
	boid[objcount].color.b = 0;
	boid[objcount-1].alpha	= 1.0;

	boid[objcount-1].bodyrot = 0;
	////////////////////////////////
	// Lead boid
	////////////////////////////////
	boid[0].color.r = 0;
	boid[0].color.g = 0;
	boid[0].color.b = 0.0;
	boid[0].alpha	= 0.5;
}

void buildTriangle(int count, poly3s *tri, vec3d tl, vec3d tr, vec3d br) {

		
		tri[count-1].topleft.x = tl.x;
		tri[count-1].topleft.y = tl.y;
		tri[count-1].topleft.z = tl.z;
		tri[count-1].topright.x = tr.x;
		tri[count-1].topright.y = tr.y;
		tri[count-1].topright.z = tr.z;
		tri[count-1].botright.x = br.x;
		tri[count-1].botright.y = br.y;
		tri[count-1].botright.z = br.z;

		/*
		triangle[tricount-1].topleft.x = vert[i][0][0];
		triangle[tricount-1].topleft.y = vert[i][0][1];
		triangle[tricount-1].topleft.z = vert[i][0][2];
		triangle[tricount-1].topright.x = vert[i][1][0];
		triangle[tricount-1].topright.y = vert[i][1][1];
		triangle[tricount-1].topright.z = vert[i][1][2];
		triangle[tricount-1].botright.x = vert[i][2][0];;
		triangle[tricount-1].botright.y = vert[i][2][1];
		triangle[tricount-1].botright.z = vert[i][2][2];
		*/

}

void buildWalls(int &count, int num, poly3s *tri, int vert1[][3][3]) {
	vec3d vrtx[3];
	for(int i=0; i<num; i++) {
		count++;
		for(int j=0; j<3; j++) {
			
				vrtx[j].x = vert1[i][j][0];
				vrtx[j].y = vert1[i][j][1];
				vrtx[j].z = vert1[i][j][2];
			
		}

		buildTriangle(count, tri, vrtx[0],  vrtx[1],  vrtx[2]);
	}
}

void createWalls1() {

		// first triangle
		tricount++;
		vec3d topleft, topright, botright;
		topleft.x = -(rand()%10 + 70);
		topleft.y = rand()%5 + 15;
		topleft.z = -70;
		topright.x = -(rand()%10 + 40);
		topright.y = rand()%5 + 15;
		topright.z = -70;
		botright.x = -(rand()%10 + 40);
		botright.y = -(rand()%10 + 20);
		botright.z = -70;
		triangle[tricount-1].topleft.x = topleft.x;
		triangle[tricount-1].topleft.y = topleft.y;
		triangle[tricount-1].topleft.z = topleft.z;
		triangle[tricount-1].topright.x = topright.x;
		triangle[tricount-1].topright.y = topright.y;
		triangle[tricount-1].topright.z = topright.z;
		triangle[tricount-1].botright.x = botright.x;;
		triangle[tricount-1].botright.y = botright.y;
		triangle[tricount-1].botright.z = botright.z;

		// second triangle
		tricount++;
		vec3d topleft2, topright2, botright2;
		topleft2.x = -(rand()%10 + 40);
		topleft2.y = -(rand()%10 + 20);
		topleft2.z = -80;
		topright2.x = -(rand()%10 + 70);
		topright2.y = -(rand()%10 + 20);
		topright2.z = -80;
		botright2.x = -(rand()%10 + 70);
		botright2.y = rand()%5 + 15;
		botright2.z = -80;
		triangle[tricount-1].topleft.x = topleft2.x;
		triangle[tricount-1].topleft.y = topleft2.y;
		triangle[tricount-1].topleft.z = topleft2.z;
		triangle[tricount-1].topright.x = topright2.x;
		triangle[tricount-1].topright.y = topright2.y;
		triangle[tricount-1].topright.z = topright2.z;
		triangle[tricount-1].botright.x = botright2.x;;
		triangle[tricount-1].botright.y = botright2.y;
		triangle[tricount-1].botright.z = botright2.z;

		/*
		*/
		//addRAVertex(triangle[tricount-1].topleft);
		//addRAVertex(triangle[tricount-1].botright);
}

void createWalls2() {

		// first triangle
		tricount++;
		vec3d topleft, topright, botright;
		topleft.x = -70;
		topleft.y = 20;
		topleft.z = 10;
		topright.x = -20;
		topright.y = 20;
		topright.z = 10;
		botright.x = -20;
		botright.y = -25;
		botright.z = 10;
		triangle[tricount-1].topleft.x = topleft.x;
		triangle[tricount-1].topleft.y = topleft.y;
		triangle[tricount-1].topleft.z = topleft.z;
		triangle[tricount-1].topright.x = topright.x;
		triangle[tricount-1].topright.y = topright.y;
		triangle[tricount-1].topright.z = topright.z;
		triangle[tricount-1].botright.x = botright.x;;
		triangle[tricount-1].botright.y = botright.y;
		triangle[tricount-1].botright.z = botright.z;

		// second triangle
		tricount++;
		vec3d topleft2, topright2, botright2;
		topleft2.x = -20;
		topleft2.y = -25;
		topleft2.z = 10;
		topright2.x = -70;
		topright2.y = -25;
		topright2.z = 10;
		botright2.x = -70;
		botright2.y = 20;
		botright2.z = 10;
		triangle[tricount-1].topleft.x = topleft2.x;
		triangle[tricount-1].topleft.y = topleft2.y;
		triangle[tricount-1].topleft.z = topleft2.z;
		triangle[tricount-1].topright.x = topright2.x;
		triangle[tricount-1].topright.y = topright2.y;
		triangle[tricount-1].topright.z = topright2.z;
		triangle[tricount-1].botright.x = botright2.x;;
		triangle[tricount-1].botright.y = botright2.y;
		triangle[tricount-1].botright.z = botright2.z;
}

void keyboard (unsigned char key, int x, int y) {
	if(key == 'd')
    {
		yrot += 1;
		if (yrot >360) yrot -= 360;
    }
	 if(key == 'a')
    {
		yrot -= 1;
		if (yrot < -360)yrot += 360;
	}
	if (key=='o') {
		objcount = 0;
		objgravity =  objgrav_init;
		addObject(20);
	}
	if(key=='f') {
		createFood(1);
	    foodgravity	= food_init;
		linger = linger_init;
	}
	if(key=='p')
		createPredator(1);
	if(key=='m') {
		createWalls1();
	}

	if(key=='n') {
		createWalls2();
	}
	if(key=='-') {
		objgravity -= 0.01;
	}
}

void mouseButton(int button, int state, int x, int y) {
	if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {

	}
	if (button == GLUT_RIGHT_BUTTON) {
	}
}

void drawFood() {
	for(int i=0; i<foodcount;  i++) {
	  glPushMatrix();
		glColor4f(food[i].color.r, food[i].color.g, food[i].color.b, food[i].alpha);
		glTranslatef(food[i].position.x, food[i].position.y, food[i].position.z);
		glutWireSphere(OBJECTRADIUS,10,10);
	  glPopMatrix();
	}
}

void drawPredator() {
	for(int i = 0; i<predcount; i++) {
		glPushMatrix();
		glColor3ub( (char) rand()%256, (char) rand()%256, (char) rand()%256);
		glTranslatef(predator[i].position.x, predator[i].position.y, predator[i].position.z);
		glPushMatrix();
		glScalef(1.0, 1.0,2.2);
		glutWireSphere(OBJECTRADIUS,20,20);
		glPopMatrix();
		glTranslatef(0.0, 0.0, 2.0);
		glScalef(1.2, 1.2,1.2);
		glutWireSphere(OBJECTRADIUS,10,10);
		
			// Draw Question Mark
		if(angry > 0 && distance3d(revpoint1.position, predator[i].position) < 5) {
			glPushMatrix();
			glTranslatef(0.0, 6.0, 0.0);
			glutSolidSphere(1.5,20,20);
			glRotatef(0, 0,1,0);
			glScalef(1.5,1.5,1.5);
			glBegin(GL_TRIANGLES);
			//glColor4f(0.2, 0.2, 0.7, 1.0);
			glVertex3f(-3, 15, -1);	
			glVertex3f(3, 15, -1);	
			glVertex3f(0, 2, -1);	
			glEnd();
			glPopMatrix();
		}
		glPushMatrix();
						glPushMatrix();
						glTranslatef (1.5, 0.5, -0.4);
						glRotatef ((GLfloat) rightflap, 0.0, 0.3, 1.0);
						glTranslatef (1.0, 0.0, 0.0);
						glPushMatrix();
							glScalef (4.0, 0.3, 1.5);
							glColor4f(0.8,0.5,0,0.1);
							glutSolidCube (1.5);
						glPopMatrix();
						glPopMatrix();
						glPushMatrix();
						glTranslatef (1.5, 0.5, 0.4);
						glRotatef ((GLfloat) rightflap, 0.0, -0.1, 1.0);
						glTranslatef (1.0, 0.0, 0.0);
						glPushMatrix();
							glScalef (4.0, 0.4, 1.5);
							glColor4f(0.8,0.5,0, 0.1);
							glutSolidCube (1.2);
						glPopMatrix();
						glPopMatrix();


						glPushMatrix();
						glTranslatef (-1.5, 0.5, -0.4);
						glRotatef ((GLfloat) leftflap, 0.0, 0.3, 1.0);
						glTranslatef (-1.0, 0.0, 0.0);
						glPushMatrix();
							glScalef (4.0, 0.3, 1.5);
							glColor4f(0.8,0.5,0,0.1);
							glutSolidCube (1.2);
						glPopMatrix();
						glPopMatrix();
						glPushMatrix();
						glTranslatef (-1.5, 0.5, 0.4);
						glRotatef ((GLfloat) leftflap, 0.0, -0.1, 1.0);
						glTranslatef (-1.0, 0.0, 0.0);
						glPushMatrix();
							glScalef (4.0, 0.4, 1.5);
							glColor4f(0.8,0.5,0, 0.1);
							glutSolidCube (1.2);
						glPopMatrix();
						glPopMatrix();
		glPopMatrix();


	    glPopMatrix();
	}
}

void drawTri(vec3d first, vec3d second, vec3d third) {
	glPushMatrix();
	glBegin(GL_TRIANGLES);
		glColor4f(0.2, 0.2, 0.7, 1.0);
		glVertex3f(first.x, first.y, first.z);	
		glVertex3f(second.x, second.y,second.z);	
		glVertex3f(third.x, third.y,third.z);	
    glEnd();
	glPopMatrix();
}

void drawWalls() {
	for(int i=0; i<tricount; i++) {
	drawTri(triangle[i].topleft, triangle[i].topright, triangle[i].botright);
	}
}

void drawHabitat() {
	for(int i=0; i<habcount; i++) {
	drawTri(habitat[i].topleft, habitat[i].topright, habitat[i].botright);
	}
}


void drawLeader() {
	glPushMatrix();
		glColor3f(0.0f,0.0f,1.0f);
		glTranslatef(leader.position.x, leader.position.y, leader.position.z);
		glutWireSphere(OBJECTRADIUS,20,20);
	glPopMatrix();
}

void flapWingPred() {
	rightflap += 1 % 90;
	if(rightflap==89)
		rightflap=0;


	leftflap -= 1 % 90;
	if(leftflap==-89)
		leftflap=0;

	glutPostRedisplay();
}

void flapWing() {
	for (int i = 1; i < objcount; i++) {

		

		boid[i].rightflap += (boid[i].limbvel) % 50;
		if(boid[i].rightflap==49)
			boid[i].rightflap=0;


		boid[i].leftflap -= (boid[i].limbvel) % 50;
		if(boid[i].leftflap==-49)
			boid[i].leftflap=0;

	}

	glutPostRedisplay();
}

void drawInsect(int i) {

	float body=0.5;

					glPushMatrix();
						glutWireSphere(OBJECTRADIUS,10,10);
						glPushMatrix();
							glTranslatef(0.0, 0.0, 4.0);
							glScalef (0.7, 0.5, 1.5);
							glutWireSphere(OBJECTRADIUS,10,10);
						glPopMatrix();
						glTranslatef(0.0, 0.0, -3.0);
						glutWireSphere(OBJECTRADIUS-0.5,10,10);
							
					glPopMatrix();
					glPushMatrix();
						glTranslatef (1.5, 0.5, -0.4);
						glRotatef ((GLfloat) boid[i].rightflap, 0.0, 0.3, 1.0);
						glTranslatef (1.0, 0.0, 0.0);
						glPushMatrix();
							glScalef (5.0, 0.1, 1.0);
							glColor4f(0.8,0.5,0, 0.1);
							glutSolidCube (1.2);
						glPopMatrix();
						glPopMatrix();
						glPushMatrix();
						glTranslatef (1.5, 0.5, 0.4);
						glRotatef ((GLfloat) boid[i].rightflap, 0.0, -0.1, 1.0);
						glTranslatef (1.0, 0.0, 0.0);
						glPushMatrix();
							glScalef (5.0, 0.1, 0.5);
							glColor4f(0.8,0.5,0, 0.1);
							glutSolidCube (1.2);
						glPopMatrix();
						glPopMatrix();


						glPushMatrix();
						glTranslatef (-1.5, 0.5, -0.4);
						glRotatef ((GLfloat) boid[i].leftflap, 0.0, 0.3, 1.0);
						glTranslatef (-1.0, 0.0, 0.0);
						glPushMatrix();
							glScalef (5.0, 0.1, 1.0);
							glColor4f(0.8,0.5,0, 0.1);
							glutSolidCube (1.2);
						glPopMatrix();
						glPopMatrix();
						glPushMatrix();
						glTranslatef (-1.5, 0.5, 0.4);
						glRotatef ((GLfloat) boid[i].leftflap, 0.0, -0.1, 1.0);
						glTranslatef (-1.0, 0.0, 0.0);
						glPushMatrix();
							glScalef (5.0, 0.1, 0.5);
							glColor4f(0.8,0.5,0, 0.1);
							glutSolidCube (1.2);
						glPopMatrix();
					glPopMatrix();
}

void drawObject() {
	for(int i=0; i < objcount; i++) {
		glPushMatrix();
			glColor3f(boid[i].color.r,boid[i].color.g,boid[i].color.b);
			glTranslatef(boid[i].position.x, boid[i].position.y, boid[i].position.z);
			glRotatef(boid[i].bodyrot, boid[i].axis.x, boid[i].axis.y, boid[i].axis.z);
			if(i==0) 
				glutWireSphere(0.001,10,10);
			else {
				drawInsect(i);
			}
				//drawInsect();
			//glCallList(cube); 
		glPopMatrix();
	}

	////////////////////////////////////////////////////////////////
	// This is just the path that the boids follow when they're 
	// just flying around.
	////////////////////////////////////////////////////////////////
	glPushMatrix();
	glTranslatef(revball.position.x, revball.position.y, revball.position.z);
			glColor4f(0.0f,0.0f,0.0f, 0.1);
			glutSolidSphere(0.1, 10, 10);
	glPopMatrix();
	glPushMatrix();
	glTranslatef(revpoint1.position.x, revpoint1.position.y, revpoint1.position.z);
			glColor4f(0.0f,0.0f,0.0f,0.1);
			glutSolidSphere(0.1, 10, 10);
	glPopMatrix();
}

void eggManagement() {
  ///////////////////////////////////
  // Leader checks if there's food(egg)
  ///////////////////////////////////
  for(int f=0; f<foodcount; f++) {
	   setAccel(food[f].acceleration, food[f].velocity, food[f].mass, 0, 0, 0, windvelocity, 0);
	   eulerInt(food[f].position, food[f].velocity, food[f].acceleration, dt);
	   
	  repelAttractVelocity(food[f].position, 
		                   food[f].mass,
						   boid[0].position,
						   food[f].gravity,
							ATTRACT,
							boid[0].velocity);

	  if (food[f].stolen) {
		  boidVelocityMatch(boid[2].velocity, food[f].velocity, 0.5);
		  boidCentering(boid[2].position, food[f].position, food[f].velocity, 0.05);
		  boidAvoidance(boid[2].position, food[f].position, food[f].velocity, 0.5);
		  //boidAvoidance(food[f].position, food[f+1].position, food[f+1].velocity, 0.5);
	  }
	  /////////////////////////////////////////////////////////
	  // if close enough steal food
	  /////////////////////////////////////////////////////////
	  if(distance3d(food[f].position, boid[0].position) <  0.5) {
		  // how long to linger around food before actually taking it
		  linger -= 0.0005; 
		 
		  if(linger < 0.0) {
			  IsFoodPresent		= false;
			  revballgravity	= revball_init;
			  food[f].stolen	= true;
			  food[f].thief		= 1;
			  food[f].gravity	= 0.0;
			  food[f].love		= 0.0;
			   angry = 0.5;
		  }
	  }
  }
}
void eggManagement2() {
  ///////////////////////////////////
  // Leader checks if there's food(egg)
  ///////////////////////////////////
for(int o=0; o<objcount; o++) {
  for(int f=0; f<foodcount; f++) {
	   setAccel(food[f].acceleration, food[f].velocity, food[f].mass, 0, 0, 0, windvelocity, 0);
	   eulerInt(food[f].position, food[f].velocity, food[f].acceleration, dt);
	   
	  repelAttractVelocity(food[f].position, 
		                   food[f].mass,
						   boid[o].position,
						   food[f].gravity,
							ATTRACT,
							boid[o].velocity);

	  if (food[f].stolen) {
		  boidVelocityMatch(boid[2].velocity, food[f].velocity, 0.5);
		  boidCentering(boid[2].position, food[f].position, food[f].velocity, 0.05);
		  boidAvoidance(boid[2].position, food[f].position, food[f].velocity, 0.5);
		  //boidAvoidance(food[f].position, food[f+1].position, food[f+1].velocity, 0.5);
	  }
	  /////////////////////////////////////////////////////////
	  // if close enough steal food
	  /////////////////////////////////////////////////////////
	  if(distance3d(food[f].position, boid[o].position) <  0.5) {
		  // how long to linger around food before actually taking it
		  linger -= 0.0005; 
		  if(linger < 0.0) {
			  IsFoodPresent		= false;
			  revballgravity	= revball_init;
			  food[f].stolen	= true;
			  food[f].thief		= 1;
			  food[f].gravity	= 0.0;
			  food[f].love		= 0.0;
		  }
	  }
  }
}
}
////////////////////////////////////////////////////////
// How to handle prey and predator 
////////////////////////////////////////////////////////
void handlePredatoryBehavior(){
	for(int pr=0; pr<predcount; pr++) {
	  //////////////////////////////////////////////////
	  // Predator is of course attracted to its own egg
	  // but not for eating. AND only if the egg has 
	  // not been stolen yet
	  /////////////////////////////////////////////////
	  for(int f=0; f<foodcount; f++) {
		  
		  repelAttractVelocity(food[f].position, 
		                       food[f].mass,
							   predator[pr].position,
							   food[f].love,
							   ATTRACT,
							   predator[pr].velocity);
	  }

	  for(int ob=0; ob<objcount; ob++) { 

			  ////////////////////////////////////////////////////////////
			  // compute attract velocity of predator toward leader ONLY
			  // and only if at a certain distance to its egg
			  //////////////////////////////////////////////////////////////
			  for(int f=0; f<foodcount; f++) {
				  if( (distance3d(food[f].position, boid[0].position) < 35) 
					  && !(food[f].stolen) ){
						 repelAttractVelocity(boid[0].position, 
											  boid[0].mass, 
											  predator[pr].position, 
											  objgravity, 
											  ATTRACT,
											  predator[pr].velocity);
				  }
			  }
		  
			////////////////////////////////////////////////////////////////
			// Check how close boids are to predator before they react.
			////////////////////////////////////////////////////////////////
			if(distance3d(predator[pr].position, boid[ob].position) < 45.0) {
					  // At the same time, compute repel velocity of prey
					 // during predator presence
				if(ob==0) 
					repelAttractVelocity(predator[pr].position, 
										predator[pr].mass, 
										boid[ob].position, 
										(predgravity+5), 
										REPEL,
										boid[ob].velocity);
				else
					 repelAttractVelocity(predator[pr].position, 
										predator[pr].mass, 
										boid[ob].position, 
										predgravity, 
										REPEL,
										boid[ob].velocity);
			}

			///////////////////////////////////////////////////////////
			// This makes boids loose their cohesiveness when
			// attacker is really close. Imitates real situation better
			///////////////////////////////////////////////////////////
			if(distance3d(predator[pr].position, boid[ob].position) < 30.0) {
				  kcentering = 0.0;
				  kmatching = 0.0;
			}
			  else  {
				  kcentering = kcenter_init;
				  kmatching = kmatch_init;
			}
	   }

  }
}
void handlePredatoryBehavior2(){
	for(int pr=0; pr<predcount; pr++) {
	  //////////////////////////////////////////////////
	  // Predator is of course attracted to its own egg
	  // but not for eating. AND only if the egg has 
	  // not been stolen yet
	  /////////////////////////////////////////////////
	  for(int f=0; f<foodcount; f++) {
		  
		  repelAttractVelocity(food[f].position, 
		                       food[f].mass,
							   predator[pr].position,
							   food[f].love,
							   ATTRACT,
							   predator[pr].velocity);
	  }

	  for(int ob=0; ob<objcount; ob++) { 

			  ////////////////////////////////////////////////////////////
			  // compute attract velocity of predator toward leader ONLY
			  // and only if at a certain distance to its egg
			  //////////////////////////////////////////////////////////////
			  for(int f=0; f<foodcount; f++) {
				  if( (distance3d(food[f].position, boid[ob].position) < 35) 
					  && !(food[f].stolen) ){
						 repelAttractVelocity(boid[ob].position, 
											  boid[ob].mass, 
											  predator[pr].position, 
											  objgravity, 
											  ATTRACT,
											  predator[pr].velocity);
				  }
			  }
		  
			////////////////////////////////////////////////////////////////
			// Check how close boids are to predator before they react.
			////////////////////////////////////////////////////////////////
			if(distance3d(predator[pr].position, boid[ob].position) < 45.0) {
					  // At the same time, compute repel velocity of prey
					 // during predator presence
				if(ob==0) 
					repelAttractVelocity(predator[pr].position, 
										predator[pr].mass, 
										boid[ob].position, 
										predgravity+5, 
										REPEL,
										boid[ob].velocity);
				else
					 repelAttractVelocity(predator[pr].position, 
										predator[pr].mass, 
										boid[ob].position, 
										predgravity, 
										REPEL,
										boid[ob].velocity);
			}

			///////////////////////////////////////////////////////////
			// This makes boids loose their cohesiveness when
			// attacker is really close. Imitates real situation better
			///////////////////////////////////////////////////////////
			if(distance3d(predator[pr].position, boid[ob].position) < 25.0) {
				  kcentering = 0.0;
				  kmatching = 0.0;
			}
			  else  {
				  kcentering = kcenter_init;
				  kmatching = kmatch_init;
			}
	   }

  }
}

void predatorPolygonCollision() {
	for (int i=0; i<predcount; i++) {
		// Save old position for use in collision detection later
		vec3d pStart = predator[i].position;

		//vec3d Pos, Vel;
		setAccel(predator[i].acceleration, predator[i].velocity, predator[i].mass, xgravity, -ygravity, zgravity, windvelocity, k);
		eulerInt(predator[i].position, predator[i].velocity, predator[i].acceleration, dt);
	
		// Save new position for use in collision detection later
		vec3d pDest = predator[i].position;
	
		// Collision with own habitat
		for(int p=0; p<habcount; p++) {
			detectAndReflect(habitat[p], pStart, pDest, predator[i].velocity, e);
		}

		// Collision with other polygons
		for(int p=0; p<tricount; p++) {
			detectAndReflect(triangle[p], pStart, pDest, predator[i].velocity, e);
			
			// Repellance with other polygons
			if(distance3d(triangle[p].topleft, predator[i].position) < 35.0) {
				repelAttractVelocity(triangle[p].topleft, 
									raMass, 
									predator[i].position, 
									raForce, 
									REPEL,
									predator[i].velocity);
				repelAttractVelocity(triangle[p].topright, 
									raMass, 
									predator[i].position, 
									raForce, 
									REPEL,
									predator[i].velocity);
			}
		}
	}
}
void handleObstacles() {
	for (int i = 0; i < objcount; i++) {
		// Save old position
		vec3d pStart	= boid[i].position;
		//vec3d pStartVel = boid[i].velocity;

		// Compute total acceleration and then integrate
		setAcceleration(boid[i], xgravity, ygravity, zgravity, windvelocity, k);
		eulerIntegrate(boid[i], dt);

		// Save new position for use in collision detection later
		vec3d pDest		= boid[i].position;
		//vec3d pDestVel	= boid[i].velocity;
		//vec3d pDestAcc  = boid[i].acceleration;

		// Collision with predator's habitat
		for(int pp=0; pp<habcount; pp++) {
			 detectAndReflect(habitat[pp], pStart, pDest, boid[i].velocity, e); 
		}

		buildRepelWalls(num_repelvert, repelvert, boid[i].position, boid[i].velocity, raMass, raForce, REPEL); 
		
		//////////////////////////////////////////////////////
		// Check if there are obstacles like a polygon along the way
		// ///////////////////////////////////////////////////
		for(int p=0; p<tricount; p++) {
			 // Check how far boids are to obstacle before reacting.
			 // If I don't have this, boids react too prematurely.
			 if(distance3d(triangle[p].topleft, boid[i].position) < 45.0) {
				 repelAttractVelocity(triangle[p].topleft, raMass, boid[i].position, 
												raForce, 
												REPEL,
												boid[i].velocity);
				 repelAttractVelocity(triangle[p].topright, 
												raMass, boid[i].position, 
												raForce, 
												REPEL,
												boid[i].velocity);
			 }
			// Collision with other polygons
			 detectAndReflect(triangle[p], pStart, pDest, boid[i].velocity, e);
		}


  } // end for loop
}

void rotateToLeader() {
	for (int i = 0; i < objcount; i++) {

		vec3d leadvel  = boid[i].acceleration;
		vec3d leadvelN = normalize3Dvector(leadvel);
		vec3d boidvel  = boid[i+1].velocity;
		vec3d boidvelN = normalize3Dvector(boidvel);

		vec3d rotationaxis = cross3Dvector(boidvel, leadvel);
		float angle = dot3Dvector(boidvelN, leadvelN);


		boid[i].bodyrot = angle;
		boid[i].axis = rotationaxis;

		float z = 1;
	}
}


void handleFlockingPhysics() {
	// Set all boids to match velocity of each other (or leader maybe)
	for (int i = 1; i < objcount; i++)
		boidVelocityMatch(boid[i].velocity, boid[i+1].velocity, kmatching);

	// Centering - center to leader
	for (int m = 1; m<objcount; m++)
		boidCentering(boid[0].position, boid[m].position, boid[m].velocity, kcentering);
		
	// Avoidance - avoid each other not just leader
	for (int a=0; a<objcount; a++)
		boidAvoidance(boid[a].position, boid[a+1].position, boid[a+1].velocity, kavoid);

	boidVelocityMatch(boid[0].velocity, boid[1].velocity, 0.0001);
	boidCentering(boid[0].position, boid[1].position, boid[1].velocity, 0.001);
}

void displace() {
	if(angry > 0)
		angry -= 0.0001;
	flapWing();
	flapWingPred();
	handlePaths();
	handleObstacles();
	predatorPolygonCollision();
	handleFlockingPhysics();
	eggManagement();
	handlePredatoryBehavior();
	rotateToLeader();
	glutPostRedisplay();
}

void drawArm1() {
   
}

void saveToDisk() {
	fileIncr++;				// number to be converted to a string       

	string number2add;       // string which will contain the result
	ostringstream convert;   // stream used for the conversion
	convert << fileIncr;      // insert the textual representation of 'Number' in the characters in the stream

	number2add = convert.str(); // set 'Result' to the contents of the stream
	int length = number2add.length();
	if(length < 5) { // add trailing zeroes
		int diff = 5 - length;
		std::string dest = std::string( diff, '0').append( number2add);
		number2add  = dest;
	}
	
	string myfile = "flock";
	myfile.append(number2add);
	myfile.append(".bmp");
	char* newfilenme = (char*)myfile.c_str();

	snapshot(newfilenme);
}

void display (void) {
	glClearDepth (1);
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();  
	gluLookAt(120.0, 20.0, 140.0,		
              0.0, 0.0, 0.0,		
              0.0, 5.0, 0.0);		
	float pos[]={10.0,90.0,-110.0,1.0};        //set the position
	glLightfv(GL_LIGHT0,GL_POSITION,pos);
	glEnable(GL_COLOR_MATERIAL);
	
	camera();
	glPushMatrix();
	
	glTranslatef(0, -30, 0);
	//glScalef(10,10,10);
	//glCallList(cube);
	glPopMatrix();
	drawGrid();
	//createHabitat();
	drawWalls();
	drawHabitat();
	drawFood();
	drawObject();
	drawPredator();

	//saveToDisk();
    glutSwapBuffers();
}

void init (void) {
	// If Depth Test is not enabled, I would see my object infront of 
	// my polygon even if my object falls behind polygon. 
	glClearDepth(1);
	glEnable(GL_DEPTH_TEST);
	glEnable (GL_BLEND);
	//glBlendFunc (GL_SRC_ALPHA, GL_ONE);
	glClearColor (0.0, 0.0, 0.0, 0.0);

	
	windvelocity.x = 0.5;
	windvelocity.y = 0.5;
	windvelocity.z = 0.5;

	buildWalls(habcount, num_tri_hab, habitat, habvert);
	createPredator(1);
	addObject(20);
	//buildWalls(tricount, num_triangles, triangle, vert);
	//cube=loadObject("birds.obj"); 
	cube=loadObject("Fiat509_2.6.obj");
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glShadeModel (GL_SMOOTH);
    glEnable (GL_LIGHTING);
    glEnable (GL_LIGHT0);
	float col[]={1.0,1.0,1.0,1.0};  //light color is white
    glLightfv(GL_LIGHT0,GL_DIFFUSE,col);
	texture[0] = LoadTextureRAW( "wood.bmp",500,334);
	
 }

void reshape (int w, int h) {
    glViewport (0, 0, (GLsizei)w, (GLsizei)h);
    glMatrixMode (GL_PROJECTION);
    glLoadIdentity ();
    gluPerspective (60, (GLfloat)w / (GLfloat)h, 1.0, 400.0);
    glMatrixMode (GL_MODELVIEW);
}

int main (int argc, char **argv) {

    glutInit (&argc, argv);
    glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    glutInitWindowSize (1000, 650);
    glutInitWindowPosition (100, 50);
    glutCreateWindow ("A basic OpenGL Window");

    init();
    glutDisplayFunc (display);
    glutIdleFunc (displace);
    glutReshapeFunc (reshape);
	glutMouseFunc(mouseButton);
	glutKeyboardFunc(keyboard);

    glutMainLoop ();
    return 0;
}
