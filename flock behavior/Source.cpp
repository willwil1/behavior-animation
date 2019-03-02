// standard
#include <assert.h>
#include <math.h>
#include <iostream>
#include <time.h>

// glut
#include <GL/glut.h>

//flock
#include "flock.h"
flock individual[20];
int flock_count = 20;
float center[3]= { 0.0,0.0,-50.0 };

//obstacles
#include"obstacles.h"
obstacles O[10];
int o_count = 10;

//================================
// global variables
//================================
// screen size
int g_screenWidth = 0;
int g_screenHeight = 0;

// frame index
int g_frameIndex = 0;


//================================
// init
//================================
void init(void) {
	//initial force
	int m, n;
	int vec = 3;
	float **initialF;
	initialF = (float**)malloc(flock_count * sizeof(int**));
	for (m = 0; m < flock_count; m++) {
		initialF[m] = (float*)malloc(vec * sizeof(int*));
	}
	//initial position
	float **initialP;
	initialP = (float**)malloc(flock_count * sizeof(int**));
	for (m = 0; m < flock_count; m++) {
		initialP[m] = (float*)malloc(vec * sizeof(int*));
	}
	//initial obstacles position
	float **initialO;
	initialO = (float**)malloc(o_count * sizeof(int**));
	for (m = 0; m < o_count; m++) {
		initialO[m] = (float*)malloc(vec * sizeof(int*));
	}

	srand((unsigned)time(NULL));
	for (m = 0; m < flock_count; m++) {
		for (n = 0; n < 3; n++) {
			initialF[m][n] = rand() % 10+1-5 ;
		}
		initialP[m][0] = rand() % 20+1- 10 ;
		initialP[m][1] = rand() %  20+ 1 - 10;
		initialP[m][2] = rand() % 10 + 1 - 55;
	}
	srand((unsigned)time(NULL));
	for (m = 0; m < o_count; m++) {
		initialO[m][0] = rand() % 50 + 1 - 25;
		initialO[m][1] = rand() % 50 + 1 - 25;
	}


	//	float initialF[2][3] = { {60.0,0.0,0.0},{-60.0,0.0,0.0} };
	//	float initialP[2][3] = { {-10.0,10.0,-30.0},{10.0,10.0,-30.0}};
	int i = 0;
	for (i = 0; i < flock_count; i++) {
		individual[i].setMass(1.0);
		individual[i].setRadius(1.0);
		individual[i].setInitialF(initialF[i]);
		individual[i].setInitialP(initialP[i]);

	}
	for (i = 0; i < o_count; i++) {
		O[i].set_init(initialO[i][0], initialO[i][1], -50.0, 20.0);
	}


	for (i = 0; i < flock_count; i++) {
		free(initialF[i]);
		free(initialP[i]);
	}
	free(initialF);
	free(initialP);
	for (i = 0; i < o_count; i++) {
		free(initialO[i]);
	}
	free(initialO);
}

//================================
// update
//================================
void update(void) {
	// update boids states
	
		int i, j;
		for (i = 0; i < flock_count; i++) {
			for (j = 0; j < flock_count; j++) {
				if (j != i) {
					individual[i].cal_perception(individual[j]);
					individual[i].velocity_matching(individual[i]);
					individual[i].collision_avoidance(individual[j]);
					individual[i].centering();
					individual[i].local_centering(individual[i]);
					individual[i].spiral();
				}
			}
		}
		for (i = 0; i < flock_count; i++) {
			for (j = 0; j < o_count; j++) {
				individual[i].collision_obstacles(O[j]);
			}
		}
		for (i = 0; i < flock_count; i++) {
			individual[i].updateState(individual[i].local_count);
	}

}


//================================
// render
//================================
void render(void) {
	// clear buffer
	glClearColor(0.0, 0.0, 0.0, 0.0);
	glClearDepth(1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// render state
	glEnable(GL_DEPTH_TEST);
	glShadeModel(GL_SMOOTH);


	// enable lighting
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	// light source attributes
	GLfloat LightAmbient[] = { 0.4f, 0.4f, 0.4f, 1.0f };
	GLfloat LightDiffuse[] = { 0.3f, 0.3f, 0.3f, 1.0f };
	GLfloat LightSpecular[] = { 0.4f, 0.4f, 0.4f, 1.0f };
	GLfloat LightPosition[] = { 5.0f, 5.0f, 5.0f, 1.0f };

	glLightfv(GL_LIGHT0, GL_AMBIENT, LightAmbient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, LightDiffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, LightSpecular);
	glLightfv(GL_LIGHT0, GL_POSITION, LightPosition);

	float material_Pa[] = { 0.75f, 0.70725f, 0.70725f, 0.922f };
	float material_Pd[] = { 1.0f, 0.829f, 0.829f, 0.922f };
	float material_Ps[] = { 0.296648f, 0.296648f, 0.296648f, 0.922f };
	float material_PS = 11.264f;

	glMaterialfv(GL_FRONT, GL_AMBIENT, material_Pa);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, material_Pd);
	glMaterialfv(GL_FRONT, GL_SPECULAR, material_Ps);
	glMaterialf(GL_FRONT, GL_SHININESS, material_PS);

//draw control point

	glPushMatrix();
	glTranslatef(center[0],center[1],center[2]);
	glutSolidSphere(1.0, 60, 60);
	glPopMatrix();
	//draw obstacles
	int i;
	for (i = 0; i < o_count; i++) {
		glPushMatrix();
		glTranslatef(O[i].position[0], O[i].position[1], O[i].position[2]);
		glScalef(1, 1, O[i].scale);
		glutSolidCube(1.0);
		glPopMatrix();
	}




	// surface material attributes
	GLfloat material_Ka[] = { 0.51f, 0.56f, 0.11f, 1.0f };
	GLfloat material_Kd[] = { 0.43f, 0.47f, 0.54f, 1.0f };
	GLfloat material_Ks[] = { 0.33f, 0.33f, 0.52f, 1.0f };
	GLfloat material_Ke[] = { 0.1f , 0.0f , 0.1f , 1.0f };
	GLfloat material_Se = 10;

	glMaterialfv(GL_FRONT, GL_AMBIENT, material_Ka);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, material_Kd);
	glMaterialfv(GL_FRONT, GL_SPECULAR, material_Ks);
	glMaterialfv(GL_FRONT, GL_EMISSION, material_Ke);
	glMaterialf(GL_FRONT, GL_SHININESS, material_Se);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	for (i = 0; i < flock_count; i++) {

		glPushMatrix();
		glTranslatef(individual[i].position[0], individual[i].position[1], individual[i].position[2]);
		glutSolidSphere(1.0, 60, 60);
		glPopMatrix();
	}

	// disable lighting
	glDisable(GL_LIGHT0);
	glDisable(GL_LIGHTING);

	// swap back and front buffers
	glutSwapBuffers();
}

//================================
// keyboard input
//================================
//add control to the control point
void specialkey(int key, int x, int y) {
	int i;
	switch (key) {
	case GLUT_KEY_LEFT:
		center[0] -= 0.4;
		for (i = 0; i < flock_count; i++) {
			individual[i].setcenter(center[0],center[1],center[2]);
		}
		break;
	case GLUT_KEY_RIGHT:
		center[0] += 0.4;
		for (i = 0; i < flock_count; i++) {
			individual[i].setcenter(center[0], center[1], center[2]);
		}
		break;
	case GLUT_KEY_UP:
		center[1] += 0.4;
		for (i = 0; i < flock_count; i++) {
			individual[i].setcenter(center[0], center[1], center[2]);
		}
		break;
	case GLUT_KEY_DOWN:
		center[1] -= 0.4;
		for (i = 0; i < flock_count; i++) {
			individual[i].setcenter(center[0], center[1], center[2]);
		}
		break;
		
	}
}

//================================
// reshape : update viewport and projection matrix when the window is resized
//================================
void reshape(int w, int h) {
	// screen size
	g_screenWidth = w;
	g_screenHeight = h;

	// viewport
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);

	// projection matrix
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluPerspective(80.0, (GLfloat)w / (GLfloat)h, 1.0, 10000.0);
}


//================================
// timer : triggered every 16ms ( about 60 frames per second )
//================================
void timer(int value) {
	// increase frame index
	g_frameIndex++;

	update();

	// render
	glutPostRedisplay();

	// reset timer
	// 16 ms per frame ( about 60 frames per second )
	glutTimerFunc(16, timer, 0);

}

//================================
// main
//================================
int main(int argc, char** argv) {
	// create opengL window
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(600, 600);
	glutInitWindowPosition(100, 100);
	glutCreateWindow(argv[0]);

	// init
	init();

	// set callback functions
	glutDisplayFunc(render);
	glutReshapeFunc(reshape);
	glutSpecialFunc(specialkey);
	glutTimerFunc(16, timer, 0);

	// main loop
	glutMainLoop();


	return 0;
}