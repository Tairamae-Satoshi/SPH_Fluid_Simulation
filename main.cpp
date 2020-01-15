#pragma once
#include <cstdio>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

#include "gluvi.h"
#include "gl/glu.h"
#include"sph_system_solver3.h"

using namespace std;

string frame_number = "Frame 0";
Frame frame;
//Sim parameters
SphSystemSolver3 solver;
auto particles = solver.sphData();

bool filming = false;
bool running = false;

char* sgifileformat;

std::vector<Vector3D> particles_positions;
std::vector<Vector3D> particles_velocities;


void advance_sim() {

	solver.update(frame);

	//Update the frame label
	frame.advance();
	std::ostringstream strout;
	strout.str("");
	strout << "Frame " << frame.index;
	frame_number = strout.str();
}

void set_view(Gluvi::Target3D& cam)
{
	//setup initial view point
	cam.target[0] = 0.5;
	cam.target[1] = 0.5;
	cam.target[2] = 0.5;
	cam.heading = 0.5;
	cam.pitch = -0.5;
	cam.dist = 3;
}

void set_lights_and_material(int object)
{
	glEnable(GL_LIGHTING);
	GLfloat global_ambient[4] = { 0.1f, 0.1f, 0.1f, 1.0f };
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, global_ambient);
	glShadeModel(GL_SMOOTH);

	//Light #1
	GLfloat color[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
	GLfloat position[3] = { 1.0f, 1.0f, 1.0f };
	glLightfv(GL_LIGHT0, GL_SPECULAR, color);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, color);
	glLightfv(GL_LIGHT0, GL_POSITION, position);

	//Light #2
	GLfloat color2[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
	GLfloat position2[3] = { -1.0f, -1.0f, 1.0f };
	glLightfv(GL_LIGHT1, GL_SPECULAR, color2);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, color2);
	glLightfv(GL_LIGHT1, GL_POSITION, position2);

	GLfloat obj_color[4] = { .2, .3, .7 };
	glMaterialfv(GL_FRONT, GL_AMBIENT, obj_color);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, obj_color);

	GLfloat specular[4] = { .4, .2, .8 };
	glMaterialf(GL_FRONT, GL_SHININESS, 32);
	glMaterialfv(GL_FRONT, GL_SPECULAR, specular);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);

}

void timer(int value)
{
	if (filming) {
		Gluvi::sgi_screenshot(sgifileformat, frame);
	}

	if (filming || running) {
		advance_sim();
		glutTimerFunc(10, timer, 0);
		glutPostRedisplay();
	}

}


void display(void)
{
	glClearColor(0.6f, 0.7f, 0.9f, 1);

	glEnable(GL_LIGHTING);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	set_lights_and_material(1);

	//Draw the particles as simple spheres.
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINES);
	GLUquadric* particle_sphere;
	particle_sphere = gluNewQuadric();
	gluQuadricDrawStyle(particle_sphere, GLU_FILL);
	auto particles = solver.sphData();
	particles_positions = particles->positions();
	double particle_radius = particles->radius();
	for (unsigned int p = 0; p < particles_positions.size(); ++p) {
		glPushMatrix();
		Vector3F pos = particles_positions[p].v;
		glTranslatef(pos[0], pos[1], pos[2]);
		gluSphere(particle_sphere, particle_radius, 20, 20);
		glPopMatrix();
	}

	//Draw bounding box wireframe
	glDisable(GL_LIGHTING);
	glColor3f(0, 0, 0);
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 0, 1);

	glVertex3f(0, 0, 0);
	glVertex3f(0, 1, 0);

	glVertex3f(0, 0, 0);
	glVertex3f(1, 0, 0);

	glVertex3f(0, 1, 0);
	glVertex3f(1, 1, 0);

	glVertex3f(1, 1, 0);
	glVertex3f(1, 0, 0);

	glVertex3f(1, 0, 0);
	glVertex3f(1, 0, 1);

	glVertex3f(0, 1, 0);
	glVertex3f(0, 1, 1);

	glVertex3f(1, 1, 0);
	glVertex3f(1, 1, 1);

	glVertex3f(0, 1, 1);
	glVertex3f(1, 1, 1);

	glVertex3f(1, 0, 1);
	glVertex3f(1, 1, 1);

	glVertex3f(0, 0, 1);
	glVertex3f(1, 0, 1);

	glVertex3f(0, 0, 1);
	glVertex3f(0, 1, 1);

	glEnd();

}


void toggle_filming() {
	if (!running) {
		if (!filming) {
			filming = true;
			glutTimerFunc(10, timer, 0);
		}
		else {
			filming = false;
		}
	}
}

void toggle_running() {
	if (!filming) {
		if (!running) {
			running = true;
			glutTimerFunc(10, timer, 0);
		}
		else {
			running = false;
		}
	}
}

struct MovieButton : public Gluvi::Button {
	const char* filename_format;
	MovieButton(const char* label, const char* filename_format_) : Gluvi::Button(label), filename_format(filename_format_) {}
	void action()
	{
		toggle_filming();
	}
};

struct RunButton : public Gluvi::Button {
	RunButton(const char* label) : Gluvi::Button(label) {}
	void action()
	{
		toggle_running();
	}
};

void keyPress(unsigned char key, int x, int y) {

	if (key == 'r') {
		toggle_running();
	}
	else if (key == 'f') {
		toggle_filming();
	}
	glutPostRedisplay();


}



Gluvi::Target3D* cam_local;

int main(int argc, char** argv)
{

	Gluvi::init("Particle Animation", &argc, argv);

	glutKeyboardFunc(keyPress);

	Gluvi::Target3D cam;
	set_view(cam);
	Gluvi::camera = &cam;

	Gluvi::userDisplayFunc = display;

	Gluvi::StaticText frametext(frame_number.c_str());
	Gluvi::root.list.push_back(&frametext);

	sgifileformat = new char[255];
	sprintf(sgifileformat, "screenshot%%04d.sgi");
	printf("%s\n", sgifileformat);

	MovieButton movie("(f)ilm", sgifileformat);
	Gluvi::root.list.push_back(&movie);

	RunButton run("(r)un");
	Gluvi::root.list.push_back(&run);

	Gluvi::StaticText instructiontext("Shift + Drag with Mouse Buttons to adjust view.");
	Gluvi::root.list.push_back(&instructiontext);

	Gluvi::run();
	return 0;
}
