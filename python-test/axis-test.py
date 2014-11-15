#!/usr/bin/env python
import re
import time
import Image
import serial
import numpy
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import sys

ESCAPE = '\033'
onex = 0;
oney = 0;
onez = 0;
two = 0;


# Number of the glut window.
window = 0


#img = Image.open('images.jpeg') # .jpg, .bmp, etc. also work
#img_data = numpy.array(list(img.getdata()), numpy.int8)
#texture = glGenTextures(1)
#glPixelStorei(GL_UNPACK_ALIGNMENT,1)
#glBindTexture(GL_TEXTURE_2D, texture)
#glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP)
#glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP)
#glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
#glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
#glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, img.size[0], img.size[1], 0, GL_RGB, GL_UNSIGNED_BYTE, img_data)


# A general OpenGL initialization function.  Sets all of the initial parameters. 
def InitGL(Width, Height):				# We call this right after our OpenGL window is created.
    glClearColor(0.0, 0.0, 0.0, 0.0)	# This Will Clear The Background Color To Black
    glClearDepth(1.0)					# Enables Clearing Of The Depth Buffer
    glDepthFunc(GL_LESS)				# The Type Of Depth Test To Do
    glEnable(GL_DEPTH_TEST)				# Enables Depth Testing
    glShadeModel(GL_SMOOTH)				# Enables Smooth Color Shading
	
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()					# Reset The Projection Matrix
										# Calculate The Aspect Ratio Of The Window
    gluPerspective(45.0, float(Width)/float(Height), 0.1, 1000.0)

    glMatrixMode(GL_MODELVIEW)

# The main drawing function. 
def DrawGLScene():
	global onex;
	global oney;
	global onez;
	global two;
	
	serial_poll()
	print "Drqwing"
	
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);	# Clear The Screen And The Depth Buffer
	glLoadIdentity();					# Reset The View
	glTranslatef(-1.5,0.0,-6.0)				# Move Left And Into The Screen
	
	#gluLookAt( 0, 0, 0, 0, 0, 0, 0, 0, 0 )
	glRotatef(onex,1.0,0.0,0.0)
	glRotatef(oney,0.0,1.0,0.0)
	glRotatef(onez,0.0,0.0,1.0)
	
	glColor3f( 1, 0, 0 )
	glutWireTeapot(1)
	
	glLoadIdentity();					# Reset The View
	glTranslatef(1.5,0.0,-6.0)				# Move Left And Into The Screen
	glRotatef(two,0.0,1.0,0.0)
	glColor3f( 0, 1, 0 )
	glutWireTeapot(1)
	
	
	
	#  since this is double buffered, swap the buffers to display what just got drawn. 
	glutSwapBuffers()
	
	

# The function called whenever a key is pressed. Note the use of Python tuples to pass in: (key, x, y)  
def keyPressed(*args):
	# If escape is pressed, kill everything.
    if args[0] == ESCAPE:
	    glutDestroyWindow(window)
	    sys.exit()
def serialInit():
	global ser
	ser = serial.Serial('/dev/ttyACM0', 115200)
	
	
def serial_poll():
	global ser
	global onex
	global oney
	global onez
	global two
	

	line = ser.readline()
	#print line
	if line.startswith( 'ypr' ):
		val =  re.findall(r"[-+]?\d*\.\d+|\d+", line)
		if len(val) > 2:
			onez = float(val[2])
			onex = float(val[1])
			oney = float(val[0])
			#two is y axis
			two  = float(val[3]) 
			two = 0
			oney = two
		
		#DrawGLScene()
	
	else:
		print line
	#time.sleep(1)
	#print "sll"
		
	
	
def main():
	global window

	glutInit(sys.argv)
	serialInit();

	# Select type of Display mode:   
	#  Double buffer 
	#  RGBA color
	# Alpha components supported 
	# Depth buffer
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH)
	
	# get a 640 x 480 window 
	glutInitWindowSize(800, 600)
	
	# the window starts at the upper left corner of the screen 
	glutInitWindowPosition(0, 0)
	
	# Okay, like the C version we retain the window id to use when closing, but for those of you new
	# to Python (like myself), remember this assignment would make the variable local and not global
	# if it weren't for the global declaration at the start of main.
	window = glutCreateWindow("Accel test")

   	# Register the drawing function with glut, BUT in Python land, at least using PyOpenGL, we need to
	# set the function pointer and invoke a function to actually register the callback, otherwise it
	# would be very much like the C version of the code.	
	#glutDisplayFunc(serial_poll)
	#glutDisplayFunc()
	
	# Uncomment this line to get full screen.
	# glutFullScreen()

	# When we are doing nothing, redraw the scene.
	glutIdleFunc(DrawGLScene)
	
	#
		
	# Register the function called when our window is resized.
	#glutReshapeFunc(ReSizeGLScene)
	
	
	# Register the function called when the keyboard is pressed.  
	glutKeyboardFunc(keyPressed)
	
	# Initialize our window. 
	InitGL(800, 600)
	#serial_poll();

	
# Print message to console, and kick off the main to get it rolling.
print "Hit ESC key to quit."

if __name__ == '__main__':
	try:
		GLU_VERSION_1_2
	except:
		print "Need GLU 1.2 to run this demo"
		sys.exit(1)
	main()
	glutMainLoop()
