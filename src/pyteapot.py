#AHRS Python Desktop App

import pygame
import math
from OpenGL.GL import *
from OpenGL.GLU import *
from pygame.locals import *

import datetime
import struct

import serial
ser = serial.Serial('COM3', 115200) #by default: bytesize=EIGHTBITS, parity=PARITY_NONE, stopbits=STOPBITS_ONE

def main():
    video_flags = OPENGL | DOUBLEBUF
    (numpass, numfail) = pygame.init() #initialize all imported pygame modules. The total number if successful and failed inits will be returned as a tuple
    #Setting the display mode in pygame creates a visible image surface on the monitor. The display surface is nothing more than a standard pygame surface 
    #object. You let pygame choose it's bit depth by calling set_mode() with no depth argument or a depth of 0, or you can call mode_ok() to find a closest
    #matching bit depth to what you need.
    screen = pygame.display.set_mode((640, 480), video_flags)
    pygame.display.set_caption("AHRS (BNO055)") #Set the current window caption
    resizewin(640, 480)
    init()
    frames = 0
    ticks = pygame.time.get_ticks()

    while 1:
        event = pygame.event.poll() #get a single event from the queue
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            break

        [w, nx, ny, nz, gravity_x, gravity_y, gravity_z, sys_cal, accel_cal, gyro_cal, mag_cal] = read_data()
        
        draw(w, nx, ny, nz, gravity_x, gravity_y, gravity_z, sys_cal, accel_cal, gyro_cal, mag_cal)        

        pygame.display.flip()
        frames += 1
    print("fps: %d" % ((frames*1000)/(pygame.time.get_ticks()-ticks)))
    if (useSerial):
        ser.close()


def resizewin(width, height):
    """
    For resizing window
    """
    if height == 0:
        height = 1
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, 1.0*width/height, 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()


def init():
    glShadeModel(GL_SMOOTH)
    glClearColor(0.0, 0.0, 0.0, 0.0)
    glClearDepth(1.0)
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LEQUAL)
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)


def read_data():
    ser.reset_input_buffer() #Flush input buffer, discarding all it’s contents.
        
    while not(ser.read() == b'B' and ser.read() == b'F'):
        pass        
                
    while ser.in_waiting < 32:
        pass
        
    payload = ser.read(32)    
    
    #quaternion 
    quat = struct.unpack('4f', payload[0:16])
    w = quat[0]
    nx = quat[1]
    ny = quat[2]
    nz = quat[3]  
    
    #gravity vector - m/s^2
    gravity = struct.unpack('3f', payload[16:28])
    gravity_x = gravity[0]
    gravity_y = gravity[1]
    gravity_z = gravity[2]   
    
    #calibration status 
    sys_cal = payload[28]
    accel_cal = payload[29]
    gyro_cal = payload[30]
    mag_cal = payload[31]
    
    print("{:10.6f}, {:10.6f}, {:10.6f}, {:10.6f}, {:10.6f}, {:10.6f}, {:10.6f}, {:4d}, {:4d}, {:4d}, {:4d}".format(w, nx, ny, nz, gravity_x, gravity_y, gravity_z, 
                                                                                                                    sys_cal, accel_cal, gyro_cal, mag_cal)) 

    return [w, nx, ny, nz, gravity_x, gravity_y, gravity_z, sys_cal, accel_cal, gyro_cal, mag_cal] 


def draw(w, nx, ny, nz, gravity_x, gravity_y, gravity_z, sys_cal, accel_cal, gyro_cal, mag_cal):
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()
    glTranslatef(0, 0.0, -7.0)

    #Header and footer
    #drawText((-2.6, 1.8, 2), "Sensor: BNO055 | Sensor controller: based on Arduino", 16)    
    drawText((-2.6, -2, 2), "Press Escape to exit.", 14)

    #Sensor calibration status
    drawText((-2.6, 1.8, 2), "Sensor Calibration Status", 16)
    drawText((-2.6, 1.6, 2), "Syst: %d, Accel: %d, Gyro: %d, Mag: %d" %(sys_cal, accel_cal, gyro_cal, mag_cal), 16)

    #Sensor data  
    drawText((-2.6, -1.2, 2), "qW: %f, qX: %f, qY: %f, qZ: %f" %(w, nx, ny, nz), 16)
    drawText((-2.6, -1.4, 2), "gx: %f, gy: %f, gz: %f" %(gravity_x, gravity_y, gravity_z), 16)
        
    glRotatef(2 * math.acos(w) * 180.00/math.pi, -1 * nx, nz, ny)
    

    glBegin(GL_QUADS)
    glColor3f(0.0, 1.0, 0.0)
    glVertex3f(1.0, 0.2, -1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(1.0, 0.2, 1.0)

    glColor3f(1.0, 0.5, 0.0)
    glVertex3f(1.0, -0.2, 1.0)
    glVertex3f(-1.0, -0.2, 1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(1.0, -0.2, -1.0)

    glColor3f(1.0, 0.0, 0.0)
    glVertex3f(1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(-1.0, -0.2, 1.0)
    glVertex3f(1.0, -0.2, 1.0)

    glColor3f(1.0, 1.0, 0.0)
    glVertex3f(1.0, -0.2, -1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(1.0, 0.2, -1.0)

    glColor3f(0.0, 0.0, 1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(-1.0, -0.2, 1.0)

    glColor3f(1.0, 0.0, 1.0)
    glVertex3f(1.0, 0.2, -1.0)
    glVertex3f(1.0, 0.2, 1.0)
    glVertex3f(1.0, -0.2, 1.0)
    glVertex3f(1.0, -0.2, -1.0)
    glEnd()


def drawText(position, textString, size):
    font = pygame.font.SysFont("Courier", size, True)
    textSurface = font.render(textString, True, (255, 255, 255, 255), (0, 0, 0, 255))
    textData = pygame.image.tostring(textSurface, "RGBA", True)
    glRasterPos3d(*position)
    glDrawPixels(textSurface.get_width(), textSurface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, textData)

def quat_to_ypr(q):
    yaw   = math.atan2(2.0 * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3])
    pitch = -math.sin(2.0 * (q[1] * q[3] - q[0] * q[2]))
    roll  = math.atan2(2.0 * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3])
    pitch *= 180.0 / math.pi
    yaw   *= 180.0 / math.pi
    # Magnetic declination at UAB Escola d'Enginyeria, Cerdanyola del Valles (41°30'1"N 2°6'42"E) on Sept 25, 2020 is - 1º19'E (7.5'E)
    #yaw   += -1.317 - (datetime.datetime.now().year - 2020) * 0.125
    #yaw   += 1.317    
    roll  *= 180.0 / math.pi
    return [yaw, pitch, roll]


if __name__ == '__main__':
    main()
