import pygame
import math
from OpenGL.GL import *
from OpenGL.GLU import *
from pygame.locals import *
from datetime import datetime
import serial
import os

#useSerial = True # set true for using serial for data transmission, false for...
useQuat = True   # set true for using quaternions, false for using Euler angles (y,p,r)
pitch2 = None

from math import sqrt, acos, sin, cos

def computeSensorAxis(w, x, y, z) -> list:
  angle = 2.0 * acos(w)
  norm = sqrt(x*x + y*y + z*z)
  if norm == 0:
    return [0, 0, 0]
  ux = -x/norm
  uy = z/norm
  uz = y/norm
  return [ux*uy*(1 - cos(angle)) - uz*sin(angle),
          cos(angle) + uy*uy * (1 - cos(angle)),
          uz*uy*(1-cos(angle) + ux*sin(angle))]

def main():
    txt_file = None
    try:
        ser = serial.Serial('COM3', 115200)  # by default: bytesize=EIGHTBITS, parity=PARITY_NONE, stopbits=STOPBITS_ONE

        video_flags = OPENGL | DOUBLEBUF
        (numpass, numfail) = pygame.init() #initialize all imported pygame modules. The total number if successful and failed inits will be returned as a tuple
        #Setting the display mode in pygame creates a visible image surface on the monitor. The display surface is nothing more than a standard pygame surface
        #object. You let pygame choose it's bit depth by calling set_mode() with no depth argument or a depth of 0, or you can call mode_ok() to find a closest
        #matching bit depth to what you need.
        screen = pygame.display.set_mode((700, 600), video_flags)
        pygame.display.set_caption("AHRS") #Set the current window caption
        resizewin(640, 480)
        init()

        ticks = pygame.time.get_ticks()

        frames = 0
        flag = False
        int_flag = 0
        N_FRAMES = 50
        int_nframes = 0

        if frames == 0:
            pass

        while True:
            event = pygame.event.poll() #get a single event from the queue
            if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
                break

            if event.type == pygame.KEYDOWN and event.key == pygame.K_r:
                flag = not flag
                if flag:
                    int_flag += 1

            # resetting status/variables and create the output file
            if event.type == pygame.KEYDOWN and event.key == pygame.K_s:
                if txt_file is not None:
                    txt_file.close()

                int_nframes = 0
                NUMBER_POSITION = 100
                output_folder = os.path.join('tests', f'T{NUMBER_POSITION}')
                output_folder = os.path.join(os.getcwd(), output_folder)

                output_file = datetime.now().strftime("%Y-%m-%d--%H-%M-%S")
                output_file += f'from0to{NUMBER_POSITION}.csv'
                txt_file = open(os.path.join(output_folder, output_file), 'wt')

            if useQuat:
                [w, nx, ny, nz, ax, ay, az, gx, gy, gz, mx, my, mz, sys_cal, accel_cal, gyro_cal, rmag_cal] = read_data(ser)
                draw(w, nx, ny, nz, ax, ay, az, gx, gy, gz, mx, my, mz, sys_cal, accel_cal, gyro_cal, rmag_cal)

                #if accel_cal == 3
                # formatting the output
                output_flag = int_flag if flag else 0
                str_output = f'{frames},{w},{nx},{ny},{nz},{output_flag}'

                print(str_output)
                if txt_file is not None:
                    txt_file.write(str_output + '\n')

                # number of frames to record
                if flag:
                    if int_nframes == N_FRAMES:
                        flag = False
                    int_nframes += 1
                else:
                    int_nframes = 0
            else:
                [yaw, pitch, roll, ax, ay, az, gx, gy, gz, mx, my, mz] = read_data()
                draw(1, yaw, pitch, roll, ax, ay, az, gx, gy, gz, mx, my, mz)

            pygame.display.flip()
            frames += 1

        print("fps: %d" % ((frames*1000)/(pygame.time.get_ticks()-ticks)))
        ser.close()
    except Exception as error:
        print(error)
    except UnboundLocalError as error_not_connected:
        print('Check the arduino connected')
        print(error_not_connected.__str__())
    finally:
        if txt_file is not None:
            txt_file.close()


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


def read_data(ser):
    ser.reset_input_buffer() #Flush input buffer, discarding all it’s contents.

    line = ser.readline().decode('UTF-8').replace('\n', '') #Read and return one line from the stream.
    #print(line)

    if(useQuat):
        line = line.split(',')
        if len(line) == 18:
            w = float(line[1])
            nx = float(line[2])
            ny = float(line[3])
            nz = float(line[4])
            ax = float(line[5])
            ay = float(line[6])
            az = float(line[7])
            gx = float(line[8])
            gy = float(line[9])
            gz = float(line[10])
            mx = float(line[11])
            my = float(line[12])
            mz = float(line[13])
            #Sensor calibration status
            sys_cal = float(line[14])
            accel_cal = float(line[15])
            gyro_cal = float(line[16])
            mag_cal = float(line[17])
            #assert accel_cal == 3 and gyro_cal == 3 and mag_cal == 3, f'Calibration error as {accel_cal}, {gyro_cal}, {mag_cal}'
            #return [w, nx, ny, nz, ax, ay, az, gx, gy, gz, mx, my, mz]
            return [w, nx, ny, nz, ax, ay, az, gx, gy, gz, mx, my, mz, sys_cal, accel_cal, gyro_cal, mag_cal]
        else:
            #return [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
            return [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    else: 
        line = line.split(',')
        if len(line) == 17:
            yaw = float(line[1])
            pitch = float(line[2])
            roll = float(line[3])
            ax = float(line[4])
            ay = float(line[5])
            az = float(line[6])
            gx = float(line[7])
            gy = float(line[8])
            gz = float(line[9])
            mx = float(line[10])
            my = float(line[11])
            mz = float(line[12])
            #Sensor calibration status
            sys_cal = float(line[13])
            accel_cal = float(line[14])
            gyro_cal = float(line[15])
            mag_cal = float(line[16])

            return [yaw, pitch, roll, ax, ay, az, gx, gy, gz, mx, my, mz, sys_cal, accel_cal, gyro_cal, mag_cal]
        else:
            return [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]


def draw(w, nx, ny, nz, ax, ay, az, gx, gy, gz, mx, my, mz, sys_cal, accel_cal, gyro_cal, mag_cal):
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()
    glTranslatef(0, 0.0, -7.0)

    #Header and footer
    drawText((-2.6, 1.8, 2), "MARG sensor: BNO055 | Sensor controller: based on Arduino MEGA", 16)    
    drawText((-2.6, -2, 2), "Press Escape to exit.", 14)

    #Sensor calibration status
    drawText((-2.6, 1.5, 2), "Sensor Calibration Status", 16)
    drawText((-2.6, 1.3, 2), "Syst: %f, Accel: %f, Gyro: %f, Mag: %f" %(sys_cal, accel_cal, gyro_cal, mag_cal), 16)

    #Sensor data
    if(useQuat):
        [yaw, pitch , pitch2, roll] = quat_to_ypr([w, nx, ny, nz])
        #drawText((-2.6, -1.0, 2), "Yaw: %f, Pitch: %f, Roll: %f" %(yaw, pitch, roll), 16)
        drawText((-2.6, -1.0, 2), "Y: %f, P: %f, P-2: %f R: %f" % (yaw, pitch, pitch2, roll), 16)
        #drawText((-2.6, -1.2, 2), "qW: %f, qX: %f, qY: %f, qZ: %f" %(w, nx, ny, nz), 16)
        #drawText((-2.6, -1.6, 2), "qW: %f, qX: %f, qY: %f, qZ: %f" % (w, nx, ny, nz), 16)
        mag = math.sqrt((nx * nx) + (ny * ny) + nz * nz)
        mag = 1 if mag == 0 else mag
        nnx = nx / mag
        nny = ny / mag
        nnz = nz / mag
        drawText((-2.6, -1.6, 2), "qW: %f, qX: %f, qY: %f, qZ: %f" % (w, nnx, nny, nnz), 16)
        drawText((-2.6, -1.2, 2), f"angle: {float(2.0 * math.acos(w) * 180.00 / math.pi): .3f}, "
                                  f"-nx:{-1 * nnx: .3f}, "
                                  f"nz: {nnz: .3f}, "
                                  f"ny: {nny: .3f}", 16)
        sensorAxis = computeSensorAxis(w, nx, ny, nz)
        drawText((-2.6, -1.4, 2), "X: %f, Y: %f, Z: %f" % (sensorAxis[0], sensorAxis[1], sensorAxis[2]), 16)
        #drawText((-2.6, -1.4, 2), "Ax: %f, Ay: %f, Az: %f" % (ax, ay, az), 16)
        #drawText((-2.6, -1.6, 2), "Gx: %f, Gy: %f, Gz: %f" % (gx, gy, gz), 16)
        #drawText((-2.6, -1.8, 2), "Mx: %f, My: %f, Mz: %f" % (mx, my, mz), 16)
        glRotatef(2 * math.acos(w) * 180.00/math.pi, -1 * nnx, nnz, nny)
        #glRotatef(2 * math.acos(w) * 180.00/math.pi, sensorAxis[0], sensorAxis[1], sensorAxis[2])
    else:
        yaw = nx
        pitch = ny
        roll = nz
        drawText((-2.6, -1.0, 2), "Yaw: %f, Pitch: %f, Roll: %f" %(yaw, pitch, roll), 16)
        drawText((-2.6, -1.2, 2), "qW: %f, qX: %f, qY: %f, qZ: %f" %(w, nx, ny, nz), 16)
        drawText((-2.6, -1.4, 2), "Ax: %f, Ay: %f, Az: %f" % (ax, ay, az), 16)
        drawText((-2.6, -1.6, 2), "Gx: %f, Gy: %f, Gz: %f" % (gx, gy, gz), 16)
        drawText((-2.6, -1.8, 2), "Mx: %f, My: %f, Mz: %f" % (mx, my, mz), 16)
        glRotatef(-roll, 0.00, 0.00, 1.00)
        glRotatef(pitch, 1.00, 0.00, 0.00)
        glRotatef(yaw, 0.00, 1.00, 0.00)

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
    pitch2 = -math.asin(2.0 * (q[1] * q[3] - q[0] * q[2]))
    roll  = math.atan2(2.0 * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3])
    pitch *= 180.0 / math.pi
    yaw   *= 180.0 / math.pi
    # Magnetic declination at UAB Escola d'Enginyeria, Cerdanyola del Valles (41°30'1"N 2°6'42"E) on Sept 25, 2020 is - 1º19'E (7.5'E)
    #yaw   += -1.317 - (datetime.datetime.now().year - 2020) * 0.125
    yaw   += -1.317    
    roll  *= 180.0 / math.pi
    return [yaw, pitch, pitch2, roll]


if __name__ == '__main__':
    main()
