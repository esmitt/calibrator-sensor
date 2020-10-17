import pygame
import math
import numpy as np
import serial
import struct
import datetime
import os
from OpenGL.GL import *
from OpenGL.GLU import *
from pygame.locals import *
from datetime import datetime
from math import sqrt, acos, sin, cos, atan2, radians
from sklearn.metrics import mean_squared_error

pitch2 = None


def measure_error(v1: np.ndarray, v2: np.ndarray) -> float:
    return mean_squared_error(v1, v2)


def compute_sensor_axis(w: float, x: float, y: float, z: float) -> np.ndarray:
    angle = 2.0 * acos(w)
    norm = sqrt(x * x + y * y + z * z)
    if norm == 0:
        return np.zeros(3)
    ux = -x / norm
    uy = z / norm
    uz = y / norm
    return np.array([ux * uy * (1 - cos(angle)) - uz * sin(angle),
                     cos(angle) + uy * uy * (1 - cos(angle)),
                     uz * uy * (1 - cos(angle) + ux * sin(angle))])


def compute_sensor_theoretical(theta: float, phi: float, sensor_axis: np.ndarray) -> np.ndarray:
    """
    :param theta is the latitude
    :param phi is the longitude
    :param sensor_axis
    """

    def origin_angles(v: np.ndarray) -> float:
        theta_rad = atan2(v[0], v[1])
        theta_deg = (theta_rad / np.pi * 180)
        theta_deg = theta_deg + ((0 if theta_rad > 0 else 360) % 360)
        return theta_deg

    theta_zero = origin_angles(sensor_axis)
    theta = theta + theta_zero
    phi = 90 - phi

    rz = np.array([[np.cos(radians(theta)), -np.sin(radians(theta)), 0],
                   [np.sin(radians(theta)), np.cos(radians(theta)), 0],
                   [0, 0, 1]])
    ry = np.array([[np.cos(radians(phi)), 0, np.sin(radians(phi))],
                   [0, 1, 0],
                   [-np.sin(radians(phi)), 0, np.cos(radians(phi))]])
    return np.matmul(rz, np.matmul(ry, sensor_axis))


def main():
    txt_file = None
    try:
        ser = serial.Serial('COM3', 115200)  # by default: bytesize=EIGHTBITS, parity=PARITY_NONE, stopbits=STOPBITS_ONE

        video_flags = OPENGL | DOUBLEBUF
        (_, _) = pygame.init()
        _ = pygame.display.set_mode((600, 500), video_flags)
        pygame.display.set_caption("Calibration")
        resize_win(640, 480)
        init()

        ticks = pygame.time.get_ticks()

        frames = 0
        flag = False
        int_flag = 0
        N_FRAMES = 50
        current_frame = 0

        if frames == 0:
            pass

        while True:
            event = pygame.event.poll()  # get a single event from the queue
            if event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    break
                elif event.key == pygame.K_r:
                    flag = not flag
                    if flag:
                        int_flag += 1
                elif event.key == pygame.K_s:  # resetting status/variables and create the output file
                    if txt_file is not None:
                        txt_file.close()

                    current_frame = 0
                    NUMBER_POSITION = 100
                    output_folder = os.path.join('tests', f'T{NUMBER_POSITION}')
                    output_folder = os.path.join(os.getcwd(), output_folder)

                    output_file = datetime.now().strftime("%Y-%m-%d--%H-%M-%S")
                    output_file += f'from0to{NUMBER_POSITION}.csv'
                    txt_file = open(os.path.join(output_folder, output_file), 'wt')
            elif event.type == QUIT:
                break

            #[w, nx, ny, nz, _, _, _, _, _, _, _, _, _, sys_cal, accel_cal, gyro_cal, rmag_cal] = read_data(ser)
            [w, nx, ny, nz, sys_cal, accel_cal, gyro_cal, mag_cal] = read_data(ser)
            draw(w, nx, ny, nz, sys_cal, accel_cal, gyro_cal, mag_cal)

            # if accel_cal == 3
            # formatting the output
            output_flag = int_flag if flag else 0
            str_output = f'{frames},{w},{nx},{ny},{nz},{output_flag}'
            sensor_axis = compute_sensor_axis(w, nx, ny, nz)
            print(sensor_axis)

            if txt_file is not None:
                txt_file.write(str_output + '\n')

            # number of frames to record
            if flag:
                if current_frame == N_FRAMES:
                    flag = False
                current_frame += 1
            else:
                current_frame = 0

            pygame.display.flip()
            frames += 1

        print("fps: %d" % ((frames * 1000) / (pygame.time.get_ticks() - ticks)))
        ser.close()
    except Exception as full_error:
        print(full_error)
    except UnboundLocalError as error_not_connected:
        print('Check the arduino connected')
        print(error_not_connected.__str__())
    finally:
        if txt_file is not None:
            txt_file.close()


def resize_win(width: int, height: int) -> ():
    """
    For resizing window
    """
    if height == 0:
        height = 1
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, 1.0 * width / height, 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()


def init():
    glShadeModel(GL_SMOOTH)
    glClearColor(0.0, 0.0, 0.0, 0.0)
    glClearDepth(1.0)
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LEQUAL)
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)


def read_data(ser: serial) -> list:
    ser.flushInput()  # Flush input buffer, discarding all it’s contents.
    while not(ser.read() == b'B' and ser.read() == b'F'):
        pass        
                
    while ser.in_waiting < 20:
        pass
        
    payload = ser.read(20)    
    
    #quaternion data
    quat = struct.unpack('4f', payload[0:16])
    w = quat[0]
    nx = quat[1]
    ny = quat[2]
    nz = quat[3]      
    
    #calibration status data
    sys_cal = payload[16]
    accel_cal = payload[17]
    gyro_cal = payload[18]
    mag_cal = payload[19]
    
    print("{:10.6f}, {:10.6f}, {:10.6f}, {:10.6f}, {:4d}, {:4d}, {:4d}, {:4d}".format(w, nx, ny, nz, sys_cal, accel_cal, gyro_cal, mag_cal)) 

    return [w, nx, ny, nz, sys_cal, accel_cal, gyro_cal, mag_cal]
"""
def read_data(ser):
    ser.reset_input_buffer()  # Flush input buffer, discarding all it’s contents.
    line = ser.readline().decode('UTF-8').replace('\n', '')  # Read and return one line from the stream.
    # print(line)

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
        # Sensor calibration status
        sys_cal = float(line[14])
        accel_cal = float(line[15])
        gyro_cal = float(line[16])
        mag_cal = float(line[17])
        # assert accel_cal == 3 and gyro_cal == 3 and mag_cal == 3, f'Calibration error as {accel_cal}, {gyro_cal},
        # {mag_cal}'
        return [w, nx, ny, nz, ax, ay, az, gx, gy, gz, mx, my, mz, sys_cal, accel_cal, gyro_cal, mag_cal]
    else:
        return [0] * 17
"""

def draw(w, nx, ny, nz, sys_cal, accel_cal, gyro_cal, mag_cal):
    def draw_sensor() -> ():
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

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()
    glTranslatef(0, 0.0, -7.0)

    # Header and footer
    draw_text((-2.6, 1.8, 2), "MARG sensor: BNO055 | Sensor controller: based on Arduino MEGA", 16)
    draw_text((-2.6, -2, 2), "Press Escape to exit.", 14)

    # Sensor calibration status
    draw_text((-2.6, 1.5, 2), "Sensor Calibration Status", 16)
    draw_text((-2.6, 1.3, 2), "Syst: %f, Accel: %f, Gyro: %f, Mag: %f" % (sys_cal, accel_cal, gyro_cal, mag_cal), 16)

    # Sensor data
    # drawText((-2.6, -1.0, 2), "Yaw: %f, Pitch: %f, Roll: %f" %(yaw, pitch, roll), 16)
    # drawText((-2.6, -1.0, 2), "Y: %f, P: %f, P-2: %f R: %f" % (yaw, pitch, pitch2, roll), 16)
    # drawText((-2.6, -1.2, 2), "qW: %f, qX: %f, qY: %f, qZ: %f" %(w, nx, ny, nz), 16)
    # drawText((-2.6, -1.6, 2), "qW: %f, qX: %f, qY: %f, qZ: %f" % (w, nx, ny, nz), 16)
    mag = math.sqrt((nx * nx) + (ny * ny) + nz * nz)
    mag = 1 if mag == 0 else mag
    nnx = nx / mag
    nny = ny / mag
    nnz = nz / mag
    draw_text((-2.6, -1.6, 2), "qW: %f, qX: %f, qY: %f, qZ: %f" % (w, nnx, nny, nnz), 16)
    draw_text((-2.6, -1.2, 2), f"angle: {float(2.0 * math.acos(w) * 180.00 / math.pi): .3f}, "
                               f"-nx:{-1 * nnx: .3f}, "
                               f"nz: {nnz: .3f}, "
                               f"ny: {nny: .3f}", 16)
    sensorAxis = compute_sensor_axis(w, nx, ny, nz)
    draw_text((-2.6, -1.4, 2), "X: %f, Y: %f, Z: %f" % (sensorAxis[0], sensorAxis[1], sensorAxis[2]), 16)
    # drawText((-2.6, -1.4, 2), "Ax: %f, Ay: %f, Az: %f" % (ax, ay, az), 16)
    # drawText((-2.6, -1.6, 2), "Gx: %f, Gy: %f, Gz: %f" % (gx, gy, gz), 16)
    # drawText((-2.6, -1.8, 2), "Mx: %f, My: %f, Mz: %f" % (mx, my, mz), 16)
    glRotatef(2 * math.acos(w) * 180.00 / math.pi, -1 * nnx, nnz, nny)
    # glRotatef(2 * math.acos(w) * 180.00/math.pi, sensorAxis[0], sensorAxis[1], sensorAxis[2])
    draw_sensor()


def draw_text(position: tuple, text: str, size: int) -> ():
    font = pygame.font.SysFont("Courier", size, True)
    textSurface = font.render(text, True, (255, 255, 255, 255), (0, 0, 0, 255))
    textData = pygame.image.tostring(textSurface, "RGBA", True)
    glRasterPos3d(*position)
    glDrawPixels(textSurface.get_width(), textSurface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, textData)


if __name__ == '__main__':
    main()
    # np.set_printoptions(precision=4, suppress=True)
    # axis = compute_sensor_axis(0.1, 0.7, 0, 0)
    # print(axis)
    # the = compute_sensor_theoretical(0, 0, axis)
    # print(the)
