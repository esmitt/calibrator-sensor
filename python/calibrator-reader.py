import pygame
import math
import numpy as np
import serial
import struct
import datetime
from OpenGL.GL import *
from OpenGL.GLU import *
from pygame.locals import *
from datetime import datetime
import sensor

# values to be stored in file
list_values = list()


def main():
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
        # flag = False
        # int_flag = 0
        current_frame = 0

        # to store initial quaternion/axis
        flag_initial = False
        list_quaternion = list()
        SAVING_FRAMES = 20
        origin_axis = None
        origin_q = None
        # to store current quaternions/axis
        current_axis = None
        flag_current = False
        last_axis = None
        while True:
            event = pygame.event.poll()  # get a single event from the queue
            # region
            if event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    break
                if event.key == pygame.K_s:
                    if len(list_values) > 0:
                        # save data into file with header
                        output_file = datetime.now().strftime("%Y-%m-%d--%H-%M-%S.csv")
                        with open(output_file, 'wt') as file:
                            file.write("Entry,qw,qx,qy,qz,ax,ay,az\n")
                            file.write(f"0,{origin_q[0]},{origin_q[1]},{origin_q[2]},{origin_q[3]},{origin_axis[0]},"
                                       f"{origin_axis[1]},{origin_axis[2]}\n")
                            for index in range(len(list_values)):
                                value = list_values[index]
                                str_output = f"{index+1},{value[0][0]},{value[0][1]},{value[0][2]},{value[0][3]}," \
                                             f"{value[1][0]},{value[1][1]},{value[1][2]}\n"
                                file.write(str_output)
                        print(f'File {output_file} saved')
                elif event.key == pygame.K_i:
                    flag_initial = True
                    list_quaternion.clear()
                    current_frame = 0
                    origin_axis = None
                elif event.key == pygame.K_p and origin_axis is not None:
                    flag_current = True
                    list_quaternion.clear()
                    current_frame = 0
                    current_axis = None
                # elif event.key == pygame.K_r:
                #     flag = not flag
                #     if flag:
                #         int_flag += 1
                # elif event.key == pygame.K_s:  # resetting status/variables and create the output file
                #     if txt_file is not None:
                #         txt_file.close()
                #
                #     current_frame = 0
                #     NUMBER_POSITION = 100
                #     output_folder = os.path.join('tests', f'T{NUMBER_POSITION}')
                #     output_folder = os.path.join(os.getcwd(), output_folder)
                #
                #     output_file = datetime.now().strftime("%Y-%m-%d--%H-%M-%S")
                #     output_file += f'from0to{NUMBER_POSITION}.csv'
                #     txt_file = open(os.path.join(output_folder, output_file), 'wt')
            elif event.type == QUIT:
                break
            # endregion

            [w, nx, ny, nz, sys_cal, accel_cal, gyro_cal, mag_cal] = read_data(ser)

            if flag_initial:
                list_quaternion.append([w, nx, ny, nz])
                if current_frame == 0:
                    current_frame = frames
                elif frames - current_frame >= SAVING_FRAMES:
                    flag_initial = False
                    origin_q, origin_axis = sensor.get_axis(list_quaternion)
                else:
                    origin_axis = str(SAVING_FRAMES - (frames - current_frame))

            if flag_current:
                list_quaternion.append([w, nx, ny, nz])
                if current_frame == 0:
                    current_frame = frames
                elif frames - current_frame >= SAVING_FRAMES:
                    flag_current = False
                    q, current_axis = sensor.get_axis(list_quaternion)
                    last_axis = current_axis
                    # save this axis, quaternion to the list
                    list_values.append([q, current_axis])
                else:
                    current_axis = str(SAVING_FRAMES - (frames - current_frame))
            else:
                if isinstance(origin_axis, np.ndarray):
                    _,_,current_axis = sensor.compute_sensor_axis(w, nx, ny, nz)
                    #print(w, nx, ny, nz)

            draw(w, nx, ny, nz, [sys_cal, accel_cal, gyro_cal, mag_cal], origin_axis, current_axis, last_axis)

            pygame.display.flip()
            frames += 1

        print("fps: %d" % ((frames * 1000) / (pygame.time.get_ticks() - ticks)))
        ser.close()
    except Exception as full_error:
        print(full_error)
    except UnboundLocalError as error_not_connected:
        print('Check the arduino connected')
        print(error_not_connected.__str__())


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


def init() -> ():
    glShadeModel(GL_SMOOTH)
    glClearColor(0.15, 0.15, 0.15, 0.0)
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
    
    # quaternion data
    quat = struct.unpack('4f', payload[0:16])
    w = quat[0]
    nx = quat[1]
    ny = quat[2]
    nz = quat[3]      
    
    # calibration status data
    sys_cal = payload[16]
    accel_cal = payload[17]
    gyro_cal = payload[18]
    mag_cal = payload[19]
    # print(f"{w:10.6f}, {nx:10.6f}, {ny:10.6f}, {nz:10.6f}")

    return [w, nx, ny, nz, sys_cal, accel_cal, gyro_cal, mag_cal]


def draw(w, nx, ny, nz: float, calibrator: list, sensor_axis, current_axis, last_axis: np.ndarray = None) -> ():
    # sys_cal, accel_cal, gyro_cal, mag_cal
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
    glTranslatef(0, -0.0, -7.0)

    # Header and footer
    draw_text((-2.6, 1.8, 2), "MARG sensor: BNO055 | Sensor controller (Arduino MEGA)", 16, (150, 100, 240, 255))
    draw_text((-2.6, -2, 2), "Press Escape to exit.", 14, (220, 100, 10, 255))
    if len(list_values) > 0:
        draw_text((-2.6, -1.8, 2), f"{len(list_values)} element(s) to save", 14, (100, 220, 10, 255))
    draw_text((0, -1.6, 2), "Press 's' to save in file", 14, (220, 100, 10, 255))
    draw_text((0, -1.8, 2), "Press 'p' to record quaternion", 14, (220, 100, 10, 255))
    draw_text((0, -2, 2), "Press 'i' for initial quaternion", 14, (220, 100, 10, 255))

    # Sensor calibration status
    draw_text((-2.6, 1.5, 2), "Sensor Calibration Status", 16)
    draw_text((-2.6, 1.3, 2), f"Syst:{calibrator[0]}, Accel:{calibrator[1]}, Gyro:{calibrator[2]}, Mag:{calibrator[3]}",
              16)
    # Sensor data
    if sensor_axis is None:
        draw_text((-2.6, -1., 2), " Waiting to set initial position", 16)
    elif isinstance(sensor_axis, str):
        draw_text((-2.6, -1., 2), f"Calculating initial position {sensor_axis}...wait", 16)
    else:
        draw_text((-2.6, -1., 2), f"origin axis: {sensor_axis[0]:.3f}, {sensor_axis[1]:.3f}, {sensor_axis[2]:.3f}", 16,
                  (180, 190, 140, 255))

    if isinstance(current_axis, str):
        draw_text((-2.6, -1.2, 2), f"Calculating current position {current_axis}...wait", 16)
    elif isinstance(current_axis, np.ndarray):
        draw_text((-2.6, -1.2, 2), f"current axis: {current_axis[0]:.3f}, {current_axis[1]:.3f}, "
                                   f"{current_axis[2]:.3f}", 16)

        draw_text((-2.6, -1.4, 2), f"current q: {w:.3f}, {nx:.3f}, {ny:.3f} {nz:.3f}", 16)

    if isinstance(last_axis, np.ndarray):
        draw_text((-2.8, -1.2, 2), f"last axis: {last_axis[0]:.3f}, {last_axis[1]:.3f}, {last_axis[2]:.3f}", 16,
                  (180, 190, 140, 255))
    # draw more in Y -1.6 or -1.8

    glRotatef(2 * math.acos(w) * 180.00 / math.pi, -nx, nz, ny)
    draw_sensor()


def draw_text(position: tuple, text: str, size: int, color: tuple = (255, 255, 255, 255)) -> ():
    font = pygame.font.SysFont("Courier", size, True)
    textSurface = font.render(text, True, color, (39, 39, 39, 255))
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
    # print(compute_sensor_axis(0,0,0,0))
    # axis = [0.2, 0.1, 0.7]
    # the = compute_sensor_theoretical(0, 0, axis)
    # print(the)
