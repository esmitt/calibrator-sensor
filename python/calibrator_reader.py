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
import pickle
import struct

# values to be stored in file
list_values = list()


def main():
    try:
        ser = serial.Serial('COM3', 115200)  # by default: bytesize=EIGHTBITS, parity=PARITY_NONE, stopbits=STOPBITS_ONE

        video_flags = OPENGL | DOUBLEBUF
        (_, _) = pygame.init()
        _ = pygame.display.set_mode((600, 600), video_flags)
        pygame.display.set_caption("Calibration")
        resize_win(600, 600)
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
        list_axis = None
        list_origin_axis = list()
        ax0, ay0, az0 = None, None, None
        theta, phi = 0, 0
        list_q = None
        initial_q = None

        while True:
            event = pygame.event.poll()  # get a single event from the queue
            # region
            if event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    break
                # if event.key == pygame.K_s:
                #     if len(list_values) > 0:
                #         # save data into file with header
                #         output_file = datetime.now().strftime("%Y-%m-%d--%H-%M-%S.csv")
                #         with open(output_file, 'wt') as file:
                #             file.write("Entry,qw,qx,qy,qz,ax,ay,az\n")
                #             file.write(f"0,{origin_q[0]},{origin_q[1]},{origin_q[2]},{origin_q[3]},{origin_axis[0]},"
                #                        f"{origin_axis[1]},{origin_axis[2]}\n")
                #             for index in range(len(list_values)):
                #                 value = list_values[index]
                #                 str_output = f"{index+1},{value[0][0]},{value[0][1]},{value[0][2]},{value[0][3]}," \
                #                              f"{value[1][0]},{value[1][1]},{value[1][2]}\n"
                #                 file.write(str_output)
                #         print(f'File {output_file} saved')
                elif event.key == pygame.K_i:
                    #flag_initial = True
                    #list_quaternion.clear()
                    #current_frame = 0
                    #origin_axis = Nnone
                    list_origin_axis.clear()
                    initial_q = [w, nx, ny, nz]
                    print(initial_q)
                    ax0, ay0, az0 = sensor.compute_axis(w, nx, ny, nz)
                    list_origin_axis.append(ax0)
                    list_origin_axis.append(ay0)
                    list_origin_axis.append(az0)
                    print("Initial quaternion saved")
                elif event.key == pygame.K_a:
                    theta += 10
                elif event.key == pygame.K_s:
                    theta -= 10
                elif event.key == pygame.K_d:
                    phi += 10
                elif event.key == pygame.K_f:
                    phi -= 10
                elif event.key == pygame.K_c:
                    if list_q is None:
                        list_q = list()
                        list_q.append(list(initial_q))
                    list_q.append([w, nx, ny, nz])
                    print(f"Quaternion #{len(list_q)} saved")
                elif event.key == pygame.K_v:
                    yaws = sensor.calibrate_angle0_world(initial_q, list_q)
                    yaws_0 = sensor.calibrate_angle0_world((1, 0, 0, 1), list_q)
                    for index in range(len(yaws)):
                        y = yaws[index]
                        y0 = yaws_0[index]
                        print(f"{y} --> {y0}")
                    output_file = datetime.now().strftime("quaternions-%Y-%m-%d--%H-%M-%S.q")
                    open_file = open(output_file, "wb")
                    pickle.dump(list_q, open_file)
                    print(f"saved in {output_file}")
                    open_file.close()


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

            [w, nx, ny, nz, yaw, pitch, roll, gravity_x, gravity_y, gravity_z, sys_cal, accel_cal, gyro_cal, mag_cal] = read_data(ser)

            if flag_initial:
                list_quaternion.append([w, nx, ny, nz])
                if current_frame == 0:
                    current_frame = frames
                elif frames - current_frame >= SAVING_FRAMES:
                    flag_initial = False
                    origin_q, origin_axis = sensor.get_axis(list_quaternion)
                else:
                    origin_axis = str(SAVING_FRAMES - (frames - current_frame))
            # if flag_initial:
            #     list_quaternion.append([w, nx, ny, nz])
            #     if current_frame == 0:
            #         current_frame = frames
            #     elif frames - current_frame >= SAVING_FRAMES:
            #         flag_initial = False
            #         origin_q, origin_axis = sensor.get_axis(list_quaternion)
            #     else:
            #         origin_axis = str(SAVING_FRAMES - (frames - current_frame))
            #
            # if flag_current:
            #     list_quaternion.append([w, nx, ny, nz])
            #     if current_frame == 0:
            #         current_frame = frames
            #     elif frames - current_frame >= SAVING_FRAMES:
            #         flag_current = False
            #         q, current_axis = sensor.get_axis(list_quaternion)
            #         last_axis = current_axis
            #         # save this axis, quaternion to the list
            #         list_values.append([q, current_axis])
            #     else:
            #         current_axis = str(SAVING_FRAMES - (frames - current_frame))
            # else:
            #     if isinstance(origin_axis, np.ndarray):
            #         list_axis = list()
            #         a, b, c = sensor.compute_sensor_axis2(w, nx, ny, nz)
            #         list_axis.append(a)
            #         list_axis.append(b)
            #         list_axis.append(c)
            #         #print(w, nx, ny, nz)

            # list_axis = list()
            # #rx, ry, rz = sensor.compute_sensor_axis2(w, nx, ny, nz)
            # ax, ay, az = sensor.compute_axis(w, nx, ny, nz)
            # list_axis.append(ax)
            # list_axis.append(ay)
            # list_axis.append(az)
            draw(w, nx, ny, nz, [yaw, pitch, roll], [gravity_x, gravity_y, gravity_z], [sys_cal, accel_cal, gyro_cal, mag_cal], initial_q, list_q, list_origin_axis, (theta, phi))

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
    ser.reset_input_buffer()  # Flush input buffer, discarding all it’s contents.

    while not (ser.read() == b'B' and ser.read() == b'F'):
        pass

    while ser.in_waiting < 44:
        pass

    payload = ser.read(44)

    # quaternion
    quat = struct.unpack('4f', payload[0:16])
    w = quat[0]
    nx = quat[1]
    ny = quat[2]
    nz = quat[3]

    # Euler angles - degrees
    euler_ang = struct.unpack('3f', payload[16:28])
    yaw = euler_ang[0]
    pitch = euler_ang[1]
    roll = euler_ang[2]

    # gravity vector  - m/s^2
    gravity = struct.unpack('3f', payload[28:40])
    gravity_x = gravity[0]
    gravity_y = gravity[1]
    gravity_z = gravity[2]

    # calibration status
    sys_cal = payload[40]
    accel_cal = payload[41]
    gyro_cal = payload[42]
    mag_cal = payload[43]

    # print("{:10.6f}, {:10.6f}, {:10.6f}, {:10.6f}, {:10.6f}, {:10.6f}, {:10.6f}, {:10.6f}, {:10.6f}, {:10.6f}, {:4d}, {:4d}, {:4d}, {:4d}".format(w, nx, ny, nz,
    #                                                                                                                                              yaw, pitch, roll,
    #                                                                                                                                              gravity_x, gravity_y, gravity_z,
    #                                                                                                                                              sys_cal, accel_cal, gyro_cal, mag_cal))

    return [w, nx, ny, nz, yaw, pitch, roll, gravity_x, gravity_y, gravity_z, sys_cal, accel_cal, gyro_cal, mag_cal]


#def draw(w, nx, ny, nz: float, calibrator: list, list_axis: list, current_axis, last_axis: np.ndarray = None) -> ():
def draw(w, nx, ny, nz: float, ypr: list, gravity: list, calibrator: list, initial_q: tuple, list_q:list,
         list_origin_axis: list, angles: tuple) -> ():
    # sys_cal, accel_cal, gyro_cal, mag_cal
    def draw_sensor() -> ():
        glBegin(GL_QUADS)

        glColor3f(0.75, 0.5, 0.05)
        glVertex3f(1.0, 0.2, -1.0)
        glVertex3f(-1.0, 0.2, -1.0)
        glVertex3f(-1.0, 0.2, 1.0)
        glVertex3f(1.0, 0.2, 1.0)

        glColor3f(0.10, .80, 0.10)
        glVertex3f(1.0, -0.2, 1.0)
        glVertex3f(-1.0, -0.2, 1.0)
        glVertex3f(-1.0, -0.2, -1.0)
        glVertex3f(1.0, -0.2, -1.0)

        glColor3f(.87, 0.10, 0.30)
        glVertex3f(1.0, 0.2, 1.0)
        glVertex3f(-1.0, 0.2, 1.0)
        glVertex3f(-1.0, -0.2, 1.0)
        glVertex3f(1.0, -0.2, 1.0)

        glColor3f(.80, .80, 0.80)
        glVertex3f(1.0, -0.2, -1.0)
        glVertex3f(-1.0, -0.2, -1.0)
        glVertex3f(-1.0, 0.2, -1.0)
        glVertex3f(1.0, 0.2, -1.0)

        glColor3f(0.15, 0.05, .850)
        glVertex3f(-1.0, 0.2, 1.0)
        glVertex3f(-1.0, 0.2, -1.0)
        glVertex3f(-1.0, -0.2, -1.0)
        glVertex3f(-1.0, -0.2, 1.0)

        glColor3f(.90, 0.10, 1.0)
        glVertex3f(1.0, 0.2, -1.0)
        glVertex3f(1.0, 0.2, 1.0)
        glVertex3f(1.0, -0.2, 1.0)
        glVertex3f(1.0, -0.2, -1.0)
        glEnd()

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()
    #glTranslatef(0, -0.0, -7.0)

    startXRight = -1.0
    startYRight = +.90

    # Header and footer
    draw_text((startXRight, startYRight, -2.5), "MARG sensor: BNO055 | Sensor controller (Arduino MEGA)", 16, (150, 100, 240, 255))
    draw_text((startXRight, startYRight - 0.1, -2.5), "Press Escape to exit.", 14, (220, 100, 10, 255))
    if len(list_values) > 0:
        draw_text((startXRight + 1.0, startYRight - 0.1, -2.5), f"{len(list_values)} element(s) to save", 14, (100, 220, 10, 255))
    draw_text((startXRight, startYRight - 0.2, -2.5), "Press 's' to save in file", 14, (220, 100, 10, 255))
    draw_text((startXRight + 1.0, startYRight - 0.2, -2.5), "Press 'p' to record quaternion", 14, (220, 100, 10, 255))
    draw_text((startXRight, startYRight - 0.3, -2.5), "Press 'i' for initial quaternion", 14, (220, 100, 10, 255))

    # Sensor values received directly
    draw_text((startXRight, startYRight - 0.4, -2.5), "Sensor Calibration Status", 12, (120, 225, 10, 255))
    draw_text((startXRight + 1.0, startYRight - 0.4, -2.5), f"Syst:{calibrator[0]}, Accel:{calibrator[1]}, Gyro:{calibrator[2]}, Mag:{calibrator[3]}",
              12, (120, 225, 10, 255))
    draw_text((startXRight, startYRight - 0.45, -2.5), "Sensor Gravity Values", 12, (120, 225, 10, 255))
    draw_text((startXRight + 1.0, startYRight - 0.45, -2.5), f"X:{gravity[0]:.4f}, Y:{gravity[1]:.4f}, "
                                                             f"Z:{gravity[2]:.4f}", 12, (120, 225, 10, 255))

    draw_text((startXRight, startYRight - 0.5, -2.5), "Sensor Gravity Values", 12, (120, 225, 10, 255))
    draw_text((startXRight + 1.0, startYRight - 0.5, -2.5), f"X:{gravity[0]:.4f}, Y:{gravity[1]:.4f}, "
                                                             f"Z:{gravity[2]:.4f}", 12, (120, 225, 10, 255))

    draw_text((startXRight, startYRight - 0.5, -2.5), "Sensor Yaw-Pitch-Roll Values", 12, (120, 225, 10, 255))
    draw_text((startXRight + 1.0, startYRight - 0.5, -2.5), f"Yaw:{ypr[0]:.4f}, Pitch:{ypr[1]:.4f}, "
                                                            f"Roll:{ypr[2]:.4f}", 12, (120, 225, 10, 255))

    draw_text((startXRight, startYRight - 0.55, -2.5), "Sensor Quaternion Values", 12, (120, 225, 10, 255))
    draw_text((startXRight + 1.0, startYRight - 0.55, -2.5), f"({w:.4f}, {nx:.4f}, {ny:.4f}, {nz:.4f})", 12,
              (120, 225, 10, 255))

    # Sensor data
    # if sensor_axis is None:
    #     draw_text((-2.6, -1., 2), " Waiting to set initial position", 16)
    # elif isinstance(sensor_axis, str):
    #     draw_text((-2.6, -1., 2), f"Calculating initial position {sensor_axis}...wait", 16)
    # else:
    #     draw_text((-2.6, -1., 2), f"origin axis: {sensor_axis[0]:.3f}, {sensor_axis[1]:.3f}, {sensor_axis[2]:.3f}", 16,
    #               (180, 190, 140, 255))

    startYRight = -0.45

    # quaternion
    norm = math.sqrt(nx * nx + ny * ny + nz * nz)
    if norm == 0:
        return np.zeros(3)
    ux = nx / norm
    uy = ny / norm
    uz = nz / norm
    angle = 2.0 * math.acos(w) * 180.00 / math.pi
    draw_text((startXRight, startYRight, -2.5), f"Q normalized: ({angle:.3f}, {ux:.3f}, {uy:.3f} {uz:.3f})", 14)

    # # axis
    # draw_text((startXRight, startYRight - 0.1, -2.5),
    #           f"rx: [{R[:, 0]:.2f}, {R[:, 1]:.2f}, {R[:,2]:.2f}]", 16,
    #           color=(199, 199, 39, 255))

    R = sensor.compute_axis(w, nx, ny, nz)
    #draw_text((startXRight, startYRight - 0.1, -2.5), f"rx: {list_axis[0]}", 16, color=(199, 199, 39, 255))
    draw_text((startXRight, startYRight - 0.05, -2.5), "Compute_axis:", 14, color=(150, 250, 99, 255))
    draw_text((startXRight, startYRight - 0.10, -2.5), f"rx: [{R[0][0]:.2f}, {R[0][1]:.2f}, {R[0][2]:.2f}]", 14, color = (230, 230, 39, 255))
    draw_text((startXRight, startYRight - 0.15, -2.5), f"ry: [{R[1][0]:.2f}, {R[1][1]:.2f}, {R[1][2]:.2f}]", 14, color = (230, 230, 39, 255))
    draw_text((startXRight, startYRight - 0.20, -2.5), f"rz: [{R[2][0]:.2f}, {R[2][1]:.2f}, {R[2][2]:.2f}]", 14, color = (230, 230, 39, 255))
    yaw, pitch, roll = sensor.rotation_matrix_to_ypr(np.array(R))
    draw_text((startXRight, startYRight - 0.25, -2.5), f"{{{yaw:.3f}, {pitch:.3f}, {roll:.3f}}}", 14,
              color=(230, 130, 99, 255))

    # other techniques
    draw_text((startXRight + 1, startYRight - 0.05, -2.5), "get_rotmat:", 14, color=(150, 250, 99, 255))
    R = sensor.get_rotmat(w, nx, ny, nz)
    draw_text((startXRight + 1, startYRight - 0.10, -2.5),
              f"rx: [{R[0][0]:.2f}, {R[0][1]:.2f}, {R[0][2]:.2f}]", 14,
              color=(230, 230, 39, 255))
    draw_text((startXRight + 1, startYRight - 0.15, -2.5),
              f"ry: [{R[1][0]:.2f}, {R[1][1]:.2f}, {R[1][2]:.2f}]", 14,
              color=(230, 230, 39, 255))
    draw_text((startXRight + 1, startYRight - 0.20, -2.5),
              f"rz: [{R[2][0]:.2f}, {R[2][1]:.2f}, {R[2][2]:.2f}]", 14,
              color=(230, 230, 39, 255))
    yaw, pitch, roll = sensor.rotation_matrix_to_ypr(R)
    draw_text((startXRight + 1, startYRight - 0.25, -2.5), f"{{{yaw:.3f}, {pitch:.3f}, {roll:.3f}}}", 14,
              color=(230, 130, 99, 255))

    startYRight -= 0.25
    draw_text((startXRight, startYRight - 0.05, -2.5), "quat_to_rotmat_world:", 14, color=(150, 250, 99, 255))
    R = sensor.quat_to_rotmat_world([w, nx, ny, nz])
    draw_text((startXRight, startYRight - 0.10, -2.5),
              f"rx: [{R[0][0]:.2f}, {R[0][1]:.2f}, {R[0][2]:.2f}]", 14,
              color=(230, 230, 39, 255))
    draw_text((startXRight, startYRight - 0.15, -2.5),
              f"ry: [{R[1][0]:.2f}, {R[1][1]:.2f}, {R[1][2]:.2f}]", 14,
              color=(230, 230, 39, 255))
    draw_text((startXRight, startYRight - 0.20, -2.5),
              f"rz: [{R[2][0]:.2f}, {R[2][1]:.2f}, {R[2][2]:.2f}]", 14,
              color=(230, 230, 39, 255))
    yaw, pitch, roll = sensor.rotation_matrix_to_ypr(R)
    draw_text((startXRight, startYRight - 0.25, -2.5), f"{{{yaw:.3f}, {pitch:.3f}, {roll:.3f}}}", 14,
              color=(230, 130, 99, 255))

    startXRight += 1.0
    if initial_q is not None:
        # R = sensor.quat_to_rotmat_world([w, nx, ny, nz])
        # yaw, pitch, roll = sensor.rotation_matrix_to_ypr(R)
        # draw_text((startXRight, startYRight -0.05, -2.5), f"0){yaw:.3f}, {pitch:.3f}, {roll:.3f}}}", 14,
        #           color=(240, 130, 199, 255))
        R = sensor.quat_to_rotmat_world([initial_q[0], initial_q[1], initial_q[2], initial_q[3]])
        yaw, pitch, roll = sensor.rotation_matrix_to_ypr(R)
        draw_text((startXRight, startYRight - 0.05, -2.5), f"0){yaw:.3f}, {pitch:.3f}, {roll:.3f}}}", 14,
                  color=(240, 130, 199, 255))
        startYRight -= 0.05

        if list_q is not None:
            for index in range(1, len(list_q)):
                startYRight -= 0.05
                R = sensor.quat_to_rotmat_world([list_q[index][0], list_q[index][1], list_q[index][2], list_q[index][3]])
                yaw, pitch, roll = sensor.rotation_matrix_to_ypr(R)
                draw_text((startXRight, startYRight, -2.5), f"{index}){yaw:.3f}, {pitch:.3f}, {roll:.3f}}}", 14,
                          color=(240, 130, 199, 255))
# if len(list_origin_axis) == 3:
#     draw_text((startXRight, startYRight - 0.50, -2.5),
#               f"rx: [{list_origin_axis[0][0]:.2f}, {list_origin_axis[0][1]:.2f}, {list_origin_axis[0][2]:.2f}]", 14,
#               color=(199, 39, 199, 255))
#     draw_text((startXRight, startYRight - 0.55, -2.5),
#               f"ry: [{list_origin_axis[1][0]:.2f}, {list_origin_axis[1][1]:.2f}, {list_origin_axis[1][2]:.2f}]", 14,
#               color=(199, 39, 199, 255))
#     draw_text((startXRight, startYRight - 0.60, -2.5),
#               f"rz: [{list_origin_axis[2][0]:.2f}, {list_origin_axis[2][1]:.2f}, {list_origin_axis[2][2]:.2f}]", 14,
#               color=(199, 39, 199, 255))

    # angles
    # Tax, Tay, Taz = sensor.compute_axis_theoretical(angles[0], angles[1], list_origin_axis[0], list_origin_axis[1], list_origin_axis[2])
    #
    # startXRight += 0.75
    # draw_text((startXRight, startYRight - 0.35, -2.5),f"theta: {angles[0]}, phi: {angles[1]}", 14, color=(155, 155, 155, 255))
    # draw_text((startXRight, startYRight - 0.4, -2.5),
    #           f"Tax: [{Tax[0]:.2f}, {Tax[1]:.2f}, {Tax[2]:.2f}]", 14, color=(39, 199, 199, 255))
    # draw_text((startXRight, startYRight - 0.45, -2.5),
    #           f"Tay: [{Tay[0]:.2f}, {Tay[1]:.2f}, {Tay[2]:.2f}]", 14, color=(39, 199, 199, 255))
    # draw_text((startXRight, startYRight - 0.50, -2.5),
    #           f"Taz: [{Taz[0]:.2f}, {Taz[1]:.2f}, {Taz[2]:.2f}]", 14, color=(39, 199, 199, 255))


# # if isinstance(current_axis, str):
#     draw_text((-2.6, -1.2, 2), f"Calculating current position {current_axis}...wait", 16)
# elif isinstance(current_axis, np.ndarray):
#     draw_text((-2.6, -1.2, 2), f"current axis: {current_axis[0]:.3f}, {current_axis[1]:.3f}, "
#                                f"{current_axis[2]:.3f}", 16)
#
#
#
#
# if isinstance(last_axis, np.ndarray):
#     draw_text((-2.8, -1.2, 2), f"last axis: {last_axis[0]:.3f}, {last_axis[1]:.3f}, {last_axis[2]:.3f}", 16,
#               (180, 190, 140, 255))
# draw more in Y -1.6 or -1.8
    glPushMatrix()
    glTranslatef(0, -0.1, -10)
    glRotatef(2 * math.acos(w) * 180.00 / math.pi, -nx, nz, ny)
    glScalef(1, 4, 1)
    draw_sensor()
    glPopMatrix()


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

