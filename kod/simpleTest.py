try:
    import vrep as sim
    import math
    import time
    import random
    import numpy as np  # do this at the top of the program.
except:
    print('--------------------------------------------------------------')
    print('"sim.py" could not be imported. This means very probably that')
    print('either "sim.py" or the remoteApi library could not be found.')
    print('Make sure both are in the same folder as this file,')
    print('or appropriately adjust the file "sim.py"')
    print('--------------------------------------------------------------')
    print('')


class blok:
    def __init__(self):
        self.left = None
        self.right = None
        self.up = None
        self.down = None
        self.value = 0


def check(m,x,y):
    for i in range(x):
        for j in range(y):
            if m[i][j].value == 0:
                return False
    return True


def directory_check(orientation):
    if -1.05 >= orientation > -2:
        return "up"
    if -1.05 < orientation <= 0.50:
        return "left"
    if 0.50 < orientation < 2.18:
        return "back"
    else:
        return "right"


def quarter(tmp):
    return math.ceil(tmp * 4) / 4


def vel_amount(cl_h, l_j, r_j, vel):
    sim.simxPauseCommunication(cl_h, True)
    sim.simxSetJointTargetVelocity(cl_h, l_j[0], vel, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(cl_h, r_j[0], vel, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(cl_h, l_j[1], vel, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(cl_h, r_j[1], vel, sim.simx_opmode_oneshot)
    sim.simxPauseCommunication(cl_h, False)
    return


def map_set(m, x_pos, y_pos, left, up, right, down, direction):
    if -1.05 >= direction > -2:
        m[x_pos][y_pos].left = left
        m[x_pos][y_pos].up = up
        m[x_pos][y_pos].right = right
        m[x_pos][y_pos].down = down
    else:
        if -1.05 < direction <= 0.50:
            m[x_pos][y_pos].left = up
            m[x_pos][y_pos].up = right
            m[x_pos][y_pos].right = down
            m[x_pos][y_pos].down = left
        else:
            if 0.50 < direction < 2.18:
                m[x_pos][y_pos].left = right
                m[x_pos][y_pos].up = down
                m[x_pos][y_pos].right = left
                m[x_pos][y_pos].down = up
            else:
                m[x_pos][y_pos].left = down
                m[x_pos][y_pos].up = left
                m[x_pos][y_pos].right = up
                m[x_pos][y_pos].down = right
    m[x_pos][y_pos].value = 999
    return m


def turn(cl_h, pl_h, l_j, r_j, direction, velocity):
    if direction == "up":
        return
    previousAngle = sim.simxGetObjectOrientation(cl_h, pl_h, -1, sim.simx_opmode_blocking)[1]
    rot = 0
    error = 0.014
    val = 1.3
    val2 = 1.1
    l_znak = 1
    r_znak = 1

    if direction == "back":
        rotAmount = math.pi
        val = 1.15
    else:
        rotAmount = math.pi / 2
        val2 = 1.05

    if direction == "left":
        l_znak = -1
    else:
        r_znak = -1
    sim.simxPauseCommunication(cl_h, True)

    sim.simxSetJointTargetVelocity(cl_h, l_j[0], velocity * l_znak, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(cl_h, r_j[0], velocity * r_znak, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(cl_h, l_j[1], velocity * l_znak, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(cl_h, r_j[1], velocity * r_znak, sim.simx_opmode_oneshot)
    sim.simxPauseCommunication(cl_h, False)

    while 1:
        cur_angle = sim.simxGetObjectOrientation(cl_h, pl_h, -1, sim.simx_opmode_blocking)[1]
        d = cur_angle[2] - previousAngle[2]

        if d >= 0:
            d = math.fmod(d + math.pi, 2 * math.pi) - math.pi
        else:
            d = math.fmod(d - math.pi, 2 * math.pi) + math.pi

        rot = rot + d
        previousAngle = cur_angle

        if val * math.fabs(rot) > rotAmount:
            sim.simxPauseCommunication(cl_h, True)
            sim.simxSetJointTargetVelocity(cl_h, l_j[0], (velocity * l_znak) / 4, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(cl_h, r_j[0], (velocity * r_znak) / 4, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(cl_h, l_j[1], (velocity * l_znak) / 4, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(cl_h, r_j[1], (velocity * r_znak) / 4, sim.simx_opmode_oneshot)
            sim.simxPauseCommunication(cl_h, False)
        if val2 * math.fabs(rot) > rotAmount:
            sim.simxPauseCommunication(cl_h, True)
            sim.simxSetJointTargetVelocity(cl_h, l_j[0], (velocity * l_znak) / 10, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(cl_h, r_j[0], (velocity * r_znak) / 10, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(cl_h, l_j[1], (velocity * l_znak) / 10, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(cl_h, r_j[1], (velocity * r_znak) / 10, sim.simx_opmode_oneshot)
            sim.simxPauseCommunication(cl_h, False)
            if (previousAngle[2] < 0.003 and previousAngle[2] > - 0.003) or (
                    previousAngle[2] > (math.pi / 2) - 0.003 and previousAngle[2] < (math.pi / 2) + 0.003) or (
                    previousAngle[2] < (-math.pi / 2) + 0.003 and previousAngle[2] > (-math.pi / 2) - 0.003) or (
                    previousAngle[2] > math.pi - 0.003 or previousAngle[2] < (-math.pi) + 0.003):
                vel_amount(cl_h, l_j, r_j, 0)
                return


def go_str2(cl_h, pl_h, l_j, r_j, velocity, distance):
    direction = 1
    cur_angle = sim.simxGetObjectOrientation(cl_h, pl_h, -1, sim.simx_opmode_blocking)[1]

    if 1 < math.fabs(cur_angle[2]) < 2:
        direction = 0

    old_pos = sim.simxGetObjectPosition(cl_h, pl_h, -1, sim.simx_opmode_blocking)[1]
    disAmount = 0
    vel_amount(cl_h, l_j, r_j, velocity)

    while 1:
        pos = sim.simxGetObjectPosition(cl_h, pl_h, -1, sim.simx_opmode_blocking)[1]
        d = pos[direction] - old_pos[direction]
        disAmount = disAmount + math.fabs(d)
        if 1.2 * disAmount > distance * 0.5:
            vel_amount(cl_h, l_j, r_j, velocity / 4)
            if 1.1 * disAmount > distance * 0.5:
                vel_amount(cl_h, l_j, r_j, velocity / 6)
            if math.fabs((pos[direction] + 0.25) % 0.5) < 0.015:
                vel_amount(cl_h, l_j, r_j, 0)
                return
        else:
            vel_amount(cl_h, l_j, r_j, velocity)
        old_pos = pos


"""         print("(",pos_x,",",pos_y,")")"""


def go_str(cl_h, pl_h, l_j, r_j, velocity, s_tab, m):
    direction = 1
    cur_angle = sim.simxGetObjectOrientation(cl_h, pl_h, -1, sim.simx_opmode_blocking)[1]

    if 1 < math.fabs(cur_angle[2]) < 2:
        direction = 0

    old_pos = sim.simxGetObjectPosition(cl_h, pl_h, -1, sim.simx_opmode_blocking)[1]
    disAmount = 0
    vel_amount(cl_h, l_j, r_j, velocity)

    while 1:
        pos = sim.simxGetObjectPosition(cl_h, pl_h, -1, sim.simx_opmode_blocking)[1]
        d = pos[direction] - old_pos[direction]
        disAmount = disAmount + math.fabs(d)
        detect1 = sim.simxReadProximitySensor(cl_h, s_tab[0], sim.simx_opmode_blocking)[1]
        detect2 = sim.simxReadProximitySensor(cl_h, s_tab[1], sim.simx_opmode_blocking)[1]
        if (detect2 or (detect1 == False)) and disAmount > 0.25:
            vel_amount(cl_h, l_j, r_j, velocity / 8)
            tmp = quarter(pos[direction])
            if math.fabs(tmp - pos[direction]) < 0.04:
                vel_amount(cl_h, l_j, r_j, 0)
                detect3 = sim.simxReadProximitySensor(cl_h, s_tab[2], sim.simx_opmode_blocking)[1]
                detect4 = sim.simxReadProximitySensor(cl_h, s_tab[3], sim.simx_opmode_blocking)[1]
                pos_y = math.trunc((pos[1] + 2) / 0.5)
                pos_x = math.trunc((pos[0] + 2) / 0.5)
                return [[detect1, detect2, detect3, detect4], m, pos_x, pos_y]
        else:
            vel_amount(cl_h, l_j, r_j, velocity)

        if math.fabs((pos[direction] + 0.25) % 0.5) < 0.04:
            pos_y = math.trunc((pos[1] + 2) / 0.5)
            pos_x = math.trunc((pos[0] + 2) / 0.5)
            cur_angle = sim.simxGetObjectOrientation(clientID, handle1, -1, sim.simx_opmode_blocking)[1]
            detect3 = sim.simxReadProximitySensor(cl_h, s_tab[2], sim.simx_opmode_blocking)[1]
            detect4 = sim.simxReadProximitySensor(cl_h, s_tab[3], sim.simx_opmode_blocking)[1]
            m = map_set(m, pos_x, pos_y, detect1, detect2, detect3, detect4, cur_angle[2])

        if disAmount > 0.49:
            disAmount = 0


def calc_value(m, x_pos, y_pos, i):
    m[x_pos][y_pos].value = i
    i += 1
    if not m[x_pos][y_pos].left:
        if m[x_pos][y_pos - 1].value > i:
            m = calc_value(m, x_pos, y_pos - 1, i)
    if not m[x_pos][y_pos].up:
        if m[x_pos - 1][y_pos].value > i:
            m = calc_value(m, x_pos - 1, y_pos, i)
    if not m[x_pos][y_pos].right:
        if m[x_pos][y_pos + 1].value > i:
            m = calc_value(m, x_pos, y_pos + 1, i)
    if not m[x_pos][y_pos].down:
        if m[x_pos + 1][y_pos].value > i:
            m = calc_value(m, x_pos + 1, y_pos, i)
    return m


def clear_values(lab_map,x,y):
    for i in range(x):
        for j in range(y):
            lab_map[i][j].value = 999
    return lab_map


def scan_walls(l_m, i_it, j_it):
    print("up")
    for i in range(i_it):
        for j in range(j_it):
            if l_m[i][j].up:
                print("+", "\t", end=" "),
            else:
                if not l_m[i][j].up:
                    print("-", "\t", end=" "),
        print("\n")
    print("down")
    for i in range(i_it):
        for j in range(j_it):
            if l_m[i][j].down:
                print("+", "\t", end=" "),
            else:
                print("-", "\t", end=" "),
        print("\n")
    print("left")
    for i in range(i_it):
        for j in range(j_it):
            if l_m[i][j].left:
                print("+", "\t", end=" "),
            else:
                print("-", "\t", end=" "),
        print("\n")
    print("right")
    for i in range(i_it):
        for j in range(j_it):
            if l_m[i][j].right:
                print("+", "\t", end=" "),
            else:
                print("-", "\t", end=" "),
        print("\n")
    return


def save_map(file, i_it, j_it):
    f = open(file, "w")
    for i in range(i_it):
        for j in range(j_it):
            if lab_map[i][j].up:
                f.write("True\n")
            else:
                f.write("False\n")
            if lab_map[i][j].right:
                f.write("True\n")
            else:
                f.write("False\n")
            if lab_map[i][j].down:
                f.write("True\n")
            else:
                f.write("False\n")
            if lab_map[i][j].left:
                f.write("True\n")
            else:
                f.write("False\n")
        print("\n")
    f.close()
    pass


def show_value_map(lab_map, i_it, j_it):
    for i in range(i_it):
        for j in range(j_it):
            if lab_map[i][j].value < 10:
                print(lab_map[i][j].value, "  ", end=" "),
            else:
                print(lab_map[i][j].value, " ", end=" "),
        print("\n")


def create_way(lab_map, pos_x, pos_y, droga):
    while lab_map[pos_x][pos_y].value > 0:
        if not lab_map[pos_x][pos_y].up:
            if lab_map[pos_x - 1][pos_y].value < lab_map[pos_x][pos_y].value:
                pos_x = pos_x - 1
                droga.append("up")
                continue
        if not lab_map[pos_x][pos_y].left:
            if lab_map[pos_x][pos_y - 1].value < lab_map[pos_x][pos_y].value:
                pos_y = pos_y - 1
                droga.append("left")
                continue
        if not lab_map[pos_x][pos_y].right:
            if lab_map[pos_x][pos_y + 1].value < lab_map[pos_x][pos_y].value:
                pos_y = pos_y + 1
                droga.append("right")
                continue
        if not lab_map[pos_x][pos_y].down:
            if lab_map[pos_x + 1][pos_y].value < lab_map[pos_x][pos_y].value:
                pos_x = pos_x + 1
                droga.append("back")
                continue
    return droga


def load_map(file, lab_map,x,y):
    file1 = open(file, 'r')
    for i in range(x):
        for j in range(y):
            line = file1.readline()
            lab_map[i][j].value = 999
            if line == "True\n":
                lab_map[i][j].up = True
            else:
                lab_map[i][j].up = False
            line = file1.readline()
            if line == "True\n":
                lab_map[i][j].right = True
            else:
                lab_map[i][j].right = False
            line = file1.readline()
            if line == "True\n":
                lab_map[i][j].down = True
            else:
                lab_map[i][j].down = False
            line = file1.readline()
            if line == "True\n":
                lab_map[i][j].left = True
            else:
                lab_map[i][j].left = False
    file1.close()
    return lab_map



if __name__ == '__main__':
    print('Program started')
    sim.simxFinish(-1)  # just in case, close all opened connections
    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)  # Connect to V-REP
    x_size = 7
    y_size = 8
    lab_map = [[blok() for i in range(y_size)] for j in range(x_size)]


    if clientID != -1:
        str_vel = 8
        [err, handle1] = sim.simxGetObjectHandle(clientID, "Plane", sim.simx_opmode_blocking)
        sensor_front = sim.simxGetObjectHandle(clientID, "sens_front", sim.simx_opmode_blocking)[1]
        sensor_left = sim.simxGetObjectHandle(clientID, "sens_left", sim.simx_opmode_blocking)[1]
        sensor_right = sim.simxGetObjectHandle(clientID, "sens_right", sim.simx_opmode_blocking)[1]
        sensor_back = sim.simxGetObjectHandle(clientID, "sens_back", sim.simx_opmode_blocking)[1]
        sensors = [sensor_left, sensor_front, sensor_right, sensor_back]

        ld = sim.simxGetObjectHandle(clientID, "left_down_joint", sim.simx_opmode_blocking)[1]
        lu = sim.simxGetObjectHandle(clientID, "left_up_joint", sim.simx_opmode_blocking)[1]
        rd = sim.simxGetObjectHandle(clientID, "right_down_joint", sim.simx_opmode_blocking)[1]
        ru = sim.simxGetObjectHandle(clientID, "right_up_joint", sim.simx_opmode_blocking)[1]
        l_joints = [lu, ld]
        r_joints = [ru, rd]
        vel_amount(clientID, l_joints, r_joints, 0)

        sim.simxStartSimulation(clientID, sim.simx_opmode_blocking)
        detect_left = sim.simxReadProximitySensor(clientID, sensor_left, sim.simx_opmode_blocking)[1]
        detect_front = sim.simxReadProximitySensor(clientID, sensor_front, sim.simx_opmode_blocking)[1]
        detect_right = sim.simxReadProximitySensor(clientID, sensor_right, sim.simx_opmode_blocking)[1]
        detect_back = sim.simxReadProximitySensor(clientID, sensor_back, sim.simx_opmode_blocking)[1]
        angle = sim.simxGetObjectOrientation(clientID, handle1, -1, sim.simx_opmode_blocking)[1]
        pos = sim.simxGetObjectPosition(clientID, handle1, -1, sim.simx_opmode_blocking)[1]
        pos_y = math.trunc((pos[1] + 2) / 0.5)
        pos_x = math.trunc((pos[0] + 2) / 0.5)
        option = 0
        lab_map = map_set(lab_map, pos_x, pos_y, detect_left, detect_front, detect_right, detect_back, angle[2])
        while option not in range(1, 4):
            option = int(input(
                "Wybierz tryb tworzenia mapy:\n1. Skanowanie przez pojazd\n2. Wczytanie z pliku\n3. Koniec programu\n>"))
        if option == 1:
            if not detect_left:
                turn(clientID, handle1, l_joints, r_joints, "left", 4)
            else:
                if not detect_front:
                    "go_straight"
                else:
                    if not detect_right:
                        turn(clientID, handle1, l_joints, r_joints, "right", 4)
                    else:
                        turn(clientID, handle1, l_joints, r_joints, "back", 4)
            while 1:
                [sens, lab_map, x, y] = go_str(clientID, handle1, l_joints, r_joints, str_vel, sensors, lab_map)
                print("(", x, ",", y, ")")
                angle = sim.simxGetObjectOrientation(clientID, handle1, -1, sim.simx_opmode_blocking)[1]
                lab_map = map_set(lab_map, x, y, sens[0], sens[1], sens[2], sens[3], angle[2])
                if not sens[0]:
                    turn(clientID, handle1, l_joints, r_joints, "left", 5)
                else:
                    if not sens[1]:
                        "go_straight"
                    else:
                        if not sens[2]:
                            turn(clientID, handle1, l_joints, r_joints, "right", 5)
                        else:
                            turn(clientID, handle1, l_joints, r_joints, "back", 5)
                scan_walls(lab_map, x_size, y_size)
                if check(lab_map,x_size,y_size):
                    break
            save_map("mapa.txt", x_size, y_size)
        if option == 2:
            lab_map = load_map("mapa.txt", lab_map,x_size,y_size)
        while (option != 3):
            option = int(input("Wybierz opcje:\n1. Losuj punkt.\n2. Wybierz punkt\n3. Koniec działania programu.\n>"))
            if option >= 3:
                continue
            if option == 1:
                x = random.randint(0, x_size-1)
                y = random.randint(0, y_size-1)
            if option == 2:
                x = int(input("Podaj x: "))
                y = int(input("Podaj y: "))
            print(x, " ", y)
            lab_map = calc_value(clear_values(lab_map,x_size,y_size), x, y, 0)
            show_value_map(lab_map, x_size, y_size)
            pos = sim.simxGetObjectPosition(clientID, handle1, -1, sim.simx_opmode_blocking)[1]
            pos_y = math.trunc((pos[1] + 2) / 0.5)
            pos_x = math.trunc((pos[0] + 2) / 0.5)
            droga = []
            droga = create_way(lab_map, pos_x, pos_y, droga)
            print(droga)
            if not droga:
                continue

            check = droga[0]
            counter = 1
            new_droga = []
            up_dir = ["up", "right", "back", "left"]
            right_dir = ["left", "up", "right", "back"]
            down_dir = ["back", "left", "up", "right"]
            left_dir = ["right", "back", "left", "up"]
            it = iter(droga)
            next(it)
            for i in it:
                if i == check:
                    counter = counter + 1
                else:
                    new_droga.append(check)
                    new_droga.append(counter)
                    counter = 1
                    check = i
            new_droga.append(check)
            new_droga.append(counter)
            print(new_droga)
            angle = sim.simxGetObjectOrientation(clientID, handle1, -1, sim.simx_opmode_blocking)[1]
            direct = directory_check(angle[2])
            droga.clear()
            for i in range(0, len(new_droga), 2):
                x = up_dir.index(new_droga[i])
                if direct == "up":
                    droga.append(up_dir[x])
                if direct == "right":
                    droga.append(right_dir[x])
                if direct == "back":
                    droga.append(down_dir[x])
                if direct == "left":
                    droga.append(left_dir[x])
                droga.append(new_droga[i + 1])
                direct = new_droga[i]

            print(droga)

            for i in range(0, len(droga), 2):
                turn(clientID, handle1, l_joints, r_joints, droga[i], 3)
                if droga[i + 1] > 2:
                    go_str2(clientID, handle1, l_joints, r_joints, 60, droga[i + 1])
                else:
                    go_str2(clientID, handle1, l_joints, r_joints, 30, droga[i + 1])

        sim.simxStopSimulation(clientID, sim.simx_opmode_blocking)
        sim.simxFinish(clientID)

    else:
        print('Failed connecting to remote API server')

    print('Program ended')
