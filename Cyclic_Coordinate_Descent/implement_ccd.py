from ccd import *
import robot
import time

def main():

    baxter = robot.Baxter()

    ja_keys = ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']
    seed_angles = {}

    for keys in ja_keys:
        seed_angles[keys] = baxter.angle_limits[keys][0]

    # Seed angles for presentation
    seed_angles = {'s0':-80, 's1':0, 'e0':0, 'e1':0, 'w0':0, 'w1':-50, 'w2':0}

    print ("Seed Angles", seed_angles)

    target_position = [-0.50, -0.40, 0.35]
    e_tol = 0.05

    ccd_object = CycCorDes(seed_angles, target_position, e_tol)

    start = time.time()
    final_angles = ccd_object.exec_ccd(True)
    print "Time taken :", time.time() - start

    print ("Final output Inverse Kinematics Solution", final_angles)


if __name__ == '__main__':
    main()