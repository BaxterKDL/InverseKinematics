from ccd import *
import robot

def main():

    baxter = robot.Baxter()

    ja_keys = ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']
    seed_angles = {}

    for keys in ja_keys:
        seed_angles[keys] = baxter.angle_limits[keys][0]

    print ("Seed Angles", seed_angles)

    target_position = [0.10, 0.50, -0.30]
    e_tol = 0.01

    ccd_object = CycCorDes(seed_angles, target_position, e_tol)

    final_angles = ccd_object.exec_ccd()

    print ("Final output Inverse Kinematics Solution", final_angles)


if __name__ == '__main__':
    main()