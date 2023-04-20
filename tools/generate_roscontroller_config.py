import argparse
import numpy

try:
    import cnoid.Body
    import cnoid.Util
except ImportError:
    import sys 
    sys.path.append('/choreonoid_ws/install/lib/choreonoid-1.8/python')
    import cnoid.Body
    import cnoid.Util



if __name__=='__main__':
    parser = argparse.ArgumentParser(
            prog='cnoid_dump_model.py', # プログラム名
            usage='Demonstration of cnoid_dump_model', # プログラムの利用方法

            add_help=True, # -h/–help オプションの追加
            )
    parser.add_argument('--bodyfile', type=str, default="robotname.body")
    parser.add_argument('--controllername', type=str, default="joint_controler")
    parser.add_argument('--jointsuffix', type=str, default="")

    args = parser.parse_args()
    fname = args.bodyfile
    controllername = args.controllername
    joint_suffix = args.jointsuffix

    rbody = cnoid.Body.BodyLoader().load(str(fname))
    rbody.updateLinkTree()
    rbody.initializePosition()
    rbody.calcForwardKinematics()

    rotate_joint_list = []

    num_link = rbody.getNumLinks()
    num_joint = rbody.getNumJoints()

    for idx in range(num_link):
        lk = rbody.getLink(idx)
        if lk.isRoot():
            continue
        p = lk.getParent()
        if p:
            if lk.isRevoluteJoint():
                rotate_joint_list.append(lk.getName())
    
    if len(rotate_joint_list)>0:
        print('%s:'%rbody.getModelName())
        print('  %s:'%controllername)
        print('    type: "position_controllers/JointTrajectoryController"')
        print("    joints:")
        for joint in rotate_joint_list:
            print('      - %s%s'%(joint, joint_suffix))
        print("    gains:")
        for joint in rotate_joint_list:
            print('      %s%s:'%(joint, joint_suffix))
            print('        p: 100')
            print('        i: 10')
            print('        d: 1')
