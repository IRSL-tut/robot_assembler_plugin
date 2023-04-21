import argparse
import numpy

try:
    import cnoid.Body
    import cnoid.Util
except ImportError:
    import sys 
    import shutil
    import os
    choreonoid_path = os.path.join(os.path.dirname(shutil.which('choreonoid')), '../lib/choreonoid-1.8/python') if shutil.which('choreonoid') is not None else None
    if choreonoid_path is None:
        print('Error: choreonoid_path not found.', file=sys.stderr)
        sys.exit(1)
    sys.path.append(choreonoid_path)
    import cnoid.Body
    import cnoid.Util



if __name__=='__main__':
    parser = argparse.ArgumentParser(
            prog='cnoid_dump_model.py', # プログラム名
            usage='Demonstration of cnoid_dump_model', # プログラムの利用方法

            add_help=True, # -h/–help オプションの追加
            )
    parser.add_argument('--bodyfile', type=str, default="robotname.body")
    parser.add_argument('--controllername', type=str, default="joint_controller")
    parser.add_argument('--robotname', type=str, default="")
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

    for idx in range(num_joint):
        joint = rbody.getJoint(idx)
        rotate_joint_list.append(joint.getName())
    
    robotname = args.robotname if args.robotname != "" else rbody.getModelName()
    if len(rotate_joint_list)>0:
        print('%s:'%robotname)
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
