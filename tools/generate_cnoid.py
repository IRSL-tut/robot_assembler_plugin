import argparse
import pathlib
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
            prog='generage cnoid file', # プログラム名
            usage='Generate cnoid file for choreonoidros', # プログラムの利用方法
            add_help=True, # -h/–help オプションの追加
            )
    parser.add_argument('--bodyfile', type=str, default="robotname.body")
    parser.add_argument('--templatefile', type=str, default="choreonoid_ros_template.cnoid")
    parser.add_argument('--offsetx', type=float, default="0.0")
    parser.add_argument('--offsety', type=float, default="0.0")
    parser.add_argument('--offsetz', type=float, default="0.0")
    args = parser.parse_args()


    rbody = cnoid.Body.BodyLoader().load(str(args.bodyfile))
    rbody.updateLinkTree()
    rbody.initializePosition()
    rbody.calcForwardKinematics()


    template_filename = args.templatefile
    p = pathlib.Path(args.bodyfile)
    bodyfile_path =  str(p.resolve())
    robotname = rbody.getModelName()
    offset_x = args.offsetx
    offset_y = args.offsety
    offset_z = args.offsetz

    with open(template_filename, 'r') as f:
        lines = f.readlines()

    replace_mapping = [ ('\r',''),
                        ('\n',''),
                        ('__ROBOT__NAME__', robotname),
                        ('__BODYFILE_PATH__', bodyfile_path),
                        ('__OFFSET_X__', str(offset_x)),
                        ('__OFFSET_Y__', str(offset_y)),
                        ('__OFFSET_Z__', str(offset_z)),
                        ]

    for line in lines:
        for maping in replace_mapping:
            line = line.replace(maping[0], maping[1])
        
        print(line)