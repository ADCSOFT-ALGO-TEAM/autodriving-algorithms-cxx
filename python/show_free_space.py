'''
@brief show driver free space in 3D
@author pengcheng(yslrpch@126.com)
@Date 2020-03-13
'''

import glob
import os.path as osp
import cv2

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

def main(result_path = None):
    assert result_path is not None
    paths = osp.join(result_path, "*.txt")
    space_files = glob.glob(paths)
    
    color = ['g', 'b', 'y', 'b']
    for sf in space_files:
        x = []
        y = []
        z = []
        s_file = osp.split(sf)[1]
        img_path = osp.join(result_path ,osp.splitext(s_file)[0]+".png")
        img = cv2.imread(img_path)
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        with  open(sf) as f:
            lines = f.readlines()

            for i in range(len(lines)):
                lines[i] = lines[i].strip()
            s = []
            for i in range(len(lines)):
                if 'space:' == lines[i]:
                    s.append(i)
            s.append(len(lines))
            print(s)
            for step in range(len(s) - 1) :
                space = lines[(s[step]) +1:s[step + 1]]
                zs = []
                xs = []
                ys = []
                for area in space:
                    vals = area.split(' , ')
                    xs.append(float(vals[0]))
                    ys.append(float(vals[2]))
                    zs.append(float(-0.12))
                x.append(xs)
                y.append(ys)
                z.append(zs)
        cv2.imshow('color', img)
        cv2.waitKey(0)
        for i in range(len(x)):
            ax.plot(x[i], y[i],z[i], c = color[i])
        plt.show()
        
                    
            # for line in lines:
            #     line = line.strip()
            #     if 'space:' == line and len(xs) != 0:
            #         x.append(xs)
            #         y.append(ys)
            #         z.append(zs)
            #     if 'space:' == line:
            #         continue
            #     s = line.split(' , ')
            #     xs.append(float(s[0]))
            #     ys.append(float(s[2]))
            #     zs.append(float(-1.02))
        # print(len(x))
        

if __name__ == "__main__":
    res_path = "/home/pengccheng/Data/work/ADC/autodriving-algorithms-cxx/result"
    main(res_path)