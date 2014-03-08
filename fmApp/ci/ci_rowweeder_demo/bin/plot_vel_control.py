#!/usr/bin/env python

import rosbag
import argparse
import matplotlib.pyplot as plt

if __name__=="__main__":
    parser = argparse.ArgumentParser(description='Plot recorded velocity')
    parser.add_argument('bag', metavar='BAG', type=str, help='the bag to process')

    args = parser.parse_args()
    
    fwl = []
    fwlt = []
    fbl = []
    fblt = []
    
    fwr = []
    fwrt = []
    fbr = []
    fbrt = []
    
    tp = ['/fmData/velocity_left','/fmSignals/cmd_vel_left',
          '/fmData/velocity_right','/fmSignals/cmd_vel_right',]
    
    with rosbag.Bag(args.bag) as b:
        for topic, msg, t in b.read_messages(topics=tp):
            if topic == '/fmData/velocity_left':
                fbl.append(float(msg.data))
                fblt.append(t.to_sec())
            if topic == '/fmSignals/cmd_vel_left':
                fwl.append(float(msg.twist.linear.x))
                fwlt.append(t.to_sec())
            if topic == '/fmData/velocity_right':
                fbr.append(float(msg.data))
                fbrt.append(t.to_sec())
            if topic == '/fmSignals/cmd_vel_right':
                fwr.append(float(msg.twist.linear.x))
                fwrt.append(t.to_sec())    
    print len(fbl),len(fwl)
    
    plt.subplot(2,1,1)
    plt.plot(fwlt,fwl,fblt,fbl)
    plt.legend(['cmd_l','vel_l'])
    plt.subplot(2,1,2)
    plt.plot(fwrt,fwr,fbrt,fbr)
    plt.legend(['cmd_r','vel_r'])
    plt.show()
    

