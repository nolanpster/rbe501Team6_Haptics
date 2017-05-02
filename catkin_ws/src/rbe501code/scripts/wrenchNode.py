#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import WrenchStamped



def wrenchSweep(wrenchPub):
    wrenchMes = WrenchStamped()

    for ang in np.linspace(0,np.pi*2,10000):
        rospy.sleep(0.002)
        xComp = np.sin(ang)
        zComp = np.cos(ang)
        wrenchMes.wrench.force.x=xComp
        wrenchMes.wrench.force.z=zComp
        wrenchPub.publish(wrenchMes)
    wrenchMes.wrench.force.x=0
    wrenchMes.wrench.force.z=0
    wrenchPub.publish(wrenchMes)

#Main handler of the project
def run():
    


    rospy.init_node('rbe501WrenchPub')

    wrenchPub = rospy.Publisher("/dvrk_psm/haptics_feedback_force", WrenchStamped, queue_size=1) # you can use other types if desired


    while (1 and not rospy.is_shutdown()):
        #publishCells(mapData) #publishing map data every 2 seconds
        print("Sweeping")
        rospy.sleep(1)
        wrenchSweep(wrenchPub)
        rospy.sleep(1)  



if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
