#!/usr/bin/env python


import roslib
import rospy
import rospkg
import tf

from Tkinter import *
import tkFileDialog

import numpy as np

# Beginning of main program
SCALE=32

class calibrationGUI:

    x=0.0
    y=0.0
    z=0.0
    Rx=0.0
    Ry=0.0
    Rz=0.0


    rospy.init_node('calibration_gui')

    master = Tk()
    master.title('Gui controller')
    master.geometry('+%d+%d' % (24*SCALE, 18*SCALE))
    master.minsize(width=24*SCALE, height=18*SCALE)
    master.maxsize(width=24*SCALE, height=18*SCALE)

    def __init__(self):
	print "=============================================="
	self.camera_frame=rospy.get_param("~frame1")
	self.world_frame=rospy.get_param("~frame2")
	print "self.camera_frame",self.camera_frame
	print "self.world_frame",self.world_frame
        l=900
        Lower_limit=-3.0
        Upper_limit=3.0
        self.xslider = Scale(self.master, from_=Lower_limit, to=Upper_limit,tickinterval=0.5,resolution=0.001,length=l,label="x",orient=HORIZONTAL,command=self.slider_event,sliderlength=10,troughcolor="DarkOrange1")
        self.xslider.pack()
        self.yslider = Scale(self.master, from_=Lower_limit, to=Upper_limit,tickinterval=0.5,resolution=0.001,length=l,label="y",orient=HORIZONTAL,command=self.slider_event,sliderlength=10,troughcolor="DarkOrange1")
        self.yslider.pack()
        self.zslider = Scale(self.master, from_=Lower_limit, to=Upper_limit,tickinterval=0.5,resolution=0.001,length=l,label="z",orient=HORIZONTAL,command=self.slider_event,sliderlength=10,troughcolor="DarkOrange1")
        self.zslider.pack()
        self.Rxslider = Scale(self.master, from_=-np.pi, to=np.pi,tickinterval=np.pi/4,resolution=0.001,length=l,label="Rx",orient=HORIZONTAL,command=self.slider_event,sliderlength=10,troughcolor="DarkOrange3")
        self.Rxslider.pack()
        self.Ryslider = Scale(self.master, from_=-np.pi, to=np.pi,tickinterval=np.pi/4,resolution=0.001,length=l,label="Ry",orient=HORIZONTAL,command=self.slider_event,sliderlength=10,troughcolor="DarkOrange3")
        self.Ryslider.pack()
        self.Rzslider = Scale(self.master, from_=-np.pi, to=np.pi,tickinterval=np.pi/4,resolution=0.001,length=l,label="Rz",orient=HORIZONTAL,command=self.slider_event,sliderlength=10,troughcolor="DarkOrange3")
        self.Rzslider.pack()
        self.refreshCloud=Button(text="Refresh_cloud",command=self.refresh_cloud)
        self.refreshCloud.pack()
        self.filesave=Button(text="save_transform",command=self.file_save,bg="cyan",activebackground="cyan")
        self.filesave.pack()
        self.filesave2=Button(text="save_transform_Rz_Ry_Rx",command=self.file_save2,bg="orange",activebackground="cyan")
        self.filesave2.pack()
        self.quitbutton=Button(text="Quit",command=self.exit_gui,bg="red",activebackground="red")
        self.quitbutton.pack()

        self.transform_broadcaster = tf.TransformBroadcaster()


    def start(self):
        self.master.mainloop()


    def slider_event(self,scale):
            try:
                self.x=self.xslider.get()
                self.y=self.yslider.get()
                self.z=self.zslider.get()
                self.Rx=self.Rxslider.get()
                self.Ry=self.Ryslider.get()
                self.Rz=self.Rzslider.get()
                self.transform_broadcaster.sendTransform((self.x,self.y,self.z),tf.transformations.quaternion_from_euler(self.Rx,self.Ry,self.Rz),rospy.Time.now(),self.camera_frame,self.world_frame)
            except rospy.ServiceException:
                    rospy.logwarn('Got an exception while using service')

    def refresh_cloud(self):
        for i in range(2):
            self.transform_broadcaster.sendTransform((self.x,self.y,self.z),tf.transformations.quaternion_from_euler(self.Rx,self.Ry,self.Rz),rospy.Time.now(),self.camera_frame,self.world_frame)
            rospy.sleep(1.0)

    def file_save(self):
	rospack = rospkg.RosPack()
	path=rospack.get_path('camera_calibration_man')+'/config/'
        Q=tf.transformations.quaternion_from_euler(self.Rx,self.Ry,self.Rz)
        #Out="world_frame" + "   " + "camera_frame" + "   "+"Time",  "x" +  "y" +  "z" +   "Qx" +  "Qy" +  "Qz" +  "Qw\n"
        Out=self.world_frame + "   " + self.camera_frame + "   "+ str(rospy.Time.now())+",  "+str(self.x)+"   "+str(self.y)+"   "+str(self.z)+"   "+str(Q[0])+"   "+str(Q[1])+"   "+str(Q[2])+"   "+str(Q[3])+"\n"
        #Out=str(rospy.Time.now())+",  "+str(self.x)+"   "+str(self.y)+"   "+str(self.z)+"   "+str(self.Rx)+"   "+str(self.Ry)+"   "+str(self.Rz)+"\n"
        f2 = tkFileDialog.asksaveasfile(mode='a', defaultextension="",initialdir=path)
        if f2 is None: # asksaveasfile return `None` if dialog closed with "cancel".
            return
        f2.write(Out)
        f2.close() # `()` was missing.

    def file_save2(self):
	rospack = rospkg.RosPack()
	path=rospack.get_path('camera_calibration_man')+'/config/'
        Out=self.world_frame + "   " + str(self.camera_frame) + "   "+ str(rospy.Time.now())+",  "+str(self.x)+"   "+str(self.y)+"   "+str(self.z)+"   "+str(self.Rz)+"   "+str(self.Ry)+"   "+str(self.Rx)+"\n"

        f2 = tkFileDialog.asksaveasfile(mode='a', defaultextension="",initialdir=path)
        if f2 is None: # asksaveasfile return `None` if dialog closed with "cancel".
            return
        f2.write(Out)
        f2.close() # `()` was missing.

    def exit_gui(self):
       self.master.quit()



if __name__ == '__main__':
    print "=============================================="
    gui=calibrationGUI()
    gui.start()
