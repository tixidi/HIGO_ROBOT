#!/usr/bin/env python
#coding=utf-8


# Python libs
import sys, time

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

# OpenCV
import cv2
import cv2.cv as cv
# Ros libraries
import roslib
import rospy
from std_msgs.msg import String
# Ros Messages
from sensor_msgs.msg import CompressedImage,Image, RegionOfInterest, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

import string,cgi,time
from os import curdir, sep
from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer
from SocketServer import ThreadingMixIn
import cv
import re
from threading import Thread, Lock

VERBOSE=True

import copy

mutex = Lock()
topic_name = "/camera/image/image_raw/left"
class web_video_server:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # subscribed Topic
        # Initialize a number of global variables
        self.frame = None
        self.frame_size = None
        self.frame_width = None
        self.frame_height = None
        self.depth_image = None
        self.marker_image = None
        self.display_image = None
        # Create the cv_bridge object
        self.bridge = CvBridge()

        self.subscriber = rospy.Subscriber(topic_name,
            Image, self.callback,  queue_size = 1)
        if VERBOSE :
            print "subscribed to " + topic_name

    def getCompressedImage(self):
        mutex.acquire(1)
        result = copy.deepcopy(self.np_arr);
        mutex.release()
        return result;

    def callback(self, ros_data):
        '''Callback function of subscribed topic. '''
     #   mutex.acquire(1)
        # Store the image header in a global variable
        self.image_header = ros_data.header
        frame= self.convert_image(ros_data)

        
        # Store the frame width and height in a pair of global variables
        if self.frame_width is None:
            self.frame_size = (frame.shape[1], frame.shape[0])
            self.frame_width, self.frame_height = self.frame_size            
            
        # Create the marker image we will use for display purposes
        if self.marker_image is None:
            self.marker_image = np.zeros_like(frame)
            
        # Copy the current frame to the global image in case we need it elsewhere
        self.frame = frame.copy()

        cv2.imshow("aa", self.frame)        
        cv2.imwrite("/home/ros/gohi_ws/src/HIGO_ROBOT/robot_blockly/scripts/123.jpg",frame)
        #time.sleep(0.05)
        # Update the image display

        
        
        # Process any keyboard commands
        self.keystroke = cv2.waitKey(30)
 	#mutex.release()

    def convert_image(self,ros_image):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")       
            return np.array(cv_image, dtype=np.uint8)
        except CvBridgeError, e:
            print e

class MyHandler(BaseHTTPRequestHandler):
    def do_GET(self):

            #self.path=re.sub('[^.a-zA-Z0-9]', "",str(self.path))
            #if self.path=="" or self.path==None or self.path[:1]==".":
            #    return
            #if self.path.endswith(".html"):
            #    f = open(curdir + sep + self.path)
            #    self.send_response(200)
            #    self.send_header('Content-type',    'text/html')
            #    self.end_headers()
            #    self.wfile.write(f.read())
            #    f.close()
            #    return
            #if self.path.endswith(".mjpeg"):
            #    self.send_response(200)
            #    self.wfile.write("Content-Type: multipart/x-mixed-replace; boundary=--aaboundary")
            #    self.wfile.write("\r\n\r\n")
            #    while 1:
            #      JpegData=self.ic.getCompressedImage();                    
            #      self.wfile.write("--aaboundary\r\n")
            #      self.wfile.write("Content-Type: image/jpeg\r\n")
            #      self.wfile.write("Content-length: "+str(len(JpegData))+"\r\n\r\n" )
            #      self.wfile.write(JpegData)
            #      self.wfile.write("\r\n\r\n\r\n")    
            #      time.sleep(0.05)
            #    return
            
            
            #if self.path.endswith("*.jpg"):
            #self.send_response(200)
   
            #self.wfile.write("Content-Type: multipart/x-mixed-replace; boundary=--aaboundary")
            #self.wfile.write("\r\n\r\n")
            # JpegData=self.ic.getCompressedImage();
	    #f=open("./11.jpg","rb")
            #JpegData=f.read()
            #f.close()

            #self.wfile.write("--aaboundary\r\n")
            #self.wfile.write("Content-Type: image/jpeg\r\n")
            #self.wfile.write("Content-length: "+str(len(JpegData))+"\r\n\r\n" )
            #self.wfile.write(JpegData)
            #self.wfile.write("\r\n\r\n\r\n")
        enc="UTF-8" 
        self.send_response(200)           #发送200状态码，表示处理正常
        self.send_header("Content-type", "text/html; charset=%s" % enc)   #发送html头，这里可说明文件类型和字符集等信息
        f = open("/home/ros/gohi_ws/src/HIGO_ROBOT/robot_blockly/scripts/123.jpg","r")       #只读打开一个文件
        strs = f.read()                          #读出文件
        self.send_header("Content-Length", str(len(strs)))    #发送html头   说明文件长度 注意，这里如果长度和实际长度不一致的话，后面客户端处理时就会触发IncompleteRead 这个异常。
        self.end_headers()                #html头部分结束
        self.wfile.write(strs)            #以刚才读出的那个文件的内容作为后续内容发出给http客户端



  
class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """Handle requests in a separate thread."""

def main_server(args, ic):
    try:
        MyHandler.ic = ic
        server = ThreadedHTTPServer(('', 8080), MyHandler)
        print 'started httpserver...'
        server.serve_forever()
    except KeyboardInterrupt:
        print '^C received, shutting down server'
        server.socket.close()  


def main_ros(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('web_video_server', anonymous=False)
    print 'started ROS...'
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    ic = web_video_server()

    t = Thread(target=main_server, args=(sys.argv, ic))
    t.start()
    main_ros(sys.argv)
