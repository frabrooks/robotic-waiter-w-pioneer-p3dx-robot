#!/usr/bin/env python

import rospy
import threading
import thread
import Queue
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped

from time import sleep
from std_msgs.msg import String

import socket

UDP_PORT = 1776
TCP_PORT = 1777
IP_ADDRESS = "54.213.135.86"
SLEEP_RATE = 50
JOB_TTL = 300

running = True

x = 0
y = 0
a = 0

jobDoneQueue = Queue.Queue()

def poseCallback(pose):

    global x
    x = pose.pose.pose.position.x
    global y
    y = pose.pose.pose.position.y
    global a
    o = pose.pose.pose.orientation
    quaternion = (o.x, o.y, o.z, o.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    a = euler[2] #(yaw)
    
    #rospy.loginfo("read in x: " + str(x))
    #rospy.loginfo("read in y: " + str(y))
    #rospy.loginfo("read in x: " + str(a))

def jobDoneCallback(job):

    global jobDoneQueue
    try:
        jobDoneQueue.put_nowait(job)
    except Queue.Full:
        print "Error jobDone queue full"
    

class Networker():

    def __init__(self):
        print "init Networker"
        
        self.receivedTCPs = []
        self.outgoingTCPs = Queue.Queue(maxsize=5)
        
        self.robotUDPSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.robotTCPSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        self.running = True

        rospy.init_node('networker', anonymous=True)
        
        self.pub_order_add = rospy.Publisher('job/add', String, queue_size=10)

        
        self.pub_order_cancel = rospy.Publisher('job/delete', String, queue_size=10)
        

        self.rate = rospy.Rate(SLEEP_RATE) # 10hz

        print "connecting to AWS"
        self.robotTCPSock.connect((IP_ADDRESS, TCP_PORT))
        self.robotTCPSock.send("id:robot".encode() + "\n")

        thread.start_new_thread(network_listener, (self,))

        #init subscribers
        rospy.Subscriber("amcl_pose",PoseWithCovarianceStamped, poseCallback)
        rospy.Subscriber("job/done", String, jobDoneCallback)
        
        self.start()

    def start(self):
        global x
        global y
        global a
        global jobDoneQueue
        
        print "main loop started"
        
        while (not rospy.is_shutdown()):
            #print "sending UDP data"

            #get pose and send udp
            hello_str = "~robot>>I am at x:%sy:%sa:%s \n" % (x,y,a)
            #rospy.loginfo(hello_str)
            try:
                self.robotUDPSock.sendto(hello_str, (IP_ADDRESS, UDP_PORT))
            except:
                print "\nctrl-c OR some exception (1)"
                break
            self.rate.sleep()

            #read and process tcp messages

            if not self.receivedTCPs == []:
                try:
                    
                    print "\n::::::printing jobs:::::::\n"
                    for tup in self.receivedTCPs:
                        print tup[0] + " TTL:" + str(tup[1])
                        if (tup[1]-SLEEP_RATE)<=0:
                            print tup[0] + "job cancelled>telling jobbrain"
                            self.pub_order_cancel.publish(tup[0].split("|")[2])
                            self.receivedTCPs.remove(tup)
                        else:
                            self.receivedTCPs.remove(tup)
                            self.receivedTCPs.append((tup[0],tup[1]-1))
                    print "\n::::::done printng jobs:::::\n"

                    st = jobDoneQueue.get_nowait()
                    for tup in self.reveivedTCPs:
                        if tup[0] == st:
                            print "job: " + st + "is done"
                            self.receivedTCPs.remove(tup)

                    
                except Queue.Empty:
                    pass
                except:
                    print "\nctrl-c read OR some exception (2)"
                    break
            
        self.running = False

        print "exiting\n"
        
        self.robotTCPSock.shutdown(socket.SHUT_RDWR)
        
        self.robotTCPSock.close()
        self.robotUDPSock.close()

        #give threads chance to close
        sleep(0.1)
        
        print "\nsockets closed succesfully\n"



def network_listener(networker):
    print "listening"
    
    while networker.running:

        try:
            data = ""
            while True:
                part = networker.robotTCPSock.recv(1)
                if not part:
                    print "\nno data: socket closed\n"
                    break
                if part != "\n":
                    data+=part
                elif part == "\n":
                    break
            if data == "":
                break
            newOrder = True
            for d in networker.receivedTCPs:
                if d[0] == data.rstrip():
                    newOrder = False
                    networker.receivedTCPs.remove(d)
                    networker.receivedTCPs.append((data.rstrip(), JOB_TTL))

            print "loopin"
            
            if newOrder:
                print "new order received:"
                print(data)
                networker.receivedTCPs.append((data.rstrip(),JOB_TTL))
                networker.pub_order_add.publish(data.rstrip())
        except IOError:
            print "error, incoming message queue is full"

    print "listener exiting, shutting down ros node\n"

    rospy.signal_shutdown("Network Unavailable")
    

if __name__ == '__main__':
    n = Networker()
    
