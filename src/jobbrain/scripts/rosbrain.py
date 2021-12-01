#! /usr/bin/env python

import rospy
import sys
from Queue import Queue
from job import Job, Status
from geometry_msgs.msg import Pose, PoseStamped, Point
from std_msgs.msg import String, Bool
from nav_msgs.msg import GridCells
from multiprocessing import Process
from geometry_msgs.msg import PoseWithCovarianceStamped


robot_x = 0
robot_y = 0

### Global stuff (puke)
jobQueue = Queue()
locLookup = {
    'bar':(15,21),
    'table_1':(2,3),
    'table_2':(15,1),
    'table_3':(5,25),
    'table_4':(20,22),
    'table_5':(11,19),
    'table_6':(10,12),
    'table_7':(10,7)
}
currentJob = Job('','','')
deletedJobs = []

def deleteJob(jobid):
    deletedJobs.append(jobid.data)

    print "cancelling job: " + jobid.data
    
    if currentJob.jobid == jobid.data:
        print "current job cancelled"
        currentJob.status = Status.deleted

def tableCostMap(pub):
    costmap = GridCells()
    costmap.header.stamp = rospy.Time.now()
    costmap.header.frame_id = 'map'
    costmap.cell_width = 0.05
    costmap.cell_height = 0.05

    for k,v in locLookup.items():
        p = Point()
        p.x = v[0]
        p.y = v[1]
        costmap.cells.append(p)

    pub.publish(costmap)

def currentJobPub(pub):
    pub.publish(str(currentJob))


def qrReadCallback(qr_code):
    if qr_code.data == currentJob.jobid:
        currentJob.set_status(Status.completed)
        rospy.loginfo('qr code read ' + qr_code.data + ' matched, id job completed')


def toMapLookup(job):

    if job.status != Status.to_location and job.status != Status.conformation_wait:
        xy = locLookup['bar']
    else:
        xy = locLookup[job.goal]

    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = 'map'
    pose.pose.position.x = xy[0]
    pose.pose.position.y = xy[1]
    pose.pose.position.z = 0 # Arbitrary rotation
    return pose

def publishGoal(pub):
    pub.publish(toMapLookup(currentJob))

def poseCallback(pose):

    global robot_x
    robot_x = pose.pose.pose.position.x
    global robot_y
    robot_y = pose.pose.pose.position.y

def jobStatusCallback(comp_bool):

    if currentJob.status != Status.to_location or currentJob.status != Status.conformation_wait:
        xy = locLookup['bar']
    else:
        xy = locLookup[job.goal]

    dx = xy[0] - robot_x
    dy = xy[1] - robot_y

    dist = dx * dx + dy * dy

    sqrRad = 1
    if dist > sqrRad:
        return
    
    if comp_bool.data == True:
        print(dist)
        if currentJob.status == Status.to_bar:
            currentJob.set_status(Status.to_location)
            rospy.loginfo('Arrived at bar, moving to table')
        elif currentJob.status == Status.to_location:
            currentJob.set_status(Status.conformation_wait)
            rospy.loginfo('Arrived at table, waiting for qr code')

# Publish location to go to and also status
def jobAddCallback(jobString):
    jinfo = jobString.data.split('|')
    job = Job(jinfo[0], jinfo[1], jinfo[2])
    rospy.loginfo('Job added: ' + jobString.data)
    jobQueue.put(job)

def main():

    rospy.Subscriber("amcl_pose",PoseWithCovarianceStamped, poseCallback)
    rospy.Subscriber('job/add', String, jobAddCallback)
    rospy.Subscriber('job/advance', Bool, jobStatusCallback)
    rospy.Subscriber('job/delete', String, deleteJob)
    rospy.Subscriber('qr_reader', String, qrReadCallback)

    job_done = rospy.Publisher('job/done', String, queue_size=1)

    pub_job = rospy.Publisher('job/current', String, queue_size=1)
   # pub_job_proc = Process(target=currentJobPub, args=(pub_job,))
   # pub_job_proc.start()

    pub_table = rospy.Publisher('job/tables', GridCells, queue_size=1)
   # pub_table_proc = Process(target=tableCostMap,args=(pub_table,))
   # pub_table_proc.start()

    pub_goal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
   # pub_goal_proc = Process(target=publishGoal,args=(pub_goal,))
   # pub_goal_proc.start()

    rospy.loginfo('Started Loop')

    # Main loop
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        global currentJob

        # Wait for queue
        while jobQueue.empty() and not rospy.is_shutdown():

            currentJobPub(pub_job)
            tableCostMap(pub_table)
            publishGoal(pub_goal)

            rate.sleep()

        currentJob = jobQueue.get()
        while (currentJob.jobid in deletedJobs) and (not rospy.is_shutdown()):

            while jobQueue.empty() and not rospy.is_shutdown():

                currentJobPub(pub_job)
                tableCostMap(pub_table)
                publishGoal(pub_goal)
                rate.sleep()

            currentJob = jobQueue.get()

        rospy.loginfo('Job.selected ' + str(currentJob))
        currentJob.set_status(Status.to_bar)

        while (currentJob.status != Status.conformation_wait
               and not rospy.is_shutdown()):

            if currentJob.status == Status.deleted:
                break

            currentJobPub(pub_job)
            tableCostMap(pub_table)
            publishGoal(pub_goal)

            rate.sleep()

        while (currentJob.status != Status.completed
               and not rospy.is_shutdown()):

            if currentJob.status == Status.deleted:
                break

            currentJobPub(pub_job)
            tableCostMap(pub_table)
            publishGoal(pub_goal)

            rate.sleep()

        if currentJob.status == Status.completed:
            job_done.publish(str(currentJob))

        rospy.loginfo('Job ' + currentJob.jobid + ' ' + currentJob.status.name)


if __name__ == '__main__':

    rospy.init_node('job_node', anonymous=True, disable_signals=True)

    try:
        main()
    except KeyboardInterrupt:
        sys.exit()
