#!/usr/bin/env python
'''script for determining the movement of the skeleton'''
##### The joints order is [0, 1, 20, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,
        # 13, 14, 15, 16, 17, 18, 19, 21, 22, 23, 24]

import rospy
import math
import json
import tf
# import time
from k2_client.msg import BodyArray,Body
#  from physical_activities.msg import Joints
from std_msgs.msg import String,ColorRGBA
from visualization_msgs.msg import Marker,MarkerArray

class SkeletonDraw(object):
    '''class for processing'''
    def __init__(self):

        ### Publishers and subscribers

        self.sub = rospy.Subscriber('/head/kinect2/k2_bodies/bodies',
            BodyArray, self.processing)

        self.posture_pub = rospy.Publisher('/skeleton_draw',
                                        MarkerArray, queue_size=5)

        self.base_frame=rospy.get_param('skeleton_frame','/base_link')
        ### other variables
        self.body = None
        self.posture = {}
        self.euler_angles = None
        self.joints = None
        self.orientations = None
        self.points = 25
        self.show = False
        self.elbow_threshold = 15
        #  self.body_info = Joints()
        self.response = None
        self.count = 0
        self.message = None
        
        self.colorcount=0
        self.userColor=[]
        self.userColor.append(ColorRGBA(1.0,0.0,0.0,1.0))
        self.userColor.append(ColorRGBA(0.0,1.0,0.0,1.0))
        self.userColor.append(ColorRGBA(0.0,0.0,1.0,1.0))
        self.userColor.append(ColorRGBA(1.0,1.0,0.0,1.0))
        self.userColor.append(ColorRGBA(1.0,0.0,1.0,1.0))
        self.userColor.append(ColorRGBA(0.0,1.0,1.0,1.0))
        self.userColor.append(ColorRGBA(1.0,1.0,1.0,1.0))
        self.userColor.append(ColorRGBA(0.2,0.2,0.8,1.0))
        
        self.bone_list=[]
        self.bone_list.append([1,20])
        self.bone_list.append([1,0])
        self.bone_list.append([16,0])
        self.bone_list.append([16,17])
        self.bone_list.append([18,17])
        self.bone_list.append([18,19])
        self.bone_list.append([12,0])
        self.bone_list.append([12,13])
        self.bone_list.append([14,13])
        self.bone_list.append([14,15])
        self.bone_list.append([2,20])
        self.bone_list.append([3,2])
        self.bone_list.append([4,20])
        self.bone_list.append([4,5])
        self.bone_list.append([6,5])
        self.bone_list.append([6,7])
        self.bone_list.append([21,7])
        self.bone_list.append([22,6])
        self.bone_list.append([8,20])
        self.bone_list.append([8,9])
        self.bone_list.append([9,10])
        self.bone_list.append([10,11])
        self.bone_list.append([11,23])
        self.bone_list.append([10,24])

        self.colorid=dict()
        self.joint = ['BaseSpine', 'MiddleSpine', 'Neck', 'Head', 'LShoulder',
                 'LElbow', 'LWrist', 'LHand', 'RShoulder', 'RElbow', 'RWrist',
                 'RHand', 'LHip', 'LKnee', 'LAnkle', 'LFoot', 'RHip', 'RKnee',
                 'RAnkle', 'RFoot', 'Spine Shoulder', 'TipLHand', 'LThumb',
                 'TipRHand', 'RThumb']

    def processing(self, data):
        '''main processing function'''
        Marray=MarkerArray()
        time=rospy.Time()
        print (len(data.bodies))
        for n_user,body in enumerate(data.bodies):
            #  b=Body()
            #  b.trackingId
            bodycolor=[]
            if self.colorid.has_key(body.trackingId):
                bodycolor=self.colorid.get(body.trackingId)
            else:
                print self.colorcount,len(self.userColor)
                bodycolor=self.userColor[self.colorcount%len(self.userColor)]
                self.colorid[body.trackingId]=bodycolor
                self.colorcount=self.colorcount+1


            for ind_joint,joint in enumerate(body.jointPositions):
                m=Marker()
                m.lifetime=rospy.Duration(0.2)
                m.id=int(str(n_user)+str(ind_joint))
                m.header.frame_id=self.base_frame#"/base_link" #self.joint[joint.jointType]
                m.header.stamp=time
                m.ns="trackID_"+str(body.trackingId)
                m.action=Marker.ADD
                m.type=m.SPHERE
                m.scale.x=0.1
                m.scale.y=0.1
                m.scale.z=0.1
                m.pose.position.x=joint.position.x
                m.pose.position.y=joint.position.y
                m.pose.position.z=joint.position.z
                m.color.r=bodycolor.r
                m.color.g=bodycolor.g
                m.color.b=bodycolor.b
                m.color.a=1.0-0.8*joint.trackingState
                Marray.markers.append(m)
            for bone in self.bone_list:


                line_m=Marker()
                line_m.action=Marker.ADD
                line_m.lifetime=rospy.Duration(0.2)
                line_m.id=int(str(n_user)+'0'+str(bone[0])+'0'+str(bone[1]))
                line_m.type=Marker.LINE_STRIP
                line_m.header.frame_id="/base_link" 
                line_m.header.stamp=time
                #  line_m.pose.position=body.jointPositions[bone[0]].position
                line_m.points.append(body.jointPositions[bone[0]].position)
                line_m.points.append(body.jointPositions[bone[1]].position)
                #  print line_m.points
                line_m.scale.x=0.01
                line_m.color.r=self.userColor[n_user].r
                line_m.color.b=self.userColor[n_user].b
                line_m.color.g=self.userColor[n_user].g
                line_m.color.a=1.0
                #  Marray.markers.append(line_m)

        self.posture_pub.publish(Marray)
        # print 'bla'

        #if len(data.bodies) == 0:
            # there is no body in view
            # print "No data"

           # self.body = None
          #  self.joints = None
         #   self.orientations = None

        #else:
            #self.count = self.count + 1
            # print self.count
            # there is at least one body in view
            
            #self.euler_angles = []
            #self.body = data.bodies[0]
            #self.joints = self.body.jointPositions
            #self.orientations = data.bodies[0].jointOrientations
            #self.convert_quaternion()
            
            # print self.joints[4]
            #self.process_wholebody()
            #self.response = json.dumps(self.posture)

            #self.posture_pub.publish(self.response)

            #print self.posture
            # print "Head", self.body_info.head
            # print 'am trimis'
    #  def comp_base_angles(self, first_point, second_point):
    #      '''computes the angle between the given points'''
    #
    #      lat_y = abs(second_point.position.x - first_point.position.x)
    #      lat_x = first_point.position.y - second_point.position.y
    #      angle = math.degrees(math.atan2(lat_y, lat_x))
    #      return angle
    #
    #  def process_wholebody(self):
    #      '''this method processes the body information'''
    #
    #      # incep cu capul
    #      self.process_head()
    #
    #      self.process_arms()
    #      self.process_body()
    #      # print self.posture
    #
    #  def set_values(self, par, parset, val_less, val_more, cond):
    #
    #      if par < cond:
    #          self.posture[parset] = val_less
    #      else:
    #          self.posture[parset] = val_more
    #
    #  def process_head(self):
    #      '''returns the state of the head'''
    #      # I need the head, neck and spine
    #      self.get_head()
    #      self.get_neck()
    #      self.get_arms()
    #
    #      # print 'Info cap', self.body_info.head
    #      head_front_back = self.body_info.head.position.z - self.body_info.neck.position.z
    #      self.set_values(head_front_back, 'Head_FrontBack', 'Front', 'Back', 0)
    #      if head_front_back < 0:
    #          self.posture['Head_FrontBack'] = 'Front'
    #      else:
    #          self.posture['Head_FrontBack'] = 'Back'
    #
    #      head_left_right = self.body_info.head.position.x - self.body_info.neck.position.x
    #      self.set_values(head_left_right, 'Head_LeftRight', 'Left', 'Right', 0)
    #
    #      # print self.response['Head_FrontBack']
    #
    #      # aici am umerii
    #      dif_gat_l = self.body_info.neck.position.y - self.body_info.lshoulder.position.y
    #      dif_gat_r = self.body_info.neck.position.y - self.body_info.rshoulder.position.y
    #
    #  def process_body(self):
    #      '''returns the state of the body'''
    #
    #      self.get_spine()
    #      # self.get_neck()
    #
    #
    #      # inclinat stanga/dreapta
    #      angle_left_right = self.comp_base_angles(self.body_info.spinebase,
    #                                                  self.body_info.neck)
    #      left_right = self.body_info.neck.position.x - self.body_info.spinebase.position.x
    #
    #      self.set_values(left_right, 'Body_LeftRight', 'Left', 'Right', 0)
    #      self.posture['Degree_LeftRight'] = angle_left_right
    #
    #      # inclinat fata/spate
    #      front_back = self.body_info.neck.position.z - self.body_info.spinebase.position.z
    #      how_much_front_back = self.body_info.neck.position.y - self.body_info.spinebase.position.y
    #
    #      self.set_values(front_back, 'Body_FrontBack', 'Front', 'Back', 0)
    #      self.posture['Degree_FrontBack'] = abs(how_much_front_back)
    #
    #  # def send_posture(self, msg):
    #  #     '''sends the posture '''
    #
    #  #     print 'trimit acum'
    #  #     self.message = msg
    #  #     self.response = json.dumps(self.posture)
    #  #     print self.response
    #
    #  #     return getPostureResponse(self.response)
    #
    #  def process_arms(self):
    #      '''returns the state of both arms'''
    #
    #      # self.get_arms()
    #      l_shoulder_elbow = self.comp_base_angles(self.body_info.lshoulder,
    #                                               self.body_info.lelbow)
    #      l_shoulder_wrist = self.comp_base_angles(self.body_info.lshoulder,
    #                                            self.body_info.lwrist)
    #      r_shoulder_elbow = self.comp_base_angles(self.body_info.rshoulder,
    #                                               self.body_info.relbow)
    #      r_shoulder_wrist = self.comp_base_angles(self.body_info.rshoulder,
    #                                            self.body_info.rwrist)
    #
    #      lelbow_diff = abs(l_shoulder_elbow - l_shoulder_wrist)
    #      relbow_diff = abs(r_shoulder_elbow - r_shoulder_wrist)
    #
    #      # cat de sus e ridicata mana in lateral
    #      self.posture['Angle_LeftArm'] = round(l_shoulder_elbow)
    #      self.posture['Angle_RightArm'] = round(r_shoulder_elbow)
    #      self.posture['LElbow_diff'] = lelbow_diff
    #      self.posture['RElbow_diff'] = relbow_diff
    #
    #      if lelbow_diff > self.elbow_threshold:
    #          self.posture['LStraight_arm'] = 'False'
    #      else:
    #          self.posture['LStraight_arm'] = 'True'
    #
    #      if relbow_diff > self.elbow_threshold:
    #          self.posture['RStraight_arm'] = 'False'
    #      else:
    #          self.posture['RStraight_arm'] = 'True'
    #
    #  def process_legs(self):
    #      '''returns the state of both legs'''
    #      pass
    #
    #  def get_head(self):
    #      '''get the position and orientation of the head'''
    #       # head
    #      self.body_info.head.position = self.joints[4].position
    #      # self.body_info.head.orientation.roll = self.euler_angles[4][0]
    #      # self.body_info.head.orientation.pitch = self.euler_angles[4][1]
    #      # self.body_info.head.orientation.yaw = self.euler_angles[4][2]
    #
    #  def get_neck(self):
    #      '''get the position and orientation of the neck'''
    #      # neck
    #      self.body_info.neck.position = self.joints[3].position
    #      self.body_info.neck.orientation.roll = self.euler_angles[3][0]
    #      self.body_info.neck.orientation.pitch = self.euler_angles[3][1]
    #      self.body_info.neck.orientation.yaw = self.euler_angles[3][2]
    #
    #  def get_spine(self):
    #      '''get the position and orientation of the spine points'''
    #      # spine_base
    #      self.body_info.spinebase.position = self.joints[0].position
    #      self.body_info.spinebase.orientation.roll = self.euler_angles[0][0]
    #      self.body_info.spinebase.orientation.pitch = self.euler_angles[0][1]
    #      self.body_info.spinebase.orientation.yaw = self.euler_angles[0][2]
    #
    #      # spine_mid
    #      self.body_info.spinemid.position = self.joints[1].position
    #      self.body_info.spinemid.orientation.roll = self.euler_angles[1][0]
    #      self.body_info.spinemid.orientation.pitch = self.euler_angles[1][1]
    #      self.body_info.spinemid.orientation.yaw = self.euler_angles[1][2]
    #
    #      # spine_shoulder
    #      self.body_info.spineshoulder.position = self.joints[2].position
    #      self.body_info.spineshoulder.orientation.roll = self.euler_angles[2][0]
    #      self.body_info.spineshoulder.orientation.pitch = self.euler_angles[2][1]
    #      self.body_info.spineshoulder.orientation.yaw = self.euler_angles[2][2]
    #
    #  def get_arms(self):
    #      '''get the position and orientation of the points on the arms'''
    #      # left shoulder
    #      self.body_info.lshoulder.position = self.joints[5].position
    #      self.body_info.lshoulder.orientation.roll = self.euler_angles[5][0]
    #      self.body_info.lshoulder.orientation.pitch = self.euler_angles[5][1]
    #      self.body_info.lshoulder.orientation.yaw = self.euler_angles[5][2]
    #
    #      # right shoulder
    #      self.body_info.rshoulder.position = self.joints[9].position
    #      self.body_info.rshoulder.orientation.roll = self.euler_angles[9][0]
    #      self.body_info.rshoulder.orientation.pitch = self.euler_angles[9][1]
    #      self.body_info.rshoulder.orientation.yaw = self.euler_angles[9][2]
    #
    #      # left elbow
    #      self.body_info.lelbow.position = self.joints[6].position
    #      self.body_info.lelbow.orientation.roll = self.euler_angles[6][0]
    #      self.body_info.lelbow.orientation.pitch = self.euler_angles[6][1]
    #      self.body_info.lelbow.orientation.yaw = self.euler_angles[6][2]
    #
    #      # right elbow
    #      self.body_info.relbow.position = self.joints[10].position
    #      self.body_info.relbow.orientation.roll = self.euler_angles[10][0]
    #      self.body_info.relbow.orientation.pitch = self.euler_angles[10][1]
    #      self.body_info.relbow.orientation.yaw = self.euler_angles[10][2]
    #
    #      # left wrist
    #      self.body_info.lwrist.position = self.joints[7].position
    #      self.body_info.lwrist.orientation.roll = self.euler_angles[7][0]
    #      self.body_info.lwrist.orientation.pitch = self.euler_angles[7][1]
    #      self.body_info.lwrist.orientation.yaw = self.euler_angles[7][2]
    #
    #      # right wrist
    #      self.body_info.rwrist.position = self.joints[11].position
    #      self.body_info.rwrist.orientation.roll = self.euler_angles[11][0]
    #      self.body_info.rwrist.orientation.pitch = self.euler_angles[11][1]
    #      self.body_info.rwrist.orientation.yaw = self.euler_angles[11][2]
    #
    #  def get_hands(self):
    #      '''get the position and orientations of the hands'''
    #      # left hand
    #      self.body_info.lhand.position = self.joints[8].position
    #      self.body_info.lhand.orientation.roll = self.euler_angles[8][0]
    #      self.body_info.lhand.orientation.pitch = self.euler_angles[8][1]
    #      self.body_info.lhand.orientation.yaw = self.euler_angles[8][2]
    #
    #      # right hand
    #      self.body_info.rhand.position = self.joints[12].position
    #      self.body_info.rhand.orientation.roll = self.euler_angles[12][0]
    #      self.body_info.rhand.orientation.pitch = self.euler_angles[12][1]
    #      self.body_info.rhand.orientation.yaw = self.euler_angles[12][2]
    #
    #      # left hand tip
    #      self.body_info.lhandtip.position = self.joints[21].position
    #      self.body_info.lhandtip.orientation.roll = self.euler_angles[21][0]
    #      self.body_info.lhandtip.orientation.pitch = self.euler_angles[21][1]
    #      self.body_info.lhandtip.orientation.yaw = self.euler_angles[21][2]
    #
    #      # right hand tip
    #      self.body_info.rhandtip.position = self.joints[23].position
    #      self.body_info.rhandtip.orientation.roll = self.euler_angles[23][0]
    #      self.body_info.rhandtip.orientation.pitch = self.euler_angles[23][1]
    #      self.body_info.rhandtip.orientation.yaw = self.euler_angles[23][2]
    #
    #      # left thumb
    #      self.body_info.lthumb.position = self.joints[22].position
    #      self.body_info.lthumb.orientation.roll = self.euler_angles[22][0]
    #      self.body_info.lthumb.orientation.pitch = self.euler_angles[22][1]
    #      self.body_info.lthumb.orientation.yaw = self.euler_angles[22][2]
    #
    #      # right thumb
    #      self.body_info.rthumb.position = self.joints[24].position
    #      self.body_info.rthumb.orientation.roll = self.euler_angles[24][0]
    #      self.body_info.rthumb.orientation.pitch = self.euler_angles[24][1]
    #      self.body_info.rthumb.orientation.yaw = self.euler_angles[24][2]
    #
    #  def get_legs(self):
    #      '''get the position and orientations of the points on the legs'''
    #      # left hip
    #      self.body_info.lhip.position = self.joints[13].position
    #      self.body_info.lhip.orientation.roll = self.euler_angles[13][0]
    #      self.body_info.lhip.orientation.pitch = self.euler_angles[13][1]
    #      self.body_info.lhip.orientation.yaw = self.euler_angles[13][2]
    #
    #      # right hip
    #      self.body_info.rhip.position = self.joints[17].position
    #      self.body_info.rhip.orientation.roll = self.euler_angles[17][0]
    #      self.body_info.rhip.orientation.pitch = self.euler_angles[17][1]
    #      self.body_info.rhip.orientation.yaw = self.euler_angles[17][2]
    #
    #      # left knee
    #      self.body_info.lknee.position = self.joints[14].position
    #      self.body_info.lknee.orientation.roll = self.euler_angles[14][0]
    #      self.body_info.lknee.orientation.pitch = self.euler_angles[14][1]
    #      self.body_info.lknee.orientation.yaw = self.euler_angles[14][2]
    #
    #      # right knee
    #      self.body_info.rknee.position = self.joints[18].position
    #      self.body_info.rknee.orientation.roll = self.euler_angles[18][0]
    #      self.body_info.rknee.orientation.pitch = self.euler_angles[18][1]
    #      self.body_info.rknee.orientation.yaw = self.euler_angles[18][2]
    #
    #      # left foot
    #      self.body_info.lfoot.position = self.joints[16].position
    #      self.body_info.lfoot.orientation.roll = self.euler_angles[16][0]
    #      self.body_info.lfoot.orientation.pitch = self.euler_angles[16][1]
    #      self.body_info.lfoot.orientation.yaw = self.euler_angles[16][2]
    #
    #      # right foot
    #      self.body_info.rfoot.position = self.joints[20].position
    #      self.body_info.rfoot.orientation.roll = self.euler_angles[20][0]
    #      self.body_info.rfoot.orientation.pitch = self.euler_angles[20][1]
    #      self.body_info.rfoot.orientation.yaw = self.euler_angles[20][2]
    #
    #      # left ankle
    #      self.body_info.lankle.position = self.joints[15].position
    #      self.body_info.lankle.orientation.roll = self.euler_angles[15][0]
    #      self.body_info.lankle.orientation.pitch = self.euler_angles[15][1]
    #      self.body_info.lankle.orientation.yaw = self.euler_angles[15][2]
    #
    #      # right ankle
    #      self.body_info.rankle.position = self.joints[19].position
    #      self.body_info.rankle.orientation.roll = self.euler_angles[19][0]
    #      self.body_info.rankle.orientation.pitch = self.euler_angles[19][1]
    #      self.body_info.rankle.orientation.yaw = self.euler_angles[19][2]
    #
    #  def convert_quaternion(self):
    #      '''returns the euler angles from quaternion'''
    #      quater = None
    #      euler = None
    #
    #      for ind in range(self.points):
    #          quater = (self.orientations[ind].orientation.x,
    #                    self.orientations[ind].orientation.y,
    #                    self.orientations[ind].orientation.z,
    #                    self.orientations[ind].orientation.w)
    #
    #          euler = tf.transformations.euler_from_quaternion(quater)
    #          self.euler_angles.append(euler)

def main():
    '''main function '''
    rospy.init_node('skeleton', anonymous=True)
    sk=SkeletonDraw()
    try:

        rospy.spin()
    except KeyboardInterrupt:
        print 'Shutting down'

if __name__ == '__main__':
    main()
