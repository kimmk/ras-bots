#!/usr/bin/python3

import os
import sys
import signal
import cv2
import imutils
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import matplotlib.pyplot as plt
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose

bridge = CvBridge()

#devel mode = 1: picking single images from file to process
#devel mode = 0: picking images from subscriber image stream
devel_mode = 1
debug_mode = 1

def is_within_bbox(p, bbox):
    x2 = bbox[0]+bbox[2]
    y2 = bbox[1]+bbox[3]
    return p[0] >= bbox[0] and p[1] >= bbox[1] and p[0] <= x2 and p[1] <= y2

def points_within_bbox(cnts, bbox):
    points = []
    for cnt in cnts:
        points.extend([[[p[0][0],p[0][1]]] for p in cnt if is_within_bbox(p[0], bbox)])
    return np.array(points, dtype=np.int32)

def add_bboxes(a, b):
    # x, y, w, h
    x = min(a[0], b[0])
    y = min(a[1], b[1])
    a_xmax = a[0] + a[2]
    a_ymax = a[1] + a[3]
    b_xmax = b[0] + b[2]
    b_ymax = b[1] + b[3]
    xmax = max(a_xmax, b_xmax)
    ymax = max(a_ymax, b_ymax)
    w = xmax-x
    h = ymax-y
    return (x,y,w,h)

class GateDetector:
    def __init__(self):
        self.gate_img = rospy.Publisher("gate_img", Image, queue_size=10)       # Debug image of gate vision
        #self.gate_angle = rospy.Publisher("gate_angle", Float64, queue_size=10) # Estimated angle to gate
        self.gate_pose = rospy.Publisher("gate_pose", Pose, queue_size=1)   # Estimated pose of gate
        self.kerneldim = 19
        self.largest_element = 0
        rospy.Subscriber("/image", Image, self.camera_callback)                 # Tello camera image

    def test_video(self, filepath):
        cap = cv2.VideoCapture(filepath)
        while(cap.isOpened()):
            ret, frame = cap.read()
            self.image_processing(frame)
            #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            #cv2.imshow('frame',gray)
            #wait for n millisec or a keypress
            if cv2.waitKey(100) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()
        
        
    def test_image(self, filepath):
        cv2_img = cv2.imread(filepath)
        
        if cv2_img is not None:
            
            self.image_processing(cv2_img)
            
    def test_images(self, path):
        for i in range(0,100):
            
            
            file = "/picture" + str(i) + ".png"
            #print(file)
            self.test_image(path+file)
    
        #for file in os.listdir(path):
        #    self.test_image(path+"/"+file)

    def camera_callback(self, img):
        try:
            cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        else:
            if debug_mode: print("img recvd")
            self.image_processing(cv2_img)

    def image_processing(self, cv2_img):
        self.kerneldim = 19
        cv2_img = imutils.resize(cv2_img, width=600)
        gate_data = self.detect_gate(cv2_img.copy())
        
        attempts = 0
        while gate_data is None and attempts < 4:
            
            #largest element updated in find_gate_4gon
            if self.largest_element < 15000:
                self.kerneldim -= 2
            else:
                self.kerneldim += 2
            attempts += 1
            print("largest shape " + str(self.largest_element))
            print("kerneldim " + str(self.kerneldim))
            self.largest_element = 0
            gate_data = self.detect_gate(cv2_img.copy())
        
        
        if gate_data is not None:
            pose = self.createposition(gate_data)
            self.gate_pose.publish(pose)
            
    def createposition(self,gate_data):
        
        angle, dist, (x, y) = gate_data
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = dist
        pose.orientation.w= angle
        pose.orientation.x=0
        pose.orientation.y=0
        pose.orientation.z=1
        return pose
    
    
    def imshow_bgr(self, img):
        cv2.imshow("frame", img)
        #img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        
        #plt.imshow(img)
        #plt.show()

    def imshow_grayscale(self, img):    
        plt.imshow(img, cmap='gray', vmin=0, vmax=255)
        plt.show()

    def debug_draw_gate(self, gate, img):
        angle,dist,center = gate
        p = (img.shape[0]*1//3,img.shape[1]*2//3)
        #cv2.putText(img, f"angle: {angle:.2f}", p, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.circle(img, center, 4, (255,255,255), -1)

    def filter_gates(self, img):
        # Apply HSV filter for green gates
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        clr_gate = np.array([38, 195, 143])
        clr_range = np.array([5, 60, 75])
        lower = np.array(clr_gate-clr_range, dtype="uint8")
        upper = np.array(clr_gate+clr_range, dtype="uint8")
        mask = cv2.inRange(img_hsv, lower, upper)
        img_masked = cv2.bitwise_and(img_hsv, img_hsv, mask=mask)

        # Take the grayscale image from value channel, 
        #fliter out all but vertical and horizontal shapes and threshold it
        gray = cv2.split(img_masked)[2]
        
        
        
        blurred = cv2.medianBlur(gray, ksize= 7)
        
        kerneldim = self.kerneldim
        kernelH    =  (1.0/kerneldim)*np.ones((1,kerneldim))
        kernelV    =  (1.0/kerneldim)*np.ones((kerneldim, 1))
        
        erodeH=cv2.erode(blurred, kernelH, iterations=1)
        dilateH=cv2.dilate(erodeH, kernelH, iterations=1)
        erodeV=cv2.erode(dilateH, kernelV, iterations=1)
        dilateV=cv2.dilate(erodeV, kernelV, iterations=1)
        
        thresh = cv2.threshold(dilateV, 10, 255, cv2.THRESH_BINARY)[1]
        
        
        if devel_mode and False:
            #self.imshow_bgr(gray)
            #print("blurred")
            self.imshow_bgr(blurred)
            #print("H")
            #self.imshow_bgr(dilateH)
            print("V")
            self.imshow_bgr(dilateV)
            #print("thresh")
            #self.imshow_bgr(thresh)
            
        return thresh
        

    def find_gate_4gon(self, cnts, debug_img=None):
        # Select largest contour by it's bounding box
        gate_points = None
        gate_center = None
        biggest_box = 0
        box_ratio = 0
        for points in cnts:
            x,y,w,h = cv2.boundingRect(points)
            box_sz = w*h
            
            if debug_img is not None:
                cv2.rectangle(debug_img, (x,y), (x+w,y+h), (0,0,255), 2)
            if box_sz > biggest_box and box_sz > 30000:
                biggest_box = box_sz
                box_ratio = (w+0.0)/h
                gate_points = points
                gate_center = (x+w//2,y+h//2)
                
            #store largest element for calibration check
            if box_sz > self.largest_element:
                self.largest_element = box_sz
                
        # If found, try to approximate gate as a 4-gon
        if gate_points is not None and box_ratio > 0.3 and box_ratio < 3.0:
            epsilon = 0.05*cv2.arcLength(gate_points,closed = True)
            approx = cv2.approxPolyDP(gate_points,epsilon,closed = True)
            if debug_img is not None:
                #print("draw approx, blue")
                pass
                #self.imshow_bgr(debug_img)
            if len(approx) == 4:
                bbox = cv2.boundingRect(gate_points)
                if debug_img is not None:
                    cv2.drawContours(debug_img,[approx],0,(255,0,0),2)
                
                
                
                w_half = bbox[2]//2
                bbox_a = (bbox[0],bbox[1],w_half,bbox[3])
                bbox_b = (bbox[0]+w_half,bbox[1],w_half,bbox[3])

                points_a = points_within_bbox(cnts, bbox_a)
                points_b = points_within_bbox(cnts, bbox_b)
                if devel_mode:
                    print("points_a")
                    print(points_a)
                bbox_a = cv2.boundingRect(points_a)
                bbox_b = cv2.boundingRect(points_b)
                
                h1 = bbox_a[3]
                h2 = bbox_b[3]
                
                
                gate = [bbox, h1,h2]
                
                """
                box = [c[0] for c in approx] # resolve annoying lists within list
                box.sort(key=lambda p: p[0]) # sort points by x coordinate

                h0 = np.linalg.norm(np.array(box[0])-np.array(box[1]))
                h1 = np.linalg.norm(np.array(box[2])-np.array(box[3]))
                gate_angle = h0/h1
                gate = (gate_angle, -1, gate_center)

                if debug_img is not None:
                    for i, coord in enumerate(box):
                        p = tuple(coord)
                        #cv2.putText(debug_img, f"{i}", p, cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                        #cv2.circle(img, p, 4, (255,255,255), -1)
                    self.debug_draw_gate(gate, debug_img)
                #"""
                return gate
        return None

    def find_gate_6gon(self, cnts, debug_img):
        candidates = []
        for points in cnts:
            #epsilon = 0.03*cv2.arcLength(points,True)
            #approx = cv2.approxPolyDP(points,epsilon,True)
            
            x,y,w,h = cv2.boundingRect(points)
            box_sz = w*h
            
            if box_sz > 1000:
                epsilon = 0.03*cv2.arcLength(points,closed = True)
                approx = cv2.approxPolyDP(points,epsilon,closed = True)
                if len(approx) >=5 and len(approx) <=7:
                    candidates.append((approx, box_sz))
        
                
                
                
                if debug_img is not None:
                    #print(approx[0][0])
                    #cv2.putText(debug_img, f"{len(approx)}", tuple(approx[0][0]), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                    clr = (255,255,255)
                    #if len(approx) == 5:
                        #clr = (255,0,255)
                    cv2.drawContours(debug_img,[approx],0,clr,2)
        
        #sorting in descending size
        candidates = sorted(candidates, key = lambda x: x[1], reverse = True)
        if debug_mode:
            print("corners:candidates sizes")
            cand_sizes = [x[1] for x in candidates]
            print(cand_sizes)
        cand_len = len(candidates)
        corners = []
        if debug_mode:
            print("corners:cand_len: " + str(cand_len))
        if cand_len >=3:
            corners = candidates[0][0]
            corner_bboxes = [cv2.boundingRect(corners)]
            #print(corners)
            
            
            for i in range(1,min(cand_len,4)):
                if candidates[i][1] > 0.3*candidates[0][1]:
                    cand = candidates[i][0]
                    corner_bboxes.append(cv2.boundingRect(cand))
                    corners = np.append(corners, cand, 0)
            cornercount = len(corner_bboxes)
            if debug_mode: print("corners:cornercount: " + str(cornercount))
            #print(corners)
            #print(len(corner_bboxes))
            #flat_corners = [item for sublist in corners for item in sublist]
            
            #print(flat_corners)
            
            
            corners = cv2.UMat(corners)
            gate_bbox = cv2.boundingRect(corners)
            box_ratio = (gate_bbox[2]+0.0)/gate_bbox[3]
            #if box wider than tall its not the proper gate
            if box_ratio > 1.0:
                return None
            #print(gate_bbox)
            #print(corner_bboxes)
            #sort corners from left to rigth
            corner_bboxes = sorted(corner_bboxes, key = lambda x: x[0], reverse = False)
            
            
            
            
            if cornercount >=3:
                two_lefts = False
                two_rights = False
                if corner_bboxes[1][0] < gate_bbox[0] + gate_bbox[2]*0.4:
                    two_lefts = True
                if corner_bboxes[-2][0] > gate_bbox[0] + gate_bbox[2]*0.4:
                    two_rights = True
                #print(two_lefts)
            
            #bbox format x,y,w,h
                gate_top = gate_bbox[1]
                gate_bot = gate_bbox[1]+gate_bbox[3]
                corner_top = []
                corner_bot = []
                for i in range(cornercount):
                    corner_top.append(corner_bboxes[i][1])
                    corner_bot.append(corner_bboxes[i][1]+corner_bboxes[i][3])
                
            
                if two_lefts:
                    dist_left_top_edge = min(abs(gate_top-corner_top[0]), abs(gate_top-corner_top[1]))
                    dist_left_bot_edge = min(abs(gate_bot-corner_bot[0]), abs(gate_bot-corner_bot[1]))
                    h1 = gate_bbox[3]-np.mean([dist_left_top_edge, dist_left_bot_edge])
                else:
                    dist_left_top_edge = abs(gate_top-corner_top[0])
                    dist_left_bot_edge = abs(gate_bot-corner_bot[0])
                    h1 = gate_bbox[3]-min(dist_left_top_edge, dist_left_bot_edge)
                    
                if two_rights:
                    dist_right_top_edge = min(abs(gate_top-corner_top[cornercount-1]), abs(gate_top-corner_top[cornercount-2]))
                    dist_right_bot_edge = min(abs(gate_bot-corner_bot[cornercount-1]), abs(gate_bot-corner_bot[cornercount-2]))
                    h2 = gate_bbox[3]-np.mean([dist_right_top_edge, dist_right_bot_edge])
                else:
                    dist_right_top_edge = abs(gate_top-corner_top[cornercount-1])
                    dist_right_bot_edge = abs(gate_bot-corner_bot[cornercount-1])
                    h2 = gate_bbox[3]-min(dist_right_top_edge, dist_right_bot_edge)
            #print(corner_bboxes[0][1])
                #print([h1,h2])
                if debug_img is not None:
                    x,y,w,h = gate_bbox
                    cv2.rectangle(debug_img, (x,y), (x+w,y+h), (255,0,255), 2)
                return(gate_bbox, h1,h2)
        return None
        
        
    def detect_gate(self, img):
        gate = None
        debug_img = None
        if devel_mode:
            debug_img = img

        # Filter 'gate' features from image
        gate_img = self.filter_gates(img)

        # Get gate contours
        cnts = cv2.findContours(gate_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        im_cnts = imutils.grab_contours(cnts)
        #im_cnts = [cnt for cnt in im_cnts if len(cnt)>100]
        
        if devel_mode:
            cv2.drawContours(debug_img, im_cnts, -1, (0, 255, 0), 2)

        # Find gates, both rectangle and four corners-type
        gate_rect = self.find_gate_4gon(im_cnts, debug_img)
        
        gate_parts = self.find_gate_6gon(im_cnts, debug_img)
        if debug_mode:
            print("Gate data: rect and corners, bbox, h1,h2")
            print(gate_rect)
            print(gate_parts)
        
        
        #picking the bigger(closer) gate
        sz_gate_rect = 0
        sz_gate_parts = 0
        
        gate_data = None
        if gate_rect:
            sz_gate_rect = gate_rect[0][2]*gate_rect[0][3]
        if gate_parts:
            sz_gate_parts = gate_parts[0][2]*gate_parts[0][3]
        if sz_gate_rect or sz_gate_parts:
            if sz_gate_rect > sz_gate_parts:
                gate_data = self.detect_gate_angle(img,gate_rect[0],gate_rect[1],gate_rect[2])
            else:
                gate_data = self.detect_gate_angle(img,gate_parts[0],gate_parts[1],gate_parts[2])
        
        
        
        
        if debug_img is not None:
            self.imshow_bgr(debug_img)
            self.gate_img.publish(bridge.cv2_to_imgmsg(debug_img))
        
        return gate_data
    
    def detect_gate_angle(self, img, bbox, h1,h2):
        
        #napkin calculations reveal:
                #sin alpha = (r1-r2)/(w/2), with one r being close edge, other being center, w being actual gate width=0.7m
                #sin alpha = alpha if angle < 40deg
                #r = cali/h, with cali=px*dist=?, h= bbox height
                #-> alpha = cali/0.35*(1/h1-1/h2)
        cali = 250.0
        gate_w = 0.7
        angle = cali/(gate_w/2.0)*(1.0/h1-1.0/h2)
        #angle = 1/bbox_a[3]-1/bbox_b[3]
            
            
        center = (bbox[0]+bbox[2]//2.0,bbox[1]+bbox[3]//2.0)
        image_width_half = img.shape[1]/2.0
        image_height_half = img.shape[0]/2.0
        center = ((center[0]-image_width_half)/image_width_half, (center[1]-image_height_half)/image_height_half)
            
            
        #smaller bbox corresponds to center height, cali=px*dist
        dist = cali/(min(h1,h2))
        gate = (angle,dist,center)
        print("gate data: angle, dist, (x,y %)")
        print(gate)
        return(gate)
    
    def detect_gate_bbox(self, img):
        gate = None
        if devel_mode:
            debug_img = img

        # Filter 'gate' features from image
        gate_img = self.filter_gates(img)

        # Get gate contours
        cnts = cv2.findContours(gate_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        im_cnts = imutils.grab_contours(cnts)

        # Get gate contour bboxes
        bboxes = [cv2.boundingRect(points) for points in im_cnts]

        # Remove little bboxes
        bboxes = [b for b in bboxes if b[2] > 30 and b[3] > 30]
        
        # Combine close enough bboxes
        combi_bboxes = []
        combi_range = img.shape[0]*0.3
        #b = add_bboxes(bboxes[0], bboxes[1])
        #x,y,w,h = b

        # Get largest bbox
        bbox = (-1,-1,-1,-1)
        for b in bboxes:
            if b[2]*b[3] > bbox[2]*bbox[3]:
                bbox = b
        box_ratio = (bbox[2]+0.0)/bbox[3]
        
        if devel_mode: 
            print("bbox")
            print(bbox)
            print(box_ratio)
        
        if bbox[0] != -1 and box_ratio > 0.3 and box_ratio < 3.0:
            w_half = bbox[2]//2
            bbox_a = (bbox[0],bbox[1],w_half,bbox[3])
            bbox_b = (bbox[0]+w_half,bbox[1],w_half,bbox[3])

            points_a = points_within_bbox(im_cnts, bbox_a)
            points_b = points_within_bbox(im_cnts, bbox_b)
            print("points_a")
            print(points_a)
            bbox_a = cv2.boundingRect(points_a)
            bbox_b = cv2.boundingRect(points_b)

            # angle,dist,center
            
            #napkin calculations reveal:
                #sin alpha = (r1-r2)/(w/2), with one r being close edge, other being center, w being actual gate width=0.7m
                #sin alpha = alpha if angle < 40deg
                #r = cali/h, with cali=px*dist=?, h= bbox height
                #-> alpha = cali/0.35*(1/h1-1/h2)
            cali = 250.0
            gate_w = 0.7
            angle = cali/(gate_w/2.0)*(1.0/bbox_a[3]-1.0/bbox_b[3])
            #angle = 1/bbox_a[3]-1/bbox_b[3]
            
            
            center = (bbox[0]+bbox[2]//2.0,bbox[1]+bbox[3]//2.0)
            image_width_half = img.shape[1]/2.0
            image_height_half = img.shape[0]/2.0
            center = ((center[0]-image_width_half)/image_width_half, (center[1]-image_height_half)/image_height_half)
            
            
            #smaller bbox corresponds to center height, cali=px*dist
            dist = cali/(min(bbox_a[3],bbox_b[3]))
            gate = (angle,dist,center)
            print(gate)
            if devel_mode:
                x,y,w,h = bbox_a
                cv2.rectangle(debug_img, (x,y), (x+w,y+h), (255,0,0), 2)
                x,y,w,h = bbox_b
                cv2.rectangle(debug_img, (x,y), (x+w,y+h), (0,0,255), 2)
                #self.debug_draw_gate(gate, img)

        if devel_mode:
            cv2.drawContours(debug_img, im_cnts, -1, (0, 255, 0), 2)
            self.imshow_bgr(debug_img)

        return gate

def handle_exit(signum, frame):
    sys.exit(0)

if __name__ == '__main__':
    signal.signal(signal.SIGINT, handle_exit)
    rospy.init_node('gate_detector', anonymous=True)
    gate = GateDetector()
    
    if devel_mode:
        print("main")
        #gate.test_images("../images/tello_gates")
        gate.test_video("../images/gate_vids/video.avi")
    else:
        rospy.spin()
    #rospy.spin()
    #gate.test_image("gate.png")
    
