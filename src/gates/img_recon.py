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
from datetime import datetime

################################### Global Defs
bridge = CvBridge()

################################### DEVEL VARS & DEFS
#devel mode = 1: picking single images from file to process
#devel mode = 2: write cv2 img and log in file
#devel mode = 0: picking images from subscriber image stream
devel_mode = 1
debug_mode = 0
dateTimeFormat = "%d-%m-%Y_%H:%M:%S"
sloperun = 0

# Not deleted: required when debugging without Drone
# uncomment calls to this def to use it
def imshow_bgr(self, img):
    cv2.imshow("frame", img)
    #img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    #plt.imshow(img)
    #plt.show()

################################### NOT BEING CALLED ANYWHERE ??
## Returns all the points from input that in inside bbox
def points_within_bbox(cnts, bbox):
    points = []
    for cnt in cnts:
        points.extend([[[p[0][0],p[0][1]]] for p in cnt if is_within_bbox(p[0], bbox)])
    return np.array(points, dtype=np.int32)

## Point P is inside bbox
def is_within_bbox(p, bbox):
    x2 = bbox[0]+bbox[2]
    y2 = bbox[1]+bbox[3]
    return p[0] >= bbox[0] and p[1] >= bbox[1] and p[0] <= x2 and p[1] <= y2

## Takes bboxes a and b, returns bbox a+b
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

##################################### Output Creation
def createposition(self,gate_data):
    angle, dist, (x, y) = gate_data
    pose = Pose()
    #coord shift beacuse x-y is on the vertical image plane
    pose.position.x = x
    pose.position.y = y
    pose.position.z = -dist
    pose.orientation.w= angle
    pose.orientation.x=0
    pose.orientation.y=0
    pose.orientation.z=1
    return pose

##################################### Data Input


def camera_callback(self, img):
    try:
        cv2_img = bridge.imgmsg_to_cv2(img, "bgr8")
    except CvBridgeError as e:
        print(e)
    else:
        if debug_mode: print("img recvd")
        self.image_processing(cv2_img)



######################################### Image Detection
class GateDetector:
    def __init__(self):
        self.gate_img = rospy.Publisher("gate_img", Image, queue_size=1) # Debug image of gate vision
        self.gate_pose = rospy.Publisher("gate_pose", Pose, queue_size=1) # Estimated pose of gate
        self.bw_img_pub = rospy.Publisher("bw_img", Image, queue_size=1) # Debug image of gate vision
        self.kerneldim_init = 19
        self.largest_element = 0
        
        #offset corrections in case gate is not recognised centrally, in px, + moves gate in image right
        self.x_offset_6gon = 5
        self.x_offset_4gon = 0
        #rospy.Subscriber("/image", Image, self.camera_callback)  # Tello camera image

    def imshow_grayscale(self, img):    
        plt.imshow(img, cmap='gray', vmin=0, vmax=255)
        plt.show()

    def debug_draw_gate(self, gate, img):
        angle,dist,center = gate
        p = (img.shape[0]*1//3,img.shape[1]*2//3)
        #cv2.putText(img, f"angle: {angle:.2f}", p, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.circle(img, center, 4, (255,255,255), -1)
    
    ## Main Function - Public
    def image_processing(self, cv2_img):
        self.kerneldim = self.kerneldim_init
        cv2_img = imutils.resize(cv2_img, width=600)
        gate_data = self.detect_gate(cv2_img.copy())
        attempts = 0
        while gate_data is None and attempts < 0:
            #largest element updated in find_gate_4gon
            if self.largest_element < 15000:
                self.kerneldim -= 2
            else:
                self.kerneldim += 2
            attempts += 1
            # print("largest shape " + str(self.largest_element))
            # print("kerneldim " + str(self.kerneldim))
            self.largest_element = 0
            gate_data = self.detect_gate(cv2_img.copy())

        if devel_mode > 1:
            if gate_data is not None:
                filename = "gate_"+datetime.now().strftime(dateTimeFormat)+".jpg"
                cv2.imwrite(filename, cv2_img)
        return gate_data
        # Expected returns are None, {Angle2Gate, Distance2Gate, Tuple XY GateInImage}
        # if gate_data is not None:
        #     pose = self.createposition(gate_data)
        #     self.gate_pose.publish(pose)




    ## Orchestrator
    # in: camera image
    # out: gate position as:
           # angle as seen from gate
           # distance to gate
           # gate % offset from image center (x,y)
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
            logging.debug("Gate data: rect and corners, bbox, h1,h2")
            logging.debug(gate_rect)
            logging.debug(gate_parts)
        
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
            #self.imshow_bgr(debug_img)
            #pass
            self.gate_img.publish(bridge.cv2_to_imgmsg(debug_img))
        return gate_data

    ## Image Manipulation
    #in: camera image
    #out: filtered image
    #filters: 
        #1. green mask
        #2. hor and vert filters, effectively a distance filter (small objects get eliminated)
        #3. threshold
    def filter_gates(self, img):
        # Apply HSV filter for green gates
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        clr_gate = np.array([38, 195-30, 143-20])
        clr_range = np.array([10, 60+30, 75+20])
        lower = np.array(clr_gate-clr_range)
        upper = np.array(clr_gate+clr_range)
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
        self.bw_img_pub.publish(bridge.cv2_to_imgmsg(gray))
        #if devel_mode:
        #self.imshow_bgr(img)
            #print("blurred")
            #self.imshow_bgr(blurred)
            #print("H")
            #self.imshow_bgr(dilateH)
            #logging.debug("V")
            #self.imshow_bgr(dilateV)
            #print("thresh")
            #self.imshow_bgr(thresh)
            
        return thresh

    ########################### Feature Detection
    
    #in: all found contours from filtered image
    #out: (gate bounding box (x,y,w,h), height_left_edge, height_right_edge ) 
    def find_gate_4gon(self, cnts, debug_img=None):
        # Select largest contour by it's bounding box
        gate_points = None
        gate_center = None
        gate_box = None
        biggest_box = 0
        box_ratio = 0
        for points in cnts:
            gate_box = cv2.boundingRect(points)
            x,y,w,h = gate_box
            box_sz = w*h
            
            if box_sz > biggest_box and box_sz > 10000:
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
            if len(approx) == 4:
            
                x,y,w,h = cv2.boundingRect(approx)
                x = x+self.x_offset_4gon
                gate_bbox = x, y, w, h
                if debug_img is not None:
                    
                    cv2.rectangle(debug_img, (x,y), (x+w,y+h), (0,0,255), 2)
                    image_x = debug_img.shape[1]//2
                    image_y = debug_img.shape[0]//2
                    
                    center = (int(x+w//2.0),int(y+h//2.0))
                    print("center x,y: ", center)
                    cv2.rectangle(debug_img, (image_x-5,image_y-5), (image_x+5,image_y+5), (255,0,255), 2)
                    cv2.rectangle(debug_img, (center[0]-5,center[1]-5), (center[0]+5,center[1]+5), (255,0,0), 2)
                box = [c[0] for c in approx] # resolve annoying lists within list
                box.sort(key=lambda p: p[0]) # sort points by x coordinate
                # h0 = box left edge height
                # h1 = box right edge height
                #take the 4 points, order them horizontally
                #and then h0 = 'distance of points 0, 1' and h1 = 'distance of points 2, 3'
                h0 = np.linalg.norm(np.array(box[0])-np.array(box[1]))
                h1 = np.linalg.norm(np.array(box[2])-np.array(box[3]))
                
                return (gate_bbox, h0, h1)
        return None
    
    
    #in: all found contours from filtered image
    #out: (gate bounding box (x,y,w,h), height_left_edge, height_right_edge ) 
    def find_gate_6gon(self, cnts, debug_img):
        candidates = []
        for points in cnts:
            #epsilon = 0.03*cv2.arcLength(points,True)
            #approx = cv2.approxPolyDP(points,epsilon,True)
            x,y,w,h = cv2.boundingRect(points)
            box_sz = w*h
            boxlimit = 1000
            if sloperun:
                boxlimit = 100
            if box_sz > boxlimit:
                epsilon = 0.03*cv2.arcLength(points,closed = True)
                approx = cv2.approxPolyDP(points,epsilon,closed = True)
                if len(approx) >=4 and len(approx) <=7:
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
            logging.debug("corners:candidates sizes")
            cand_sizes = [x[1] for x in candidates]
            logging.debug(cand_sizes)
        cand_len = len(candidates)
        corners = []
        if debug_mode:
            logging.debug("corners:cand_len: " + str(cand_len))
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
            if debug_mode: 
                logging.debug("corners:cornercount: " + str(cornercount))
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
            # find left and right edge heights for any case of 3 or 4 corners
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
                x,y,w,h = gate_bbox
                gate_bbox = x+self.x_offset_6gon, y, w, h
                if debug_img is not None:
                    x,y,w,h = gate_bbox
                    cv2.rectangle(debug_img, (x,y), (x+w,y+h), (255,0,255), 2)
                    image_x = debug_img.shape[1]//2
                    image_y = debug_img.shape[0]//2
                    
                    center = (int(x+w//2.0),int(y+h//2.0))
                    print("center x,y: ", center)
                    cv2.rectangle(debug_img, (image_x-5,image_y-5), (image_x+5,image_y+5), (255,0,255), 2)
                    cv2.rectangle(debug_img, (center[0]-5,center[1]-5), (center[0]+5,center[1]+5), (255,0,0), 2)
                
                return(gate_bbox, h1,h2)
        return None

    ## Gate Maths
    # in: gate bounding box, height_left_edge, height_right_edge
    # out: gate position as:
           # angle as seen from gate
           # distance to gate
           # gate % offset from image center (x,y)
    def detect_gate_angle(self, img, bbox, h1,h2):
        cali = 250.0
        gate_w = 0.7
        #angle = cali/(gate_w/2.0)*(1.0/h1-1.0/h2)
        #angle = 1/bbox_a[3]-1/bbox_b[3]
        angle = (h1-h2)/(h1+h2)        
            
        center = (bbox[0]+bbox[2]//2.0,bbox[1]+bbox[3]//2.0)
        image_width_half = img.shape[1]/2.0
        image_height_half = img.shape[0]/2.0
        center = ((center[0]-image_width_half)/image_width_half, (center[1]-image_height_half)/image_height_half)
            
            
        #smaller bbox corresponds to center height, cali=px*dist
        dist = cali/(min(h1,h2))
        gate = (angle,dist,center)
        if debug_mode:
            logging.debug("gate data: angle, dist, (x,y %)")
            logging.debug(gate)
        return(gate)