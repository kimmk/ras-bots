import numpy as np
import cv2
import imutils
import matplotlib.pyplot as plt
from simple_pid import PID

# For debugging
def imshow_grayscale(img):    
    plt.imshow(img, cmap='gray', vmin=0, vmax=255)
    plt.show()

def imshow_hsv(img):
    img = cv2.cvtColor(img, cv2.COLOR_HSV2RGB)
    plt.imshow(img)
    plt.show()

def box_center(box):
    return (box[0]+box[2]//2, box[1]+box[3]//2)

class Lander(object):
    def __init__(self):
        self.pid_x = PID(10.0, 0, 3.0)
        self.pid_y = PID(10.0, 0, 3.0)

    def find_leds(self, img, hsv_min, hsv_max, debug_img=None):

        # Filter LED color only
        lower = np.array(hsv_min, dtype="uint8")
        upper = np.array(hsv_max, dtype="uint8")
        mask = cv2.inRange(img, lower, upper)
        img_masked = cv2.bitwise_and(img, img, mask=mask)

        # Take grayscale, apply blur and threshold
        gray = cv2.split(img_masked)[2]
        blurred = cv2.medianBlur(gray, ksize=5)
        thresh = cv2.threshold(blurred, 5, 255, cv2.THRESH_BINARY)[1]

        # Find contours
        cnts = cv2.findContours(thresh, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        im_cnts = imutils.grab_contours(cnts)

        # Return features that are large enough to be candidates
        leds = []
        for points in im_cnts:
            led_box = cv2.boundingRect(points)
            x,y,w,h = led_box
            if w*h > 20:
                leds.append(led_box)
                if debug_img is not None:
                    cv2.rectangle(debug_img, (x,y), (x+w,y+h), (100,255,255), 2)

        # Sort by size
        leds.sort(key=lambda box: box[2]*box[3], reverse=True)

        return leds

    # img:              landing camera image in cv2 HSV format
    # led_a, led_b:     HSV color ranges for front led (led_a) and back led (led_b)
    # land_pos:         target landing position in image, in percentages of X and Y
    # debug_img:        if set, debug info will be drawn to this image
    def land_update(self, img, led_a, led_b, land_pos=(0.5,0.5), debug_img=None):

        # Find LED candidate positions
        leds_a = self.find_leds(img, led_a[0], led_a[1], debug_img)
        leds_b = self.find_leds(img, led_b[0], led_b[1], debug_img)
        if len(leds_a) == 0 or len(leds_b) == 0:
            return np.array([0,0])

        # Right now, just pick the largest leds. Maybe could do some fancy size matching here?
        a_box = leds_a[0]
        b_box = leds_b[0]
        a_pos = box_center(a_box)
        b_pos = box_center(b_box)

        # Estimate craft position, height, angle
        x = (a_pos[0] + b_pos[0]) // 2
        y = (a_pos[1] + b_pos[1]) // 2
        h = 1.0
        a = 0

        # Calculate initial velocity vector
        self.pid_x.setpoint = img.shape[0] * land_pos[0]
        self.pid_y.setpoint = img.shape[1] * land_pos[1]
      
        dx = self.pid_x(x)
        dy = self.pid_x(y)
        v = np.array([dx,dy])

        # Normalize velocity vector
        norm = np.linalg.norm(v)
        v = v / norm

        # Apply height modifier and generic velocity multiplier
        v = (1-(1/h)) * 1.0 * v

        # TODO: Rotate velocity vector according to the craft angle

        # Debug drawing
        if debug_img is not None:
            cv2.circle(debug_img, a_pos, 4, (160,255,255), -1)
            cv2.circle(debug_img, b_pos, 4, (40,255,255), -1)

        return v

if __name__ == '__main__':
    lander = Lander()
    img = cv2.imread("test.png")
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    debug_img = img_hsv.copy()
    led_a = [[0,240,110],[10,255,255]]
    led_b = [[110,240,110],[130,255,255]]
    leds = lander.land_update(img_hsv, led_a, led_b, debug_img=debug_img)
    imshow_hsv(debug_img)
    print(leds)