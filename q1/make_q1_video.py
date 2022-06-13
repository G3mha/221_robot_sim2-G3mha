import math
import cv2
import numpy as np
from random import triangular

class CannonShot:
    def __init__(self, x, h, v = 300):
        self.h = h
        self.x = x
        self.y = img_h
        self.v = v #pixels por segundo

    def update(self, dt):
        self.y -= self.v * dt # dt is in secs

    def draw(self, bgr):
        cv2.line(bgr, (int(self.x), int(self.y-self.h)), (int(self.x), int(self.y)), cannon_bgr, 5)


if __name__ == "__main__":

    # constants
    img_w = 800
    img_h = 600
    timespan = 30
    fps = 30
    frames = timespan * fps

    cannon_bgr = (255, 255, 0)
    alien_bgr = (0, 128, 255)

    # Alien image
    alien_img = cv2.imread("aliengray.png", cv2.IMREAD_COLOR)
    alien_img[:,:,0] = cv2.scaleAdd(alien_img[:,:,0],alien_bgr[0]/255,np.zeros_like(alien_img[:,:,0]))
    alien_img[:,:,1] = cv2.scaleAdd(alien_img[:,:,1],alien_bgr[1]/255,np.zeros_like(alien_img[:,:,1]))
    alien_img[:,:,2] = cv2.scaleAdd(alien_img[:,:,2],alien_bgr[2]/255,np.zeros_like(alien_img[:,:,2]))
    cv2.imwrite("alien_color.png", alien_img)

    # Cannon laser
    cannon_interval = 0.5 #secs
    cannon_h = img_h//20
    cannon_dx = img_w//3
    cannon_shots = []

    freq_x_cannon = 10/fps
    cannon_x_center = img_w//2
    cannon_x_ampl = img_w/2

    # Point
    #freq_x_point = 1/fps # 1/n_frames
    #freq_y_point = 2/fps # 1/n_frames
    #center_x = img_w//2
    #center_y = (img_h*2)//5
    #ampl_x = img_w//2 - 60
    #ampl_y = img_h//5

    # Alien
    freq_x_alien = 0.7/fps
    ampl_x_alien = img_w//2
    center_x_alien = img_w//2
    alien_y0 = 50
    vel_y_alien = 10/fps # pixel per frame


    # independent movement variables
    t_cannon = 0
    t_point_x = 0
    t_point_y = 0
    t_alien = 0

    last_shot = -1
    fourcc = cv2.VideoWriter_fourcc("H","2","6","4")
    writer = cv2.VideoWriter("laserdefense.mp4",fourcc,30,(img_w,img_h),True)
    for t in range(frames):
        bgr = np.zeros((img_h,img_w,3), dtype=np.uint8)

        # changing variables - cannon
        freq_x = freq_x_cannon
        cannon_x = cannon_x_ampl * math.sin(2*math.pi*freq_x*t_cannon) + cannon_x_center

        # Shot
        shot_time = t/fps/cannon_interval 
        if abs(shot_time - math.floor(shot_time) < 0.01) :
            cannon_shots.append(CannonShot(cannon_x,cannon_h))

        # changing variables - point
        #freq_x = freq_x_point
        #point_x = ampl_x * math.sin(2*math.pi*freq_x*t_point_x) + center_x
        #freq_y = freq_y_point
        #point_y = ampl_y * math.sin(2*math.pi*freq_y*t_point_y) + center_y

        #changing variables - alien
        freq_x = freq_x_alien
        alien_x = ampl_x_alien * math.sin(2*math.pi*freq_x*t_alien) + center_x_alien
        alien_y = alien_y0 + vel_y_alien * t_alien

        # update independent variables
        t_cannon += triangular(0,3)
        t_point_x += triangular(0,3)
        t_point_y += triangular(0,3)
        t_alien += triangular(0,3)

        # drawing - alien
        i_min = max(0,int(alien_y) - alien_img.shape[0]//2)
        j_min = max(0,int(alien_x) - alien_img.shape[1]//2)
        i_min = min(bgr.shape[0] - alien_img.shape[0], i_min)
        j_min = min(bgr.shape[1] - alien_img.shape[1], j_min)

        bgr[i_min:i_min+alien_img.shape[0], j_min:j_min+alien_img.shape[1],:] = alien_img 

        # drawing - cannon shot
        # Cannon shots:
        for shot in cannon_shots:
            shot.draw(bgr)
            shot.update(1.0/fps)
            if(shot.y < 0):
                del shot


        # Show image
        cv2.imshow("Game", bgr)
        cv2.imshow("Alien", alien_img)
        writer.write(bgr)
        if cv2.waitKey(1000//fps) == ord('q'):
            break

    cv2.destroyAllWindows()
    writer.release()






