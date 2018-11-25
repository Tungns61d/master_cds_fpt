import numpy as np
import cv2
import matplotlib.pyplot as plt

polygons = np.array([
            #[(100,height),(width,height),(width,267),(320,170)]
            #[(30,height),(450,height),(280,130)]
            [(210,720), (550,450), (717,450),(1280,720)]
        ])





def canny(image):
        image_gray = cv2.cvtColor(image,cv2.COLOR_RGB2GRAY)
        blur = cv2.GaussianBlur(image_gray,(5,5),0)
        canny = cv2.Canny(blur,50,250)
        cv2.imshow('Canny',canny)
        cv2.waitKey()
        return canny



#masked
def region_of_interest(image, polygons):
#         height = image.shape[0]
#         width = image.shape[1]
#         x = [210, 550,717 ,1280];
#         y = [720, 450, 450, 720];
#         polygons = np.array([
#             #[(100,height),(width,height),(width,267),(320,170)]
#             #[(30,height),(450,height),(280,130)]
#             [(210,720), (550,450), (717,450),(1280,720)]
#         ])
        mask = np.zeros_like(image)
        cv2.fillPoly(mask,polygons,255)
        masked_image = cv2.bitwise_and(image,mask)
        return masked_image


def find_slopes_lines(image):
    canny_image = canny(image)
    crop_image = region_of_interest(canny_image,polygons)
    lines = cv2.HoughLinesP(crop_image,2,np.pi/180,50,np.array([]),minLineLength=20,maxLineGap=100)
    ## y=mx+b
    line_image=np.zeros_like(image)
    #print(lines.shape)
    x = lines[:,:,0:2]
    x = x.reshape(len(lines),2)
    y = lines[:,:,2:4]
    y = y.reshape(len(lines),2)
    slopes = []
    lines_update = []
    threshold = 0.35
    for l in range(len(lines)):
        if (y[l][0]-x[l][0] ==0):
            slope = 1000 # //oy
        else:
            slope = (y[l][1]-x[l][1])/(y[l][0]-x[l][0])

        if (abs(slope) > threshold):
            slopes.append(slope)
            lines_update.append(lines[l])
    return slopes,lines_update



def split_hough_point(lines_update):
    lines_update = np.array(lines_update)
    #print(lines_update.shape,'update lines')
    xx = lines_update[:,:,0:2]
    xx = xx.reshape(len(lines_update),2)
    yy = lines_update[:,:,2:4]
    yy = yy.reshape(len(lines_update),2)
    return xx,yy

def detect_left_right(image):
    

    slopes,lines_update = find_slopes_lines(image)
    #print(slopes)
    #print(len(lines_update),'lines_update')
    first,second = split_hough_point(lines_update)
    center = image.shape[1]/2
    right_lines = []
    left_lines = []
    tag = []
    #global tagleft,tagright
    for l in range(len(lines_update)):
        #print(l,': Index')
        if ((slopes[l] > 0) and (first[l][0] > center+50) and (second[l][0] > center+50)) :
            right_lines.append(lines_update[l])
            tagright = 1;
            tagleft = 0;
        else:# ((slopes[l] < 0)  and (first[l][0] < center) and (second[l][0] < center)) :
            left_lines.append(lines_update[l])
            tagleft = 1;
            tagright = 0;
    right_lines = np.array(right_lines)
#     right_lines = right_lines.reshape(right_lines.shape[0],4)
    left_lines  = np.array(left_lines)
#     left_lines  = left_lines.reshape(left_lines.shape[0],4)
    return right_lines,left_lines


def detect(image):
    right_lines, left_lines = detect_left_right(image)
    first_r,second_r = split_hough_point(right_lines)
    first_l,second_l = split_hough_point(left_lines)
    x_r = np.array((first_r[:,0],second_r[:,0]))
    x_r = x_r.reshape(x_r.shape[0]*x_r.shape[1])
    y_r = np.array((first_r[:,1],second_r[:,1]))
    y_r = y_r.reshape(y_r.shape[0]*y_r.shape[1])
    
    
    # print(first_l,'first')
    # print(second_l,'second')
    
    x_l = np.array((first_l[:,0],second_l[:,0]))
    y_l = np.array((first_l[:,1],second_l[:,1]))
    # print(x_l,'x_ shape')
    # print(y_l.shape,'y_ shape')
    x_l = x_l.reshape(x_l.shape[0]*x_l.shape[1])
    
    y_l = y_l.reshape(y_l.shape[0]*y_l.shape[1])
    plt.figure()
    plt.scatter(x_r,y_r,color='red')
    plt.scatter(x_l,y_l,color='blue')
    plt.imshow(image)
    # print(x_r.shape,'xr')
    # print(y_r,'yr')
    
    
    if (len(x_r)>0):
        pol = np.polyfit(x_r,y_r,1)
        rm  = pol[0]
        rb  = pol[1]
    else:
        rm = 1
        rb =1
        
    if (len(x_l)>0):
        pol = np.polyfit(x_l,y_l,1)
        lm  = pol[0]
        lb  = pol[1]
    else:
        lm = 1
        lb =1
    line_image=np.zeros_like(image)
    y1 = image.shape[0]
    y2 = y1/2
    right_x1 = (y1 - rb) / rm;
    right_x2 = (y2 - rb) / rm;
    left_x1 = (y1 - lb) / lm;
    left_x2 = (y2 - lb) / lm;
    
    polygons = np.array([
        #[(100,height),(width,height),(width,267),(320,170)]
        #[(30,height),(450,height),(280,130)]
      #   [(250, 700), (550,450), (717,450),(1280,720)]
      [(int(left_x1), int(y1)), (int(left_x2), int(y2)),(int(right_x2), int(y2)),(int(right_x1), int(y1))]
    ])

    plt.figure()
    mask = np.zeros_like(image)
    cv2.fillPoly(mask,polygons,255)
    masked_image = cv2.bitwise_or(mask,image)
    plt.imshow(masked_image, cmap='gray')

    print(rm,'slope right')
    if (rm <= .70):
        cv2.putText(masked_image, 'LEFT', (150, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)

    elif (lm >= .68):
        cv2.putText(masked_image, 'RIGHT', (150, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)

    else:
        cv2.putText(masked_image, 'STRAIGH', (150, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)



    cv2.imshow('Detect',masked_image)
    cv2.waitKey(1)
def main():

    # image = cv2.imread('../Image/lane.png')
    cv2.namedWindow('View')
    cv2.namedWindow('Canny')
    cv2.namedWindow('Detect')
    
    
    # #Start
    # cv2.imshow('View',image)
    # cv2.waitKey(1)
    # detect(image) 
    # cv2.destroyAllWindows()


    #cap = cv2.VideoCapture('challenge_video.mp4')
    cap = cv2.VideoCapture('detect_lane.mp4')
    while (cap.isOpened()):
        cv2.startWindowThread()
        ret, frame = cap.read()
        cv2.imshow('View',frame)
        cv2.waitKey(500)
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break
        else:
            detect(frame)
    cap.release()
    cv2.destroyAllWindows()
        
    
if __name__ == '__main__':
    main()