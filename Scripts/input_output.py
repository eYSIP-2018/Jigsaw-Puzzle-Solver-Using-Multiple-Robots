import cv2
import numpy as np
import imutils
import time

def template_matching(image, template):
    flag = 0
    # image = cv2.resize(image, (90, 90), interpolation = cv2.INTER_CUBIC)
    h1, w1, c1 = image.shape
    img = image.copy()  # cv2.resize(image, (int(w1/6), int(h1/6)), interpolation=cv2.INTER_CUBIC)
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
    w, h = template.shape[::-1]
    thresh = 0
    b = 0
    for i in range(361):
        
        # M = cv2.getRotationMatrix2D((h/2, w/2), i, 1)
        dst = imutils.rotate_bound(template, i)  # cv2.warpAffine(template, M, (h, w))
        res = cv2.matchTemplate(img_gray, dst, cv2.TM_CCOEFF_NORMED)
        for p in zip(*res):
            for k in p:
                if k > thresh:
                    thresh = k
                    b = i
                 
    """loc = np.where(res == thresh)
    for pt in zip(*loc[::-1]):
        flag = 1
        b = i
        break"""
    return thresh, b

ini_time = time.time()
image = cv2.imread('sample_arena.jpg')
#image = cv2.resize(image, (640, 480), interpolation = cv2.INTER_CUBIC)
final_image = cv2.imread('jigsaw_3.jpg')

h , w , c = final_image.shape
n=1
for i in range(3):
    for j in range(3):
        temp = final_image[int(i*h/3):int((i+1)*h/3) , int(j*w/3):int((j+1)*w/3)]
        temp = cv2.resize(temp, (30, 30), interpolation = cv2.INTER_CUBIC)
        cv2.imwrite('template'+ str(n) + '.jpg', temp)
        n += 1

# final_copy = final_image.copy()
# final_image = cv2.resize(final_image, (90, 90), interpolation = cv2.INTER_CUBIC)

gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
ret, thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY_INV)

img , contours , hier = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
cv2.imshow('thresh', thresh)
blocks = []
for cont in contours:
    c = 1
    ki = 0
    ai = 0
    area = cv2.contourArea(cont)
    if area > 700:
        # print(area)
        x, y, href, wref = cv2.boundingRect(cont)
        ima_ge = image[y:y + href, x:x + wref]
        temp_thresh = 0
        
        for k in range(9):
            temp_temp = cv2.imread('template' + str(k+1) + '.jpg')
            cv2.imshow('temp', temp_temp)
            cv2.waitKey(1)
            thresh, a = template_matching(ima_ge , temp_temp)
            if thresh > temp_thresh :
                ki = k
                ai = a
                temp_thresh = thresh
                
        blocks.append((ki + 1, int(x + 0.5*href), int(y + 0.5*wref)))
        cv2.rectangle(image, (x, y), (x + href, y + wref), (0, 255, 0), 2)
        cv2.putText(image, str(ki + 1) +','+ str(ai), (x, y), 1, 2, (0, 0, 255), 2)
                
    #    print(thresh)
    cv2.imshow('arena', image)
    if cv2.waitKey(1) & 0xff == ord('q'):
        break
    c += 1
fin_time = time.time()
print(blocks)
print("time required = %d" % (fin_time-ini_time))
cv2.destroyAllWindows()

"""

"""
