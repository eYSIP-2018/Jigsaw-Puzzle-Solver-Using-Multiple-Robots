import cv2
import numpy as np
def template_matching(image, template):
    flag = 0
    # image = cv2.resize(image, (90, 90), interpolation = cv2.INTER_CUBIC)
    h1, w1, c1 = image.shape
    img = image.copy() # cv2.resize(image, (int(w1/6), int(h1/6)), interpolation=cv2.INTER_CUBIC)
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    template = cv2.cvtColor(template , cv2.COLOR_BGR2GRAY)    
    w, h = template.shape[::-1]
    for i in range(360):
        M = cv2.getRotationMatrix2D((h/2, w/2), i, 1)
        dst = cv2.warpAffine(template, M, (h, w))
        res = cv2.matchTemplate(img_gray, dst, cv2.TM_CCOEFF_NORMED)
        thresh = 0.75
        loc = np.where(res >= thresh)
        for pt in zip(*loc[::-1]):
            flag = 1
            break
    return flag

image = cv2.imread('testarena.jpg')
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

for cont in contours:
    c = 1
    area = cv2.contourArea(cont)
    if area > 700:
        # print(area)
        x, y, href, wref = cv2.boundingRect(cont)
        ima_ge = image[y:y + href, x:x + wref]
        cv2.rectangle(image, (x, y), (x + href, y + wref), (0, 255, 0), 2) 
              
        for k in range(9):
            temp_temp = cv2.imread('template' + str(k+1) + '.jpg')
            cv2.imshow('temp', temp_temp)
            cv2.waitKey(1)
            f = template_matching(temp_temp, ima_ge)
            if f == 1:
                cv2.putText(image, str(k+1), (x, y), 1, 1,(0,0,255), 2)
                break
                         
    cv2.imshow('arena', image)
    if cv2.waitKey(10) & 0xff == ord('q'):
        break
    c += 1    
# cv2.destroyAllWindows()

"""

"""    
