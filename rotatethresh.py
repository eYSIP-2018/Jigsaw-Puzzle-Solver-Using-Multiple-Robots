import cv2
import numpy as np

flag = 0
image = cv2.imread('Testimg.jpg')
h1, w1, c1 = image.shape
img = cv2.resize(image,(int(w1/6),int(h1/6)), interpolation= cv2.INTER_CUBIC)
img2 = img.copy()
img_gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
template = cv2.imread('test-5.jpg',0)
#cv2.imshow("testimg",template)
#cv2.waitKey(0)
#cv2.destroyAllWindows()
w,h = template.shape[::-1]
for i in range(360):
    M = cv2.getRotationMatrix2D((h/2,w/2),i,1)
    dst = cv2.warpAffine(template,M,(h,w))
    #cv2.imshow("result",dst)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()

    res = cv2.matchTemplate(img_gray,dst,cv2.TM_CCOEFF_NORMED)
    thresh = .91

    loc = np.where(res>=thresh)
    for pt in zip(*loc[::-1]):
        cv2.rectangle(img2,pt,(pt[0]+w,pt[1]+h),(0,255,0),2)
        cv2.imshow("result",img2)
        flag = 1
        break

if flag is 1:
    print("templet found")
else:
    print("templet not found")
