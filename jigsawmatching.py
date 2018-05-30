import cv2
import numpy as np
x = [i for i in range(9)]
image = cv2.imread("Testimg.jpg")
h1, w1, c1 = image.shape
img = cv2.resize(image,(int(w1/6),int(h1/6)), interpolation= cv2.INTER_CUBIC)
cv2.imshow("image",img)
h, w, c = img.shape
###########################################
template = []
###########################################
crop_img1 = img[0:int(h/3), 0:int(w/3)]
template.append(cv2.resize(crop_img1,(int(w/3),int(h/3)), interpolation= cv2.INTER_CUBIC))
##########################################
crop_img2 = img[0:int(h/3), int(w/3):int(2*w/3)]
template.append(cv2.resize(crop_img2,(int(w/3),int(h/3)), interpolation= cv2.INTER_CUBIC))
###########################################
crop_img3 = img[0:int(h/3), int(2*w/3):int(w)]
template.append(cv2.resize(crop_img3,(int(w/3),int(h/3)), interpolation= cv2.INTER_CUBIC))
###########################################
crop_img4 = img[int(h/3):int(2*h/3), 0:int(w/3)]
template.append(cv2.resize(crop_img4,(int(w/3),int(h/3)), interpolation= cv2.INTER_CUBIC))
###########################################
crop_img5 = img[int(h/3):int(2*h/3), int(w/3):int(2*w/3)]
template.append(cv2.resize(crop_img5,(int(w/3),int(h/3)), interpolation= cv2.INTER_CUBIC))
#############################5##############
crop_img6 = img[int(h/3):int(2*h/3), int(2*w/3):int(w)]
template.append(cv2.resize(crop_img6,(int(w/3),int(h/3)), interpolation= cv2.INTER_CUBIC))
###########################################
crop_img7 = img[int(2*h/3):int(h), 0:int(w/3)]
template.append(cv2.resize(crop_img7,(int(w/3),int(h/3)), interpolation= cv2.INTER_CUBIC))
###########################################
crop_img8 = img[int(2*h/3):int(h), int(w/3):int(2*w/3)]
template.append(cv2.resize(crop_img8,(int(w/3),int(h/3)), interpolation= cv2.INTER_CUBIC))
###########################################
crop_img9 = img[int(2*h/3):int(h), int(2*w/3):int(w)]
template.append(cv2.resize(crop_img9,(int(w/3),int(h/3)), interpolation= cv2.INTER_CUBIC))
###########################################
for i, j in enumerate(template):
	cv2.imshow("temp"+"-"+str(i), j)
	cv2.imwrite("test"+"-"+str(i)+".jpg", j)
###########################################
jigsaw = cv2.imread("test-1.jpg")
for i in template:
    difference = cv2.subtract(jigsaw,i)
    result =  np.any(difference)
    if result is True:
        print("same")
    else:
        print("different")
