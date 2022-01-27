import cv2
import numpy as np
import matplotlib.pyplot as plt
import math


def isContourValid(contour, ellipse):
    M = cv2.moments(contour)
    eccentricity = ((M['m20']-M['m02'])**2-4*M['m11']**2) / \
        (M['m20']+M['m02'])**2
    (x, y), (width, height), angle = ellipse
    a = height / 2
    b = width / 2
    # eccentricity = math.sqrt(a**2-b**2)/a

    # area = cv2.contourArea(contour)
    area = a*b*math.pi
    # perimeter = math.pi * ( 3*(a+b) - math.sqrt( (3*a + b) * (a + 3*b) ) )
    kappa = cv2.arcLength(contour, True)**2/(2*math.pi * area)
    # kappa = perimeter**2/(2*math.pi * area)

    # print(area, eccentricity, kappa)

    if not (3000 <= area <= 1000000):
        return False

    # if abs(eccentricity) > 0.5:
    #     return False

    return True


def detectDates(raw):
    raw = cv2.cvtColor(raw, cv2.COLOR_BGR2RGB)
    im = cv2.cvtColor(raw, cv2.COLOR_BGR2GRAY)

    _, im2 = cv2.threshold(im, 30, 255, cv2.THRESH_BINARY)
    kernel = np.ones((5, 5))
    new2 = cv2.morphologyEx(im2, cv2.MORPH_OPEN, kernel, iterations=3)
    new3 = cv2.morphologyEx(new2, cv2.MORPH_CLOSE, kernel, iterations=3)
    plt.figure(3)
    plt.imshow(new3, cmap='gray')
    areas = []
    contours, heirarchy = cv2.findContours(
        new3, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    for i in range(0, len(contours)):
            ellipse = cv2.fitEllipse(contours[i])
            if isContourValid(contours[i], ellipse):
                areas.append(cv2.contourArea(contours[i]))
                cv2.drawContours(raw, contours, i, (i, 65, 232), 10)
                cv2.ellipse(raw, ellipse, (255, 0, 0), 6)
    print(areas)

    return raw




# raw = cv2.imread("date_image_3.png", cv2.IMREAD_COLOR)
# raw2 = detectDates(raw)
# raw = cv2.cvtColor(raw, cv2.COLOR_BGR2RGB)



# plt.figure(1)
# plt.imshow(raw)
# plt.figure(2)
# plt.imshow(raw2)
# # plt.figure(2)
# # plt.imshow(im, cmap='gray')
# # plt.figure(3)
# # plt.imshow(new3, cmap='gray')
# plt.show()


cap = cv2.VideoCapture('date_movie.mov')
while True:
    ret, frame = cap.read()
    cv2.imshow('video', frame)
    cv2.imshow('video2', detectDates(frame))

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
cap.release()
cv2.destroyAllWindows()

