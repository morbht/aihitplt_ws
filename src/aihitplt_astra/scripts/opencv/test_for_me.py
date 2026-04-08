import cv2 as cv
if __name__ == '__main__':
	img = cv.imread('test.jpg')
	cv.imwrite("test_new.jpg",img)
	new_img = cv.imread('test_new.jpg')
	while True:
		cv.imshow('frame',img)
		cv.imshow('new_frame',new_img)
		action = cv.waitKey(10) & 0xFF
		if action == ord('q') or action == 113:
			break
	img.release()
	cv.destroyAllWindows()
