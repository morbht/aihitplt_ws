#
#!usr/bin/env python3 
# coding = utf-8

import cv2 

if __name__ == '__main__':
	path = "/home/aihit/aihitplt_ws/src/aihitplt_visual/opencv/test.jpg"
	img = cv2.imread(path)
	string = "退出"
	while True:
		cv2.imshow('test',img)
		action = cv2.waitKey(10) & 0xff
		if action == ord('q') or action == 113 :
			print(f"{string}")
			break
		
	img.release()

	cv2.destroyAllWindows()

