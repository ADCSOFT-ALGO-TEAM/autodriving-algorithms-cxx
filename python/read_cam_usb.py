import cv2
import time
import os

def cache_image():
    cap = cv2.VideoCapture(2)
    start_time = time.time()
    save_img = False
    index = 0
    while True:
        end_time = time.time()
        diff_time = end_time - start_time
        ret, frame = cap.read()
        show_frame = frame.copy()
        
        ret_, corners = cv2.findChessboardCornersSB(frame,(7,6),flags=cv2.CALIB_CB_EXHAUSTIVE+cv2.CALIB_CB_ACCURACY)
        if ret_:
            cv2.drawChessboardCorners(show_frame,(7,6),corners,True)
        if save_img:
            save_file = str(index)+".png"
            index += 1
            print('save image: {}'.format(save_file))
            cv2.imwrite("show_"+save_file, show_frame)
            cv2.imwrite(save_file, frame)
        cv2.imshow('Video', show_frame)
        key = cv2.waitKey(10)
        if key == ord('r'):
            save_img = True
        if key == ord('c'):
            save_img = False
        if key == ord('s'):
            save_file = str(index) + '.png'
            index += 1
            print('save image: {}'.format(save_file))
            cv2.imwrite("show_"+save_file, show_frame)
            cv2.imwrite(save_file, frame)
        if 27 == key:
            break
    cap.release()
    cv2.destroyAllWindows()
def deal_image():
    path = '/home/pengccheng/Data/work/ADC/autodriving-algorithms-cxx/python/calib'
    files = os.listdir(path)
    text = open('calibartions.txt', 'w')
    for f in files:
        if os.path.splitext(f)[1] in ['.png', '.jpeg', '.jpg']:
            img_path = os.path.join(path, f)
            text.write(img_path)
            text.write('\n')
if __name__ == '__main__':
    deal_image()