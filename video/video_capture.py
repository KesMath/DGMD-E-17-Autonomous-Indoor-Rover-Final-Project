import cv2 as cv

# https://docs.opencv.org/4.x/dd/d43/tutorial_py_video_display.html

# [ WARN:0@0.298] global cap_v4l.cpp:982 open VIDEOIO(V4L2:/dev/video0): can't open camera by index
# [ERROR:0@0.303] global obsensor_uvc_stream_channel.cpp:156 getStreamChannelGroup Camera index out of range
class VideoCapture():
    def __init__(self):
        self.cap = None
        self.outfile = None

    def capture_video(self):
        self.cap = cv.VideoCapture(0)

        # Define the codec and create VideoWriter object
        fourcc = cv.VideoWriter_fourcc(*'XVID')

        self.outfile = cv.VideoWriter('output.avi', fourcc, 20.0, (640,  480))
    
        while self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                print("Can't receive frame (stream end?). Exiting ...")
                break
            gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            self.outfile.write(gray_frame)
            cv.imshow('frame', gray_frame)
            if cv.waitKey(1) & 0xFF == ord('q'): # terminate when 'q' key pressed
                self.free_video_resources()
                break

    def free_video_resources(self):      
        print("releasing resources...")
        self.cap.release() # close camera
        self.outfile.release() # close output file
        cv.destroyAllWindows()

def main():
    vid = VideoCapture()
    vid.capture_video()

if __name__ == '__main__':
    main()