import cv2 as cv

# https://docs.opencv.org/3.4/d4/d15/group__videoio__flags__base.html
# https://docs.opencv.org/4.x/de/db1/group__videoio__registry.html
# https://github.com/opencv/opencv/issues/19527


# class VideoCapture():
#     def __init__(self):
#         self.cap = None
#         self.outfile = None

#     def capture_video(self):
#         self.cap = cv.VideoCapture(cv.CAP_V4L2) #cv2.CAP_V4L2

#         # Define the codec and create VideoWriter object
#         fourcc = cv.VideoWriter_fourcc(*'XVID')

#         self.outfile = cv.VideoWriter('output.avi', fourcc, 20.0, (640,  480))

#         while self.cap.isOpened():
#             ret, frame = self.cap.read()
#             if not ret:
#                 print("Can't receive frame (stream end?). Exiting ...")
#                 break
#             frame = cv.flip(frame, 0)
#             # write the flipped frame
#             self.outfile.write(frame)
#             cv.imshow('frame', frame)
#             if cv.waitKey(1) == ord('q'):
#                 break

#     def free_video_resources(self):      
#         # Release everything if job is finished
#         self.cap.release()
#         self.outfile.release()
#         cv.destroyAllWindows()

# def main():
#     vid = VideoCapture()
#     vid.capture_video()
#     vid.free_video_resources()

# if __name__ == '__main__':
#     main()



def capture_video():
    cap = cv.VideoCapture(cv.CAP_V4L2) #cv2.CAP_V4L2

    # Define the codec and create VideoWriter object
    fourcc = cv.VideoWriter_fourcc(*'XVID')

    outfile = cv.VideoWriter('output.avi', fourcc, 20.0, (640,  480))

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
        frame = cv.flip(frame, 0)
        # write the flipped frame
        outfile.write(frame)
        cv.imshow('frame', frame)
        if cv.waitKey(1) == ord('q'):
            break
    
    cap.release()
    outfile.release()
    cv.destroyAllWindows()

def main():
    capture_video()