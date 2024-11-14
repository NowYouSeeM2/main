import cv2
import rospy
from gaze_tracking import GazeTracking
from std_msgs.msg import Float32MultiArray


gaze = GazeTracking()
webcam = cv2.VideoCapture(0)
width = webcam.get(cv2.CAP_PROP_FRAME_WIDTH)
height = webcam.get(cv2.CAP_PROP_FRAME_HEIGHT)

def send_eye_position():
    pub = rospy.Publisher('set_arm', Float32MultiArray, queue_size=10)
    rospy.init_node('face_tracker', anonymous=True)
    #rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        ret, frame = webcam.read() 
        if not ret:
            continue
        gaze.refresh(frame)

        frame = gaze.annotated_frame()
        text = ""
        txt = ""

        if gaze.is_blinking():
            text = "Blinking"
        elif gaze.is_right():
            text = "Right"
        elif gaze.is_left():
            text = "Left"
        elif gaze.is_center():
            text = "Center"
        
        if gaze.is_top():
            txt = "Top"
        elif gaze.is_bottom():
            txt = "Bottom"
        elif gaze.is_senter():
            txt = "Senter"
    
    
        left_pupil = gaze.pupil_left_coords()
        right_pupil = gaze.pupil_right_coords()
    
        if left_pupil is not None and right_pupil is not None:
            
            senter_point = tuple((l + r)/2 for l, r in zip(left_pupil, right_pupil))
            if 0 <= senter_point[0] <= width and 0 <= senter_point[1] <= height:

                # arm_x = int((senter_point[0] / width) * 500)  # X 좌표 변환
                # arm_y = int((senter_point[1] / height) * 500)  # Y 좌표 변환
                # arm_z = 100  # Z 축은 임의의 고정 값 사용
                # grip_degree = 90  # 그립 각도 고정
                # gripper = 1000  # 그리퍼 고정
            
                # 메시지 작성 및 전송
                #arm_command = Float32MultiArray(data=[arm_x, arm_y, arm_z, grip_degree, gripper, 1000])
                arm_command = Float32MultiArray(data=[senter_point[0], senter_point[1], width, height])
                rospy.loginfo(f"Tracking face at: {arm_command.data}")
                pub.publish(arm_command)
    


        cv2.imshow("Arsenal Win", frame)

        if cv2.waitKey(1) == 27:
            break
        #rate.sleep()
    webcam.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        send_eye_position()
    except rospy.ROSInitException:
        pass
