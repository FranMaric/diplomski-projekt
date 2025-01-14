import cv2
import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped


# Global variables
trigger_event = False

depth_image = None  # To store the depth image
camera_info = {'fx': 525.0, 'fy': 525.0, 'cx': 319.5, 'cy': 239.5}  # Replace with your camera's parameters

# Aktivira se kada se preko nekog topica posalje komanda 
def event_callback(msg):
    global trigger_event
    if msg.data == "show_image": 
        trigger_event = True

# Depth image callback
bridge = CvBridge() # CHATGPT rekao da se mora koristiti bridge
def depth_callback(msg):
    global depth_image
    depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")


# Function to project a 2D pixel point into 3D using depth
def pixel_to_3d(pixel, depth_data, fx, fy, cx, cy):
    x, y = pixel
    z = depth_data[y, x]
    X = (x - cx) * z / fx
    Y = (y - cy) * z / fy
    Z = z
    print(f"Pixel ({x}, {y}) -> 3D Point ({X}, {Y}, {z})")
    return np.array([X, Y, Z])


# Mouse callback function to get selected points
def select_point(event, x, y, flags, param):
    image, points = param['image'], param['points']
    if event == cv2.EVENT_LBUTTONDOWN:
        points.append((x, y))
        color = (0, 0, 255) if len(points) == 1 else (255, 0, 0)  # Red for the first, Blue for others
        cv2.circle(image, (x, y), 5, color, -1)
        cv2.imshow("Select 4 Points", image)
        print(f"Point selected: ({x}, {y})")
        if len(points) == 4:  # Stop after 4 points
            cv2.destroyAllWindows()
            
            
def compute_6dof_pose(points_3d, selected_points, fx, fy, cx, cy):
  
    #vraca 6dof poziciju i orijentaciju odabranih tocke 
    # PARAMETRI
    # points_3d su 3dimenzionalne verzije odabranih 2d tocaka
    # selected points su 2d tocke
    # fx i fy fokalne duljine 
    # cx i cy sredina slike 

    # VRACA
    #     dict: A dictionary containing:
    #         - 'success' (bool): Whether SolvePnP succeeded.
    #         - 'rvec' (numpy.ndarray): Rotation vector.
    #         - 'tvec' (numpy.ndarray): Translation vector.
    #         - 'position' (numpy.ndarray): Transformed 6DOF position (x, y, z) of the first point.
    #         - 'orientation' (numpy.ndarray): Orientation in degrees (rotation vector).
    
 
    object_points = np.array(points_3d, dtype=np.float32)
    image_points = np.array(selected_points, dtype=np.float32)
    
    # Define the camera intrinsic matrix
    camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)
    dist_coeffs = np.zeros(5)  # Assuming no lens distortion

    # SolvePnP to compute rotation (rvec) and translation (tvec)
    success, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)
    
    if success:
        print("Rotation vector (rvec):", rvec)
        print("Translation vector (tvec):", tvec)

        # Convert rotation vector to a rotation matrix
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        
        # Transform the first 3D point into 6DOF coordinates
        first_point_3d = points_3d[0].reshape(3, 1)  # Convert to column vector
        transformed_point = np.dot(rotation_matrix, first_point_3d) + tvec
        print("Original 3D point (red):", first_point_3d.flatten())
        print("Transformed 6DOF Position (x, y, z):", transformed_point.flatten())

        # Include orientation (as Euler angles for interpretability)
        rvec_as_degrees = np.rad2deg(rvec).flatten()
        print("6DOF Orientation (Rotation Vector in degrees):", rvec_as_degrees)

        # Prepare the final 6DOF pose
        transformed_6dof = {
            "success": True,
            "rvec": rvec,
            "tvec": tvec,
            "position": transformed_point.flatten(),
            "orientation": rvec_as_degrees
        }
    else:
        print("solvePnP failed.")
        transformed_6dof = {
            "success": False,
            "rvec": None,
            "tvec": None,
            "position": None,
            "orientation": None
        }

    return transformed_6dof


def publish_6dof_pose(result, pose_publisher, frame_id="camera_frame"):
    # SALJE RESULTAT DALJE LUKI NA OBRADU 
    if result["success"]:
        print("Transformed 6DOF Pose:")
        print("Position:", result["position"])
        print("Orientation (rotation vector in degrees):", result["orientation"])

        # Create PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = frame_id

        # Assign position (translation vector tvec)
        pose_msg.pose.position.x = result["position"][0]
        pose_msg.pose.position.y = result["position"][1]
        pose_msg.pose.position.z = result["position"][2]

        # Assign orientation directly as a rotation vector (degrees)
        pose_msg.pose.orientation.x = result["orientation"][0]
        pose_msg.pose.orientation.y = result["orientation"][1]
        pose_msg.pose.orientation.z = result["orientation"][2]
        pose_msg.pose.orientation.w = 0  # Not meaningful here but required by PoseStamped

        # Publish the pose
        pose_publisher.publish(pose_msg)
        print("Published 6DOF pose (position + rvec) to /drone/landing_pose")
    else:
        print("6DOF pose computation failed.")


# Main function
def main():
    global trigger_event, depth_data

    depth_data = np.load("/Users/josiphanak/Documents/EDUCATION/UNI/FER/MA/2.god/1.SEM/PROJEKT/code/fran-podaci/images/depth_image_distance_19.npy") 
    
  
    # Initialize ROS node
    rospy.init_node("drone_video_processor", anonymous=True)
    rospy.Subscriber("/drone/event_topic", String, event_callback) #ovo je trigger event
    rospy.Subscriber("/camera/depth/image_raw", Image, depth_callback) #ovo ako cemo morat dobit duljinu
    
    # za slanje dalje rezultata luki
    pose_publisher = rospy.Publisher("/drone/landing_pose", PoseStamped, queue_size=10)

    # ovdje umjesto 0 bi trebao ici putanja stream od drona 
    # ili samo da kada se lupi topic se slika trenutna slika bez kamere cijelo vrijeme upaljene 
    video_stream = cv2.VideoCapture(0)

    if not video_stream.isOpened():
        print("Error: Unable to open video stream")
        return

    while not rospy.is_shutdown():
        ret, frame = video_stream.read()
        if not ret:
            print("Error: Unable to read frame")
            break

        # Display the video stream
        cv2.imshow("Drone Video Stream", frame)

        # Dobije se trigger preko nekog topica da je vrijeme za sletit
        if trigger_event:
        
            image = frame #TRENUTNI FRAME JE SADA SLIKE GDJE SE ODABIRE TOCKA ZA SLIJETANJE
            cv2.imshow("Triggered Image", image)

            fx, fy = 600, 600  # ovo moramo saznat koje su vrijednosti dronove kamere 
            cx, cy = image.shape[1] // 2, image.shape[0] // 2  # centar slike 
            display_image = image.copy()
            
            
            # Set up mouse to select a landing point
            selected_points = []
            cv2.setMouseCallback("Select 4 Points", select_point, {'image': display_image, 'points': selected_points})
            print("Please select 4 points.")
            while len(selected_points) < 4:
                cv2.waitKey(1)

            # Convert selected 2D points to 3D points
            # moramo nekako saznat podatke o dubini
            points_3d = [pixel_to_3d(p, depth_data, fx, fy, cx, cy) for p in selected_points]
            print("3D Points:", points_3d)
            
            
            result = compute_6dof_pose(points_3d, selected_points, fx, fy, cx, cy)

            # Check if the computation was successful
            if result["success"]:
                print("Transformed 6DOF Pose:")
                print("Position:", result["position"])
                print("Orientation (degrees):", result["orientation"])
                
                
                
            else:
                print("6DOF pose computation failed.")
                                    
            publish_6dof_pose(result, pose_publisher)  # SALJI LUKI DALJE REZULTAT                         
                              
        if cv2.waitKey(1) & 0xFF == ord('q'): #prekini program ako se lupi q  
            break

    # Cleanup
    video_stream.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass