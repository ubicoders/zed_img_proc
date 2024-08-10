import cv2
import pyzed.sl as sl

def main():
    # Create a Camera object
    zed = sl.Camera()

    # Create a Configuration Parameters object
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720  # Use HD720 video mode
    init_params.camera_fps = 60  # Set FPS at 30

    # Open the camera
    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print("Failed to open the ZED camera")
        return

    # Create a Mat object for the left and right images
    image_left = sl.Mat()
    image_right = sl.Mat()

    # Runtime parameters
    runtime_parameters = sl.RuntimeParameters()
    
    key = ''
    while key != 27:  # Press 'Esc' to exit
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            # Retrieve left image
            zed.retrieve_image(image_left, sl.VIEW.LEFT)
            # Retrieve right image
            zed.retrieve_image(image_right, sl.VIEW.RIGHT)

            # Convert to OpenCV format
            left_image_ocv = image_left.get_data()
            right_image_ocv = image_right.get_data()

            # Display the images
            cv2.imshow("Left Image", left_image_ocv)
            cv2.imshow("Right Image", right_image_ocv)

            # Wait for a key press
            key = cv2.waitKey(10)

    # Close the camera
    zed.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()