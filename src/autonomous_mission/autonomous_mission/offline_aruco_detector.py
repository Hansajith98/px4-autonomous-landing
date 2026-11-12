import cv2

image_path = "/tmp/gazebo_camera_1m.png"
image = cv2.imread(image_path)

if image is None:
    raise SystemExit(f"Could not read image: {image_path}")

gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

print(f"Image size: {image.shape[1]} x {image.shape[0]}")
print(f"OpenCV version: {cv2.__version__}")

if not hasattr(cv2, "aruco"):
    raise SystemExit("OpenCV ArUco module is unavailable.")

dictionary = cv2.aruco.Dictionary_get(
    cv2.aruco.DICT_4X4_50
)
parameters = cv2.aruco.DetectorParameters_create()

corners, ids, rejected = cv2.aruco.detectMarkers(
    gray,
    dictionary,
    parameters=parameters
)

if ids is None:
    print("No markers detected with DICT_4X4_50.")
    print(f"Rejected candidates: {len(rejected)}")
else:
    print("Detected marker IDs:", ids.flatten().tolist())
    print("Number of detected markers:", len(ids))

    cv2.aruco.drawDetectedMarkers(image, corners, ids)

    output_path = "/tmp/aruco_detection_result.png"
    cv2.imwrite(output_path, image)
    print(f"Annotated image saved: {output_path}")

    for i, corner in enumerate(corners):
        pts = corner.reshape(4, 2)
        center = pts.mean(axis=0)
        print(
            f"Marker {ids[i][0]} center: "
            f"x={center[0]:.1f}, y={center[1]:.1f}"
        )