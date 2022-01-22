from cv_bridge import CvBridge
import os,cv2
bridge = CvBridge()

imageFile1 = os.path.join('/code/solution/src/object_detection/src/mav0/cam0/data','1403636579763555584.png')
print(imageFile1)
img = cv2.imread(imageFile1, cv2.IMREAD_COLOR)
print(img)

with open('/code/solution/src/object_detection/src/mav0/cam0/data.csv') as file:
    from cv_bridge import CvBridge
    import os,cv2
    bridge = CvBridge()
    print(file.readline())

    lines = file.readlines()

    for line in lines:
        tokens = line.split(',')
        entry = {}
        entry["timestamp"] = tokens[0]
        entry["image"] = tokens[1].strip()
        imageFile = os.path.join('/code/solution/src/object_detection/src/mav0/cam0/data',entry["image"])
        print(imageFile)
        img = cv2.imread(imageFile, cv2.IMREAD_COLOR)
        print(img)
        entry["compressed"] = bridge.cv2_to_compressed_imgmsg(img)