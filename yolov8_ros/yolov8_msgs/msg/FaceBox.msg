# defines a Face recogntion result

# ID for tracking
int32 id
string name # for face recognition
float64 score

# 2D bounding box surrounding the object in pixels
yolov8_msgs/BoundingBox2D bbox