#include "Vision/vision.h"
#include "Utilities/utilities.h"
#include <math.h>

//saturation values can remain constant
#define SATURATE_LOW 80
#define SATURATE_HIGH 255

#define YELLOW_MIN 27
#define YELLOW_MAX 37
#define YELLOW_VAL_MIN 218
#define YELLOW_VAL_MAX 255

#define GREEN_MIN 43
#define GREEN_MAX 81

#define BLUE_MIN 80
#define BLUE_MAX 120
#define BLUE_VAL_LOW 218
#define BLUE_VAL_HIGH 255

#define RED_MIN 0
#define RED_MAX 10
#define RED_VAL_LOW 218
#define RED_VAL_HIGH 240

#define PURPLE_MIN 121
#define PURPLE_MAX 136
#define PURPLE_VAL_LOW 218
#define PURPLE_VAL_HIGH 240

#define WHITE_MIN 0
#define WHITE_MAX 23

#define WHITE_VAL_LOW 205
#define WHITE_VAL_HIGH 240

#define PINK_MIN 0
#define PINK_MAX 30
#define PINK_VAL_LOW 210
#define PINK_VAL_HIGH 255
#define PINK_SAT_LOW 0
#define PINK_SAT_HIGH 50

#define PINK_GREEN_MAX 215



#define PI 3.14159265

#define VERT_BOUND 70
#define HOR_BOUND 225

//global parameters
int iLowH = 0;
int iHighH = 179;

int iLowS = 0;
int iHighS = 255;

int iLowV = 0;
int iHighV = 255;



Mat Vision::smoothing(Mat img, int radius)
{
    Mat blurredImage;
    for(int i = 1; i < 10; i = i+2)
    {
        GaussianBlur(img, blurredImage, Size(i, i), 0, 0);
    }

     return img;
}

void colorSlider(Mat img)
{
    namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
    //Create trackbars in "Control" window
    cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
    cvCreateTrackbar("HighH", "Control", &iHighH, 179);

    cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("HighS", "Control", &iHighS, 255);

    cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("HighV", "Control", &iHighV, 255);
    Mat imgOriginal;
    imgOriginal = img;


    Mat imgHSV;
    cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

    Mat imgThresholded;

    inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    imwrite( "../Gray_Image.jpg", imgThresholded );
    imwrite( "../original.jpg", img );

    imshow("changed", imgThresholded);
}



//hleper function to get the lines
vector<Vec4f> LSD(Mat img)
{
  Mat src_gray;
  cvtColor( img, src_gray, CV_BGR2GRAY );
  Ptr<LineSegmentDetector> ls = createLineSegmentDetector(LSD_REFINE_STD);
  vector<Vec4f> lines_std;
  // Detect the lines
  ls->detect(src_gray, lines_std);
  // Show found lines
   Mat drawnLines(src_gray);
   ls->drawSegments(drawnLines, lines_std);

   //  imshow("Standard refinement", drawnLines);

  return lines_std;

}

//returns true is given hue and saturation are white and not gray
bool Vision::isInPinkRange(int hue, int sat, int val)
{
  return (hue > PINK_MIN && hue < PINK_MAX && val > PINK_VAL_LOW && val < PINK_VAL_HIGH && sat > PINK_SAT_LOW && sat < PINK_SAT_HIGH);
}

//helper function return the distance between two points
double Distance(float dX0, float dY0, float dX1, float dY1)
{
    return sqrt((dX1 - dX0)*(dX1 - dX0) + (dY1 - dY0)*(dY1 - dY0));
}





//get the robot position and angle
void Vision::getRobotPose(Mat img)
{
    Vector3d offsetCenter;
    geometry_msgs::Pose2D robot_pos;
    slash_dash_bang_hash::Pose2DStamped stamped_pose;
    int circle_radius = 25;
    printf("Home robot:\n");

    // crop so we have just the mini robot location
    offsetCenter = findCenterRobot(img, robot_color::red);



    robot_pos.x = offsetCenter[0];
    robot_pos.y = offsetCenter[1];
    //robot_pos.theta = offsetCenter[2] * 180.0 / M_PI;
    robot_pos.theta = offsetCenter[2];
    stamped_pose.header = img_header_;
    stamped_pose.pose = robot_pos;
    printf("  World coordinates: %f, %f, %f\n", robot_pos.x, robot_pos.y, robot_pos.theta);
    if(!(isnan(robot_pos.x) || isnan(robot_pos.y) || isnan(robot_pos.theta)))
      home1_pub.publish(stamped_pose);

    //////////////////////////////////////////////////
    //////////////Home robot position image///////////
    Point home_robot_center = convertWorldToPixel(robot_pos.x, robot_pos.y, img.cols, img.rows);
    circle(img, home_robot_center, circle_radius, Scalar( 0, 0, 255 ));

    printf("Away robot:\n");

    //now get the away1
    offsetCenter = findCenterRobot(img, robot_color::blue);

    robot_pos.x = offsetCenter[0];
    robot_pos.y = offsetCenter[1];
    //robot_pos.theta = offsetCenter[2] * 180.0 / M_PI;
    robot_pos.theta = offsetCenter[2];
    stamped_pose.header = img_header_;
    stamped_pose.pose = robot_pos;
    printf("  World coordinates %f, %f, %f\n", robot_pos.x, robot_pos.y, robot_pos.theta);
    if(!(isnan(robot_pos.x) || isnan(robot_pos.y) || isnan(robot_pos.theta)))
      away1_pub.publish(stamped_pose);




    //////////////////////////////////////////////////
    //////////////Away robot position image///////////
    Point away_robot_center = convertWorldToPixel(robot_pos.x, robot_pos.y, img.cols, img.rows);
    circle(img, away_robot_center, circle_radius, Scalar( 255, 0, 0 ));
    imshow("vision location", img);



  // printf("the center of the bot is: %f, %f, %f\n", robot_pos.x, robot_pos.y, robot_pos.theta);
}


//returns index of longest line
int Vision::findLongestLine(vector<Vec4f> lines)
{

  int indexOfLongestLine = 0;
  int N = lines.size();
  for(int i = 1 ; i < N; i++){
    Vec4f v = lines.at(i);
    Vec4f v2 = lines.at(indexOfLongestLine);
    if(Distance(v[0], v[1], v[2], v[3]) > Distance(v2[0], v2[1], v2[2], v2[3]))
    {
      indexOfLongestLine = i;
    }
  }
  return indexOfLongestLine;
}

Point2d Vision::imageToWorldCoordinates(Point2d point_i, Mat img)
{
    Point2d centerOfField(CAMERA_WIDTH/2, CAMERA_HEIGHT/2);
    Point2d center_w = (point_i - centerOfField);

    // You have to split up the pixel to meter conversion
    // because it is a rect, not a square!
    center_w.x *= -1 * (FIELD_WIDTH/FIELD_WIDTH_PIXELS);
    center_w.y *= (FIELD_HEIGHT/FIELD_HEIGHT_PIXELS);

    // Reflect y
    center_w.y = -center_w.y;

    return center_w;
}
Mat Vision::thresholdImage(Mat img, robot_color robotColor)
{

  Mat imgHSV;
  cvtColor(img, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
  int saturateMin =  SATURATE_LOW;
  int saturateMax = SATURATE_HIGH;
  int valueMin, valueMax;
  int hueMin, hueMax;
  switch (robotColor) {
    case robot_color::yellow:                         //will also work for orange
      valueMin = YELLOW_VAL_MIN;
      valueMax = YELLOW_VAL_MAX;
      hueMin = YELLOW_MIN;
      hueMax = YELLOW_MAX;
    break;
    case robot_color::purple:
      valueMin = PURPLE_VAL_LOW;
      valueMax = PURPLE_VAL_HIGH;
      hueMin = PURPLE_MIN;
      hueMax = PURPLE_MAX;
    break;
    case robot_color::red:
      valueMin = RED_VAL_LOW;
      valueMax = RED_VAL_HIGH;
      hueMin = RED_MIN;
      hueMax = RED_MAX;
    break;
    case robot_color::blue:
      valueMin = BLUE_VAL_LOW;
      valueMax = BLUE_VAL_HIGH;
      hueMin = BLUE_MIN;
      hueMax = BLUE_MAX;
    break;
    case robot_color::green:                            //the color of the field
    break;
  }

  Mat imgThresholded;
  inRange(imgHSV, Scalar(hueMin, saturateMin, valueMin), Scalar(hueMax, saturateMax, valueMax), imgThresholded); //Threshold the image

  dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
  erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

  dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
  erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

  return imgThresholded;
}


//expects a cropped image of the robot
Vector3d Vision::findCenterRobot(Mat img, robot_color robotColor)
{
  //Threshold the image
  Mat imgThresholded = thresholdImage(img, robotColor);
  switch(robotColor)
  {
    case robot_color::red:
      imshow("thresholdedRedImage", imgThresholded);
      break;

    case robot_color::blue:
      imshow("thresholdedBlueImage", imgThresholded);
      break;
  }
  vector< vector<Point> > contours;
  vector<Moments> mm;
  vector<Vec4i> hierarchy;
  findContours(imgThresholded, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

  Vector3d ret(0, 0, 0);
  printf("  hierarchy size: %d\n", hierarchy.size());
  if (hierarchy.size() < 2)
         return ret;

  for(int i = 0; i < hierarchy.size(); i++)
      mm.push_back(moments((Mat)contours[i]));

  std::sort(mm.begin(), mm.end(), compareMomentAreas);

  Moments mmLarge = mm[mm.size() - 1];
  Moments mmSmall = mm[mm.size() - 2];

  //Print out the center of the robot in pixels for testing purposes
  Point2d robotCenterPixels = (getCenterOfMass(mmLarge) + getCenterOfMass(mmSmall)) / 2.0;
  printf("  Robot center (pixels): %f, %f\n", robotCenterPixels.x, robotCenterPixels.y);

  //Convert to world coordinates

  Vector3d robotCenterPixel3D(robotCenterPixels.x, robotCenterPixels.y, 0);

  Vector3d centerSmall3d = convertToWorldCoord(robotCenterPixel3D, img.cols, img.rows);
  Point2d centerLarge(getCenterOfMass(mmLarge));
  Point2d centerSmall(getCenterOfMass(mmSmall));

  Point2d diff = centerSmall - centerLarge;
  double angle = atan2(-1.0*diff.y, diff.x);

  //convert angle to degrees
  angle = angle *180/M_PI;
  //printf("Center of the bot world %f, %f, %f\n", diff.x, diff.y, angle);
  Vector3d pose(centerSmall3d[0], centerSmall3d[1], angle);


  return pose;
}

void Vision::findPinkBall(Mat img)
{

  //blur this image
  Mat blurImg = smoothing(img, 5);
  vector<Mat> blurChannels;
  split(blurImg, blurChannels);
  Mat imgHSV;
  cvtColor(blurImg, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
  vector<Mat> channels;
  split(imgHSV, channels);
  if (channels.size() == 0){
    return;
  }

  int x = 0;
  int y = 0;
  int count = 0;
  // search through the image to find yellow
  for (int i = 0; i < imgHSV.cols; i++)
  {
      for (int j = 0; j < imgHSV.rows; j++)
      {
        int hue = channels[0].at<uchar>(j,i);
        int sat = channels[1].at<uchar>(j,i);
        int val = channels[2].at<uchar>(j, i);
          if( isInPinkRange(hue, sat, val) && blurChannels[1].at<uchar>(j,i) < PINK_GREEN_MAX)
          {
            //this should be where the ball is
            x += i;
            y += j;
            count ++;
          }
      }
  }

  if(count > 0)
  {
    x /= count;
    y /= count;
  }


  //printf("the ball location in pixels: %d, %d\n", x, y);
  Vector3d coordinates(x, y , 0);
  Vector3d worldCoords;
  worldCoords = convertToWorldCoord(coordinates, img.cols, img.rows);
  //printf("the ball location in world: %f, %f\n", worldCoords[0], worldCoords[1]);
  geometry_msgs::Pose2D ball_pos;

  ball_pos.x = worldCoords[0];
  ball_pos.y = -1.0 * worldCoords[1];
  ball_pos.theta = 0;

  slash_dash_bang_hash::Pose2DStamped stamped_pose;
  stamped_pose.header = img_header_;
  stamped_pose.pose = ball_pos;
  ball_pub.publish(stamped_pose);
  referee_ball_pub.publish(ball_pos);
}


//helper function to remove stray lines thave have no other lines nearby or are really small
vector<Vec4f> removeStrayLines(vector<Vec4f> lines)
{
  int N = lines.size();

  for(int i = 0; i < N; i++)
  {
    const Vec4f v = lines.at(i);
    bool lineIsGood = false;
    //now we want to check against every other point in the world
    for(int j = 0; j < N; j++)
    {
      const Vec4f v2 = lines.at(j);
      if(Distance(v[0], v[1], v2[0], v2[1]) <= 20 || Distance(v[0], v[1], v2[0], v2[1]) <= 20)
      {
        //it is probably a good line
        lineIsGood = true;
      }
    }
    if(Distance(v[0], v[1], v[2], v[3]) < 5)
    {
      lineIsGood = false;
    }
    if(!lineIsGood)
    {
      lines.erase(lines.begin()+i);
      N = lines.size();
      i--;
    }

  }
  return lines;
}

//crop the field
Rect Vision::crop(Mat img)
{
  img = smoothing(img, 50);
  Rect croppedRectangle;
  Mat croppedImg;

  int croppingOffset = 70;
  int widthOffset = 80;
  int leftBuffer = 100;
  int genericBuffer = 10;

  //get the lines for portions of the field
  //top
  croppedRectangle.x = croppingOffset;
  croppedRectangle.y = genericBuffer;
  croppedRectangle.width = img.cols - (widthOffset);
  croppedRectangle.height = VERT_BOUND;
  croppedImg = Mat(img, croppedRectangle);
  vector<Vec4f> linesTop = LSD(croppedImg);


  //left
  croppedRectangle.width = HOR_BOUND - leftBuffer;
  croppedRectangle.height = img.rows -genericBuffer;
  croppedImg = Mat(img, croppedRectangle);
  vector<Vec4f> linesLeft = LSD(croppedImg);

  //right
  croppedRectangle.x = (img.cols - HOR_BOUND);
  croppedRectangle.y = genericBuffer;
  croppedRectangle.width = (HOR_BOUND - widthOffset);
  croppedRectangle.height = img.rows -genericBuffer;
  croppedImg = Mat(img, croppedRectangle);
  vector<Vec4f> linesRight = LSD(croppedImg);

  //bottom
  croppedRectangle.x = croppingOffset;
  croppedRectangle.y = img.rows - VERT_BOUND;
  croppedRectangle.width = img.cols - (widthOffset);
  croppedRectangle.height = VERT_BOUND;
  croppedImg = Mat(img, croppedRectangle);
  vector<Vec4f> linesBottom = LSD(croppedImg);


  //so we can find the long line and the biggest concentration of lines
  //we know that there should be several points nearby each of the lines


  //get the rectangle of the crop
  croppedRectangle.x = linesLeft.at(findLongestLine(removeStrayLines(linesLeft)))[0] + widthOffset;
  croppedRectangle.y = linesTop.at(findLongestLine(removeStrayLines(linesTop)))[1] + genericBuffer;
  croppedRectangle.width = ((img.cols - HOR_BOUND) + linesRight.at(findLongestLine(removeStrayLines(linesRight)))[0] - croppedRectangle.x ) - genericBuffer;
  croppedRectangle.height = ((img.rows - VERT_BOUND) + linesBottom.at(findLongestLine(removeStrayLines(linesBottom)))[1] - croppedRectangle.y) - genericBuffer;
  // printf("x %d\n", croppedRectangle.x);
  // printf("x %d\n", croppedRectangle.y);
  // printf("height %d\n", croppedRectangle.height);
  // printf("width %d\n", croppedRectangle.width);

  croppedImg = Mat(img, croppedRectangle);


  return croppedRectangle;

}

void Vision::setDestination(const StateConstPtr &msg)
{
    destination_ = *msg;

}

void Vision::setDesiredPose(const StateConstPtr &msg)
{
  desired_pose_ = *msg;
}

// ============= This code could be used to get input from the OpenCV screen for testing.
// void Vision::mouseCallback(int event, int x, int y, int flags, void* userdata) {
//     static bool mouse_left_down = false;
//
//     if (event == EVENT_LBUTTONDOWN) {
//         mouse_left_down = true;
//         Vector2d point_meters = imageToWorldCoordinates(Vector2d(x, y));
//         char buffer[50];
//         sprintf(buffer, "Location: (%.3f m, %.3f m)", point_meters(0), point_meters(1));
//         displayStatusBar(GUI_NAME, buffer, 10000);
//
//     } else if (event == EVENT_MOUSEMOVE) {
//         if (mouse_left_down) sendBallMessage(x, y);
//
//     } else if (event == EVENT_LBUTTONUP) {
//         sendBallMessage(x, y);
//         mouse_left_down = false;
//     }
//
// }
//
// void Vision::sendBallMessage(int x, int y) {
//     // Expects x, y in pixels
//
//     // Convert pixels to meters for sending  simulated ball position from mouse clicks
//     float x_meters, y_meters;
//
//     // shift data by half the pixels so (0, 0) is in center
//     x_meters = x - (CAMERA_WIDTH/2.0);
//     y_meters = y - (CAMERA_HEIGHT/2.0);
//
//     // Multiply by aspect-ratio scaling factor
//     x_meters = x_meters * (FIELD_WIDTH/FIELD_WIDTH_PIXELS);
//     y_meters = y_meters * (FIELD_HEIGHT/FIELD_HEIGHT_PIXELS);
//
//     // mirror y over y-axis
//     y_meters = -1*y_meters;
//
//     // cout << "x: " << x_meters << ", y: " << y_meters << endl;
//
//     geometry_msgs::Vector3 msg;
//     msg.x = x_meters;
//     msg.y = y_meters;
//     msg.z = 0;
//     ball_position_pub.publish(msg);
// }
//
//
// // Alternate function to convertToWorldCoord
// Vector2d imageToWorldCoordinates(Vector2d image_point, double FIELD_WIDTH, double FIELD_HEIGHT, int cols, int rows)
// {
//     Vector2d world_point = image_point;
//     world_point[0] += FIELD_WIDTH / 2.0;
//     world_point[1] += FIELD_HEIGHT / 2.0;
//     world_point[0] /= FIELD_WIDTH;
//     world_point[1] /= FIELD_HEIGHT;
//     world_point[0] *= cols;
//     world_point[1] *= rows;
//
//     return world_point;
// }


//draw desired position and the desired destination
void Vision::drawPosDest(Mat img)
{
  //extract the point from here
  Vector2d center;
  center[0] = destination_.x;
  center[1] = destination_.y;
  //convert world to pixel


  center[0] += (double)(FIELD_WIDTH / 2.0);
  center[1] += (double)(FIELD_HEIGHT / 2.0);
  center[0] /= (double)(FIELD_WIDTH);
  center[1] /= (double)FIELD_HEIGHT;
  center[0] *= (double)img.cols;
  center[1] *= (double)img.rows;

  int radius = 3;


  Point destinationCenter(center[0], center[1]);
  circle(img, destinationCenter, radius, Scalar( 0, 0, 255 ));

  //extract the point from here
  center[0] = desired_pose_.x;
  center[1] = desired_pose_.y;
  //convert world to pixel

  radius = 5;

  center[0] += (double)(FIELD_WIDTH / 2.0);
  center[1] += (double)(FIELD_HEIGHT / 2.0);
  center[0] /= (double)(FIELD_WIDTH);
  center[1] /= (double)FIELD_HEIGHT;
  center[0] *= (double)img.cols;
  center[1] *= (double)img.rows;

  Point desiredCenter(center[0], center[1]);
  circle(img, desiredCenter, radius, Scalar( 255, 0, 0 ));

  // imshow("circle", img);
}



//call back for the vision func
void Vision::visionCallback(const sensor_msgs::ImageConstPtr& msg)
{
  double now = ros::Time::now().toSec();
  static double prev = 0;
  double dt = now - prev;
  prev = now;

  img_header_ = msg->header;

  //ROS_INFO("Vision timestamp=%d:%d", img_header_.stamp.sec, img_header_.stamp.nsec);

  static bool cropped = false;
  static Rect croppedRect;
    try
    {
        Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        Mat img;
        img = frame;
        imshow("original view", img);

        if(!cropped)
        {
          croppedRect = Vision::crop(img);
          cropped = true;
        }
        img = Mat(img, croppedRect);
        getRobotPose(img);


        imshow("cropped view", img);

        findPinkBall(img);

        Mat blurImg = Vision::smoothing(img, 5);
        colorSlider(blurImg);
        drawPosDest(img);
        waitKey(60);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

Vision::Vision() :
nh_(ros::NodeHandle()),
priv_nh("~")
{

  // Private node handle to get whether we are home or away.
  // Having the nh_ private properly namespaces it.
  ros::NodeHandle priv_nh("~");
  tau_ = priv_nh.param<double>("dirty_deriv_gain", 0.05);

  // For mouse click input
  // // Create OpenCV Window and add a mouse callback for clicking
  // namedWindow(GUI_NAME, CV_WINDOW_AUTOSIZE);
  // setMouseCallback(GUI_NAME, mouseCallback, NULL);

  // Subscribe to camera
  image_transport::ImageTransport it(nh_);

  image_sub = it.subscribe("/usb_cam_away/image_raw", 1, &Vision::visionCallback, this);
  desired_pose_sub_ = nh_.subscribe<slash_dash_bang_hash::State>("desired_pose", 1, &Vision::setDesiredPose, this);
  destination_sub_ = nh_.subscribe<slash_dash_bang_hash::State>("destination", 1, &Vision::setDestination, this);

  home1_pub = nh_.advertise<slash_dash_bang_hash::Pose2DStamped>("/vision/home1", 5);
  home2_pub = nh_.advertise<slash_dash_bang_hash::Pose2DStamped>("/vision/home2", 5);
  away1_pub = nh_.advertise<slash_dash_bang_hash::Pose2DStamped>("/vision/away1", 5);
  away2_pub = nh_.advertise<slash_dash_bang_hash::Pose2DStamped>("/vision/away2", 5);
  ball_pub = nh_.advertise<slash_dash_bang_hash::Pose2DStamped>("/vision/ball_stamped", 5);
  referee_ball_pub = nh_.advertise<geometry_msgs::Pose2D>("/vision/ball", 5);

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "home");
    Vision Vision_node;
    ros::spin();
}


Point2d Vision::getCenterOfMass(Moments moment)
{
    double m10 = moment.m10;
    double m01 = moment.m01;
    double mass = moment.m00;
    double x = m10 / mass;
    double y = m01 / mass;
    return Point2d(x, y);
}

bool Vision::compareMomentAreas(Moments moment1, Moments moment2)
{
    double area1 = moment1.m00;
    double area2 = moment2.m00;
    return area1 < area2;
}

Point Vision::convertWorldToPixel(double world_x, double world_y, int cols, int rows) {
  world_y *= -1.0;
  world_x += (FIELD_WIDTH / 2.0);
  world_y += (FIELD_HEIGHT / 2.0);
  world_x /= FIELD_WIDTH;
  world_y /= FIELD_HEIGHT;
  world_x *= cols;
  world_y *= rows;

  Point pixel_coord(world_x, world_y);
  return pixel_coord;
}

Vector3d Vision::convertToWorldCoord(Vector3d pixelCoord, int cols, int rows)
{
  pixelCoord[0] /= cols;
  pixelCoord[1] /= rows;
  pixelCoord[0] *= FIELD_WIDTH;
  pixelCoord[1] *= FIELD_HEIGHT;
  pixelCoord[0] -= FIELD_WIDTH / 2.0;
  pixelCoord[1] -= FIELD_HEIGHT / 2.0;
  //reflect the y coordinates
  pixelCoord[1] *= -1;
  return pixelCoord;
}
