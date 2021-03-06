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

#define PURPLE_MIN 135
#define PURPLE_MAX 160
#define PURPLE_VAL_LOW 218
#define PURPLE_VAL_HIGH 240
#define PURPLE_SAT_LOW 0
#define PURPLE_SAT_HIGH 50

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

#define LIGHT_PINK_MIN 160
#define LIGHT_PINK_MAX 179
#define LIGHT_PINK_VAL_LOW 225
#define LIGHT_PINK_VAL_HIGH 255
#define LIGHT_PINK_SAT_LOW 0
#define LIGHT_PINK_SAT_HIGH 45

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

int colorsMin [] = {BLUE_MIN, PURPLE_MIN, RED_MIN, YELLOW_MIN, LIGHT_PINK_MIN};
int colorsMax [] = {BLUE_MAX, PURPLE_MAX, RED_MAX, YELLOW_MAX, LIGHT_PINK_MAX};
int valuesMin [] = {BLUE_VAL_LOW, PURPLE_VAL_LOW, RED_VAL_LOW, YELLOW_VAL_MIN, LIGHT_PINK_VAL_LOW};
int valuesMax [] = {BLUE_VAL_HIGH, PURPLE_VAL_HIGH, RED_VAL_HIGH, YELLOW_VAL_MAX, LIGHT_PINK_VAL_HIGH};
int saturationMin [] = {SATURATE_LOW, PURPLE_SAT_LOW, SATURATE_LOW, SATURATE_LOW, LIGHT_PINK_SAT_LOW};
int saturationMax [] = {SATURATE_HIGH, PURPLE_SAT_HIGH, SATURATE_HIGH, SATURATE_HIGH, LIGHT_PINK_SAT_HIGH};
int saturationLow = 80;
int saturationHigh = 255;

int valueLow = 80;
int valueHigh = 255;
int left_crop = 80;
int right_crop = 730;
int top_crop = 10;
int bottom_crop = 440;

char lastKeyPressed;


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

//creates trackbar and output image for each of the 4 colors
void Vision::createTrackbars(Mat img)
{

  static const char* colors[] = {"blue", "purple", "red", "yellow", "pink"};
  static bool sliderShown[] = {false, false, false, false, false};
  for(int i = 0; i < 5; i++){

  	//cvDestroyWindow(colors[i]);
    if(!sliderShown[i]){
    namedWindow(colors[i], WINDOW_NORMAL);
  	createTrackbar("Hue Low", std::string(colors[i]), &colorsMin[i], 179);
    createTrackbar("Hue High", std::string(colors[i]), &colorsMax[i], 179);
  	createTrackbar("Sat Low", std::string(colors[i]), &saturationMin[i], 255);
    createTrackbar("Sat High", std::string(colors[i]), &saturationMax[i], 255);
  	createTrackbar("Val Low", std::string(colors[i]), &valuesMin[i], 255);
    createTrackbar("Val HIgh", std::string(colors[i]), &valuesMax[i], 255);

  	moveWindow(colors[i], 0, 0);
  	resizeWindow(colors[i], 400, 100);
      sliderShown[i] = true;
  }


    Mat imgHSV;
    cvtColor(img, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

    Mat imgThresholded;

    inRange(imgHSV, Scalar(colorsMin[i], saturationMin[i], valuesMin[i]), Scalar(colorsMax[i], saturationMax[i], valuesMax[i]), imgThresholded); //Threshold the image
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

    imshow(std::string(colors[i]) + " thresholded", imgThresholded);

  }
}


//get the robot position and angle
void Vision::getRobotPose(Mat img)
{
    Vector3d offsetCenter;
    geometry_msgs::Pose2D robot_pos;
    slash_dash_bang_hash::Pose2DStamped stamped_pose;
    int circle_radius = 25;
    if(lastKeyPressed == 'p')
      printf("Home 1 robot:\n");

    // crop so we have just the mini robot location
    offsetCenter = findCenterRobot(img, home1_color_);



    robot_pos.x = offsetCenter[0];
    robot_pos.y = offsetCenter[1];
    //robot_pos.theta = offsetCenter[2] * 180.0 / M_PI;
    robot_pos.theta = offsetCenter[2];
    stamped_pose.header = img_header_;
    stamped_pose.pose = robot_pos;
    if(lastKeyPressed == 'p')
      printf("  World coordinates: %f, %f, %f\n", robot_pos.x, robot_pos.y, robot_pos.theta);
    if(!(isnan(robot_pos.x) || isnan(robot_pos.y) || isnan(robot_pos.theta))) {
      home1_pub.publish(stamped_pose);
    }

    //////////////////////////////////////////////////
    //////////////Home robot position image///////////
    //home 1 purple
    Point home_1_robot_center = convertWorldToPixel(robot_pos.x, robot_pos.y, img.cols, img.rows);
    circle(img, home_1_robot_center, circle_radius, Scalar( 255, 0, 255 ));


    //point 2
    //Point velocityPoint(home_1_robot_center.x + (command_.linear.x * circle_radius), home_1_robot_center.y + (command_.linear.y * circle_radius));
    //arrowedLine(img, home_1_robot_center, velocityPoint, Scalar(0, 0, 0));

    //home 2
    offsetCenter = findCenterRobot(img, home2_color_);

    robot_pos.x = offsetCenter[0];
    robot_pos.y = offsetCenter[1];
    //robot_pos.theta = offsetCenter[2] * 180.0 / M_PI;
    robot_pos.theta = offsetCenter[2];
    stamped_pose.header = img_header_;
    stamped_pose.pose = robot_pos;
    // printf("  World coordinates %f, %f, %f\n", robot_pos.x, robot_pos.y, robot_pos.theta);
    if(!(isnan(robot_pos.x) || isnan(robot_pos.y) || isnan(robot_pos.theta))) {
      home2_pub.publish(stamped_pose);
    }
    else { // If we don't see them, place them off the field
      robot_pos.x = 5;
      robot_pos.y = 5;
      robot_pos.theta = 0;
      stamped_pose.header = img_header_;
      stamped_pose.pose = robot_pos;
      home2_pub.publish(stamped_pose);
    }

    //home 2 blue
    Point home_2_robot_center = convertWorldToPixel(robot_pos.x, robot_pos.y, img.cols, img.rows);
    circle(img, home_2_robot_center, circle_radius, Scalar( 255, 0, 0 ));
    if(lastKeyPressed == 'p')
      printf("Away robot:\n");

    //now get the away1
    offsetCenter = findCenterRobot(img, away1_color_);

    robot_pos.x = offsetCenter[0];
    robot_pos.y = offsetCenter[1];
    //robot_pos.theta = offsetCenter[2] * 180.0 / M_PI;
    robot_pos.theta = offsetCenter[2];
    stamped_pose.header = img_header_;
    stamped_pose.pose = robot_pos;
    if(lastKeyPressed == 'p') {
      printf("  World coordinates %f, %f, %f\n", robot_pos.x, robot_pos.y, robot_pos.theta);
    }

    if(!(isnan(robot_pos.x) || isnan(robot_pos.y) || isnan(robot_pos.theta))) {
      away1_pub.publish(stamped_pose);
    }
    else { // If we don't see them, place them off the field
      robot_pos.x = 5;
      robot_pos.y = 5;
      robot_pos.theta = 0;
      stamped_pose.header = img_header_;
      stamped_pose.pose = robot_pos;
      away1_pub.publish(stamped_pose);
    }

    //away 1 red
    Point away_1_robot_center = convertWorldToPixel(robot_pos.x, robot_pos.y, img.cols, img.rows);
    circle(img, away_1_robot_center, circle_radius, Scalar( 0, 0, 255 ));

      //Away 2
      offsetCenter = findCenterRobot(img, away2_color_);

      robot_pos.x = offsetCenter[0];
      robot_pos.y = offsetCenter[1];
      //robot_pos.theta = offsetCenter[2] * 180.0 / M_PI;
      robot_pos.theta = offsetCenter[2];
      stamped_pose.header = img_header_;
      stamped_pose.pose = robot_pos;
      // printf("  World coordinates %f, %f, %f\n", robot_pos.x, robot_pos.y, robot_pos.theta);
      if(!(isnan(robot_pos.x) || isnan(robot_pos.y) || isnan(robot_pos.theta))) {
        away2_pub.publish(stamped_pose);
      }
      else { // If we don't see them, place them off the field
        robot_pos.x = 5;
        robot_pos.y = 5;
        robot_pos.theta = 0;
        stamped_pose.header = img_header_;
        stamped_pose.pose = robot_pos;
        away2_pub.publish(stamped_pose);
      }




    //////////////////////////////////////////////////
    //////////////Away robot position image///////////
    if(lastKeyPressed == 'p'){
      //away 2 yellow
    Point away_2_robot_center = convertWorldToPixel(robot_pos.x, robot_pos.y, img.cols, img.rows);
    circle(img, away_2_robot_center, circle_radius, Scalar( 0, 255, 255 ));
    imshow("vision location", img);
  }



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
  int saturateMinVal =  SATURATE_LOW;
  int saturateMaxVal = SATURATE_HIGH;
  int valueMin, valueMax;
  int hueMin, hueMax;
  switch (robotColor) {
    case robot_color::yellow:                         //will also work for orange
      valueMin = valuesMin[3];
      valueMax = valuesMax[3];
      hueMin = colorsMin[3];
      hueMax = colorsMax[3];
      saturateMinVal = saturationMin[3];
      saturateMaxVal = saturationMax[3];
    break;
    case robot_color::purple:
      valueMin = valuesMin[1];
      valueMax = valuesMax[1];
      hueMin = colorsMin[1];
      hueMax = colorsMax[1];
      saturateMinVal = saturationMin[1];
      saturateMaxVal = saturationMax[1];
    break;
    case robot_color::red:
      valueMin = valuesMin[2];
      valueMax = valuesMax[2];
      hueMin = colorsMin[2];
      hueMax = colorsMax[2];
      saturateMinVal = saturationMin[2];
      saturateMaxVal = saturationMax[2];
    break;
    case robot_color::blue:
      valueMin = valuesMin[0];
      valueMax = valuesMax[0];
      hueMin = colorsMin[0];
      hueMax = colorsMax[0];
      saturateMinVal = saturationMin[0];
      saturateMaxVal = saturationMax[0];
    break;
    case robot_color::pink:
    valueMin = valuesMin[4];
    valueMax = valuesMax[4];
    hueMin = colorsMin[4];
    hueMax = colorsMax[4];
    saturateMinVal = saturationMin[4];
    saturateMaxVal = saturationMax[4];                           //the color of the field
    break;
  }

  Mat imgThresholded;
  inRange(imgHSV, Scalar(hueMin, saturateMinVal, valueMin), Scalar(hueMax, saturateMaxVal, valueMax), imgThresholded); //Threshold the image

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
  if(lastKeyPressed == 'p'){
    switch(robotColor)
    {
      case robot_color::red:
        imshow("thresholdedRedImage", imgThresholded);
        break;

      case robot_color::blue:
        imshow("thresholdedBlueImage", imgThresholded);
        break;
    }
  }
  vector< vector<Point> > contours;
  vector<Moments> mm;
  vector<Vec4i> hierarchy;
  findContours(imgThresholded, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

  Vector3d ret(0, 0, 0);
  if(lastKeyPressed == 'p')
    printf("  hierarchy size: %d\n", (int) hierarchy.size());
  if (hierarchy.size() < 2)
         return ret;

  for(int i = 0; i < hierarchy.size(); i++)
      mm.push_back(moments((Mat)contours[i]));

  std::sort(mm.begin(), mm.end(), compareMomentAreas);

  Moments mmLarge = mm[mm.size() - 1];
  Moments mmSmall = mm[mm.size() - 2];

  //Print out the center of the robot in pixels for testing purposes
  Point2d robotCenterPixels = (getCenterOfMass(mmLarge) + getCenterOfMass(mmSmall)) / 2.0;
  if(lastKeyPressed == 'p')
    printf("in findCenterRobot:  Robot center (pixels): %f, %f\n", robotCenterPixels.x, robotCenterPixels.y);

  //Convert to world coordinates

  Vector3d robotCenterPixel3D(robotCenterPixels.x, robotCenterPixels.y, 0);

  Vector3d centerSmall3d = convertToWorldCoord(robotCenterPixel3D, img.cols, img.rows);
  Point2d centerLarge(getCenterOfMass(mmLarge));
  Point2d centerSmall(getCenterOfMass(mmSmall));

  Point2d diff = centerSmall - centerLarge;
  double angle = atan2(-1.0*diff.y, diff.x);

  //convert angle to degrees
  angle = angle *180/M_PI;
  if(lastKeyPressed == 'p')
    printf("in findCenterRobot: Center of the bot world %f, %f, %f\n", diff.x, diff.y, angle);
  Vector3d pose(centerSmall3d[0], centerSmall3d[1], angle);


  return pose;
}

void Vision::findPinkBall(Mat img)
{
  static geometry_msgs::Pose2D ball_pos_prev;

  Mat imgThresholded = thresholdImage(img, robot_color::pink);

  vector< vector<Point> > contours;
  vector<Moments> mm;
  vector<Vec4i> hierarchy;
  findContours(imgThresholded, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

  if (hierarchy.size() < 1)
         return ;

  for(int i = 0; i < hierarchy.size(); i++)
      mm.push_back(moments((Mat)contours[i]));

  std::sort(mm.begin(), mm.end(), compareMomentAreas);

  Moments mmLarge = mm[mm.size() - 1];

  Point2d centerOfBall =  getCenterOfMass(mmLarge);
  //printf("the ball location in pixels: %d, %d\n", x, y);
  Vector3d coordinates(centerOfBall.x, centerOfBall.y , 0);
  Vector3d worldCoords;
  worldCoords = convertToWorldCoord(coordinates, img.cols, img.rows);
  //printf("the ball location in world: %f, %f\n", worldCoords[0], worldCoords[1]);
  geometry_msgs::Pose2D ball_pos;

  ball_pos.x = worldCoords[0];
  ball_pos.y = worldCoords[1];
  ball_pos.theta = 0;
  slash_dash_bang_hash::Pose2DStamped stamped_pose;
  stamped_pose.header = img_header_;
  stamped_pose.pose = ball_pos;

  if(!(isnan(ball_pos.x) || isnan(ball_pos.y) || isnan(ball_pos.theta))) {
    ball_pub.publish(stamped_pose);
    ball_pos_prev = ball_pos;
    referee_ball_pub.publish(ball_pos);
  }
  else { // If we don't see them, place them off the field
    ball_pos.x = -0;
    ball_pos.y = -0;
    ball_pos.theta = -0;
    ball_pos = ball_pos_prev;
    stamped_pose.header = img_header_;
    stamped_pose.pose = ball_pos;

    ball_pub.publish(stamped_pose);
    referee_ball_pub.publish(ball_pos);
  }


  if(lastKeyPressed == 'p'){
  //pink ball
  Point ball_center = convertWorldToPixel(ball_pos.x, ball_pos.y, img.cols, img.rows);
  circle(img, ball_center, 25, Scalar( 255, 255, 255 ));
  imshow("ball", img);
  }
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
  //crop the image yall


  cvCreateTrackbar("Left", "Crop Control", &left_crop, img.cols);
  cvCreateTrackbar("Right", "Crop Control", &right_crop, img.cols);
  cvCreateTrackbar("Top", "Crop Control", &top_crop, img.rows);
  cvCreateTrackbar("Bottom", "Crop Control", &bottom_crop, img.rows);
  Rect croppedRectangle;
  croppedRectangle.x = left_crop;
  croppedRectangle.y = top_crop;
  croppedRectangle.width = right_crop - left_crop;
  croppedRectangle.height = bottom_crop - top_crop;


  Mat croppedImg;
  croppedImg = Mat(img, croppedRectangle);
  //imshow("cropped Image control", croppedImg);
  imshow("Crop Control", croppedImg);

  return croppedRectangle;

}

void Vision::setDestination(const StateConstPtr &msg)
{
    destination_ = *msg;
}

void Vision::setVelCommand(const VelConstPtr &msg)
{
  command_ = *msg;
}

void Vision::setDesiredPose(const StateConstPtr &msg)
{
  desired_pose_ = *msg;
}

//draw desired position and the desired destination
void Vision::drawPosDest(Mat img)
{
  //extract the point from here
  Vector2d center;
  center[0] = destination_.x;
  center[1] = -destination_.y; // HACK
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
  center[1] = -desired_pose_.y; // HACK
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

  imshow("circle", img);
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
        if(lastKeyPressed == 'p'){
          imshow("original view", img);
          Vision::crop(img);
        }
        Rect croppedRectangle;
        croppedRectangle.x = left_crop;
        croppedRectangle.y = top_crop;
        croppedRectangle.width = right_crop - left_crop;
        croppedRectangle.height = bottom_crop - top_crop;
        img = Mat(img, croppedRectangle);
        getRobotPose(img);
        findPinkBall(img);

        if(lastKeyPressed == 'p'){
          createTrackbars(img);
          Mat blurImg = Vision::smoothing(img, 5);
          colorSlider(blurImg);
          drawPosDest(img);
        }
        else
        {
          imshow("cropped view", img);
        }

        char key = waitKey(1);
        if(key != -1)
          lastKeyPressed = key;
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
  home1_color_str_ = priv_nh.param<std::string>("home1_color", "purple");
  home2_color_str_ = priv_nh.param<std::string>("home2_color", "blue");
  away1_color_str_ = priv_nh.param<std::string>("away1_color", "red");
  away2_color_str_ = priv_nh.param<std::string>("away2_color", "yellow");
  home1_color_ = getColorFromString(home1_color_str_);
  home2_color_ = getColorFromString(home2_color_str_);
  away1_color_ = getColorFromString(away1_color_str_);
  away2_color_ = getColorFromString(away2_color_str_);

  // For mouse click input
  // // Create OpenCV Window and add a mouse callback for clicking
  // namedWindow(GUI_NAME, CV_WINDOW_AUTOSIZE);
  // setMouseCallback(GUI_NAME, mouseCallback, NULL);

  // Subscribe to camera
  image_transport::ImageTransport it(nh_);

  image_sub = it.subscribe("/usb_cam_away/image_raw", 1, &Vision::visionCallback, this);
  desired_pose_sub_ = nh_.subscribe<slash_dash_bang_hash::State>("desired_pose", 1, &Vision::setDesiredPose, this);
  destination_sub_ = nh_.subscribe<slash_dash_bang_hash::State>("destination", 1, &Vision::setDestination, this);
  vel_command_sub_ = nh_.subscribe<geometry_msgs::Twist>("vel_command", 1, &Vision::setVelCommand, this);
  geometry_msgs::Twist command_;

  home1_pub = nh_.advertise<slash_dash_bang_hash::Pose2DStamped>("/slash_vision/home1", 5);
  home2_pub = nh_.advertise<slash_dash_bang_hash::Pose2DStamped>("/slash_vision/home2", 5);
  away1_pub = nh_.advertise<slash_dash_bang_hash::Pose2DStamped>("/slash_vision/away1", 5);
  away2_pub = nh_.advertise<slash_dash_bang_hash::Pose2DStamped>("/slash_vision/away2", 5);
  ball_pub = nh_.advertise<slash_dash_bang_hash::Pose2DStamped>("/slash_vision/ball_stamped", 5);
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

Vision::robot_color Vision::getColorFromString(std::string str) {
  robot_color color;
  if (str == "red") {
    color = robot_color::red;
  }
  else if(str == "blue") {
    color = robot_color::blue;
  }
  else if(str == "yellow") {
    color = robot_color::yellow;
  }
  else if(str == "purple") {
    color = robot_color::purple;
  }
  else {
    ROS_ERROR("Invalid color input: %s", str.c_str());
    color = robot_color::red;
  }

  return color;
}
