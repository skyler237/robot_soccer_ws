#include "Vision/vision.h"
#include "Utilities/utilities.h"



#define YELLOW_MIN 19
#define YELLOW_MAX 36
#define SATURATE_LOW 40
#define SATURATE_HIGH 255

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

int averageBlue = 0;
int averageRed = 0;
int averageGreen = 0;
int averageVal = 0;

//export ROS_MASTER_URI=http://192.168.1.160:11311
//export ROS_IP=myip10.0.2.15



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

/*
    cvCreateTrackbar("redLow", "Control", &iLowH, 255); //Hue (0 - 179)
    cvCreateTrackbar("redHigh", "Control", &iHighH, 255);

    cvCreateTrackbar("greenLow", "Control", &iLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("greenHigh", "Control", &iHighS, 255);

    cvCreateTrackbar("blueLow", "Control", &iLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("blueHigh", "Control", &iHighV, 255);
*/

    Mat imgOriginal;
    imgOriginal = img;
    //while (true)
    //{

        Mat imgHSV;
        cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

        Mat imgThresholded;

        inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
        //inRange(imgOriginal, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded);
        //morphological opening (remove small objects from the foreground)
        erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

        //morphological closing (fill small holes in the foreground)
        dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

        imshow("Thresholded Image", imgThresholded); //show the thresholded image
        imshow("Original", imgOriginal); //show the original image


    //}
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
   imshow("Standard refinement", drawnLines);


  return lines_std;

}

//returns true is given hue and saturation are yellow and not gray
bool Vision::isInYellowRange(int hue, int sat)
{
  return (hue > YELLOW_MIN && hue < YELLOW_MAX && sat > SATURATE_LOW && sat < SATURATE_HIGH);
}
//returns true is given hue and saturation are white and not gray
bool Vision::isInwhiteRange(int hue, int sat, int val)
{
  return (hue > WHITE_MIN && hue < WHITE_MAX && val > WHITE_VAL_LOW && val < WHITE_VAL_HIGH);
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



Vector3d Vision::convertToWorldCoord(Vector3d pixelCoord, int offSetX, int offSetY, int cols, int rows)
{
  pixelCoord[0] += offSetX;
  pixelCoord[1] += offSetY;
  pixelCoord[0] /= cols;
  pixelCoord[1] /= rows;
  pixelCoord[0] *= FIELD_WIDTH;
  pixelCoord[1] *= FIELD_HEIGHT;
  pixelCoord[0] -= FIELD_WIDTH / 2;
  pixelCoord[1] -= FIELD_HEIGHT / 2;
  return pixelCoord;
}

//get the robot position and angle
void Vision::getRobotPose(Mat img)
{


  //blur this image
  // Mat blurImg = smoothing(img, 5);
  // Mat imgHSV;
  // cvtColor(blurImg, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
  // vector<Mat> channels;
  // split(imgHSV, channels);
  //
  // int x = 0;
  // int y = 0;
  // int count = 0;
  // // search through the image to find yellow
  // for (int i = 0; i < imgHSV.cols - 1; i++)
  // {
  //     for (int j = 0; j < imgHSV.rows - 1; j++)
  //     {
  //       int hue = channels[0].at<uchar>(j,i);
  //       int sat = channels[1].at<uchar>(j,i);
  //         if( isInYellowRange(hue, sat))
  //         {
  //           x += i;
  //           y += j;
  //           count++;
  //         }
  //
  //       }
  // }
  // x /= count;
  // y /= count;
  // printf("center of the Bot average: %d, %d\n", x, y);



  //crop so we have just the mini robot location
  Rect croppedRectangle;
  croppedRectangle = findYellowRobot(img);
  Mat croppedImg = Mat(img, croppedRectangle);
  Vector3d offsetCenter = findCenterRobot(croppedImg);

  geometry_msgs::Pose2D robot_pos;
  //printf("center of the Bot lines: %d, %d\n", offsetCenter[0], offsetCenter[1]);

  offsetCenter = convertToWorldCoord(offsetCenter, croppedRectangle.x, croppedRectangle.y, img.cols, img.rows);

  robot_pos.x = offsetCenter[0];
  robot_pos.y = -1.0* offsetCenter[1]; // HACK!
  robot_pos.theta = offsetCenter[2] * 180.0 / M_PI;
  home1_pub.publish(robot_pos);

  printf("the center of the bot is: %f, %f, %f\n", robot_pos.x, robot_pos.y, robot_pos.theta);
  imshow("overhead",img);
}






void thresholdImage(Mat& imgHSV, Mat& imgGray, Scalar color[])
{
    inRange(imgHSV, color[0], color[1], imgGray);

    erode(imgGray, imgGray, getStructuringElement(MORPH_ELLIPSE, Size(2, 2)));
    dilate(imgGray, imgGray, getStructuringElement(MORPH_ELLIPSE, Size(2, 2)));
}

//returns index of longest line
int findLongestLine(vector<Vec4f> lines)
{

  int indexOfLongestLine = 0;
  int N = lines.size();
  for(int i = 1 ; i < N; i++)
  {
    Vec4f v = lines.at(i);
    Vec4f v2 = lines.at(indexOfLongestLine);
    if(Distance(v[0], v[1], v[2], v[3]) > Distance(v2[0], v2[1], v2[2], v2[3]))
    {
      indexOfLongestLine = i;
    }
  }
  return indexOfLongestLine;
}

//expects a cropped image of the robot
Vector3d Vision::findCenterRobot(Mat img)
{
  vector<Vec4f> lines = LSD(img);
  Ptr<LineSegmentDetector> ls = createLineSegmentDetector(LSD_REFINE_STD);

  //find four lines that all have a similar point -- this is the center of the robot
  //should be between a majority of the lines
  //the points should be within 3px in any direction of each other

   int N = lines.size();
   //if we haven't found any lines then we should return 0,0,0
   if(N <= 0)
   {
     Vector3d ret(0,0,0);
     return ret;
   }

   int indexY = 0;
   int indexX = 0;
   int indexL = 0;
   bool centerFound = false;



   //find the longest line and go for it
   int longestIndex = findLongestLine(lines);
  const Vec4f v = lines.at(longestIndex);

   Vector2d center(v[0], v[1]);

   center[0] = (center[0] + v[2]) / 2;
   center[1] = (center[1] + v[3]) / 2;

   //printf("the center of the bot is: %f, %f\n", center[0], center[1]);

    //forward facing is the one with more black pixels
    //look at both points and determine which one has more black
    //in a 2 by 2 box

    Mat imgHSV;
    cvtColor(img, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
    vector<Mat> channels;
    split(imgHSV, channels);



    int ln_1 = 0;
    int ln_2 = 0;
    for(int i = 0 ; i < N; i++)
    {
      const Vec4f v2 = lines.at(i);
      ln_1 += Distance(v[0], v[1], v2[0], v2[1]);
      ln_1 += Distance(v[0], v[1], v2[2], v2[3]);

    }
    for(int i = 0 ; i < N; i++)
    {
      const Vec4f v2 = lines.at(i);
      ln_2 += Distance(v[2], v[3], v2[0], v2[1]);
      ln_2 += Distance(v[2], v[3], v2[2], v2[3]);
    }
    Vector2d frontPoint(v[0], v[1]);
    if(ln_1 > ln_2)
    {
      //count one is the front
      frontPoint[0] = v[2];
      frontPoint[1] = v[3];
    }

    //printf("front point %f, %f\n", frontPoint[0], frontPoint[1]);

    frontPoint[0] -= center[0];
    frontPoint[1] -= center[1];

  float angle =  atan2(-frontPoint[1], frontPoint[0]);
  //printf("angle %f\n", angle);
  Vector3d ret(center[0], center[1], angle);

  return ret;

}

//find white ball
void Vision::findWhiteBall(Mat img)
{

  //blur this image
  Mat blurImg = smoothing(img, 5);
  Mat imgHSV;
  cvtColor(blurImg, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
  vector<Mat> channels;
  split(imgHSV, channels);

  int x = 0;
  int y = 0;
  // search through the image to find yellow
  for (int i = 0; i < imgHSV.cols; i++)
  {
      for (int j = 0; j < imgHSV.rows; j++)
      {
        int hue = channels[0].at<uchar>(j,i);
        int sat = channels[1].at<uchar>(j,i);
        int val = channels[2].at<uchar>(j, i);
          if( isInwhiteRange(hue, sat, val))
          {
            //this should be where the ball is
            x = i;
            y = j;
            //TODO:find the center of the ball
          }
      }
  }
  //printf("the ball location in pixels: %d, %d\n", x, y);
  Vector3d coordinates(x, y , 0);
  Vector3d worldCoords;
  worldCoords = convertToWorldCoord(coordinates, 0, 0, img.cols, img.rows);
  //printf("the ball location in world: %f, %f\n", worldCoords[0], worldCoords[1]);
  geometry_msgs::Pose2D ball_pos;

  ball_pos.x = -1.0* worldCoords[0];
  ball_pos.y =  -1.0 * worldCoords[1];
  ball_pos.theta = 0;
  ball_pub.publish(ball_pos);
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
            x = i;
            y = j;
            //TODO:find the center of the ball
          }
      }
  }
  //printf("the ball location in pixels: %d, %d\n", x, y);
  Vector3d coordinates(x, y , 0);
  Vector3d worldCoords;
  worldCoords = convertToWorldCoord(coordinates, 0, 0, img.cols, img.rows);
  //printf("the ball location in world: %f, %f\n", worldCoords[0], worldCoords[1]);
  geometry_msgs::Pose2D ball_pos;

  ball_pos.x = worldCoords[0];
  ball_pos.y = -1.0 * worldCoords[1];
  ball_pos.theta = 0;
  ball_pub.publish(ball_pos);
}



//find the first yellow pixel that works and creates a rectangle that will contain the robot
Rect Vision::findYellowRobot(Mat img)
{
  //blur this image
  Mat blurImg = smoothing(img, 5);
  Mat imgHSV;
  cvtColor(blurImg, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
  vector<Mat> channels;
  split(imgHSV, channels);

  int x = 0;
  int y = 0;
  // search through the image to find yellow
  for (int i = 0; i < imgHSV.cols - 1; i++)
  {
      for (int j = 0; j < imgHSV.rows - 1; j++)
      {
        int hue = channels[0].at<uchar>(j,i);
        int sat = channels[1].at<uchar>(j,i);
          if( isInYellowRange(hue, sat))
          {
            //this means we have found yellow and want to create a box from this point
            int count = 0;
            //from here we want to do a search around the immediate block
            for(int k = max(0, i - 8); k < min(imgHSV.cols - 1, i + 8); k++)
            {
              for(int l = max(0, j - 8); l < min(imgHSV.rows - 1, j + 8); l++)
              {
                hue = channels[0].at<uchar>(l, k);
                sat = channels[1].at<uchar>(l,k);
                if( isInYellowRange(hue, sat))
                {
                  count++;
                }
              }
            }
            if(count > 50)
            {
              x = i;
              y = j;
              break;
            }
          }
      }
      if(x > 0 && y > 0)
      {
        break;
      }
  }






  Rect croppedRectangle;
  croppedRectangle.x = max(0, x - 50);
  croppedRectangle.y = max(0, y - 50);
  croppedRectangle.width = min(imgHSV.cols - croppedRectangle.x, 100);
  croppedRectangle.height = min(imgHSV.rows - croppedRectangle.y, 100);

  return croppedRectangle;

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
  croppedRectangle.width = HOR_BOUND - widthOffset;
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

  imshow("circle", img);
}



//call back for the vision func
void Vision::visionCallback(const sensor_msgs::ImageConstPtr& msg)
{
  double now = ros::Time::now().toSec();
  static double prev = 0;
  double dt = now - prev;
  prev = now;

  ROS_INFO("Vision visionCallback: dt=%f", dt);

  static bool cropped = false;
  static Rect croppedRect;
    try
    {
        Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        Mat img;
        img = frame;
        if(!cropped)
        {
          croppedRect = Vision::crop(img);
          cropped = true;
        }
        imshow("original view", img);
        img = Mat(img, croppedRect);
        getRobotPose(img);
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

  home1_pub = nh_.advertise<geometry_msgs::Pose2D>("/vision/home1", 5);
  home2_pub = nh_.advertise<geometry_msgs::Pose2D>("/vision/home2", 5);
  away1_pub = nh_.advertise<geometry_msgs::Pose2D>("/vision/away1", 5);
  away2_pub = nh_.advertise<geometry_msgs::Pose2D>("/vision/away2", 5);
  ball_pub = nh_.advertise<geometry_msgs::Pose2D>("/vision/ball", 5);

}





int main(int argc, char **argv)
{
    ros::init(argc, argv, "home");
    Vision Vision_node;
    ros::spin();

}
