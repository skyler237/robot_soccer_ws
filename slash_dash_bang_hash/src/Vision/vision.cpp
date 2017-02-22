#include "Vision/vision.h"
#include "Utilities/utilities.h"



#define YELLOW_MIN 23
#define YELLOW_MAX 36
#define SATURATE_LOW 40
#define SATURATE_HIGH 255

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


//run this at the begining of each half so that we can save on getting averages
//also get this in the hsv range
void calculateAverages(Mat img)
{

    //check to see if it is a valid image
    if ( !img.data )
    {
        printf("No image data \n");
        return;
    }

    int nc = img.channels();    // number of channels
    //printf("number of channels: %d\n", nc);


    // Calculate the histogram of the image
    for (int i = 0; i < img.rows; i++)
    {
        for (int j = 0; j < img.cols; j++)
        {
            for (int k = 0; k < nc; k++)
            {
                uchar val = nc == 1 ? img.at<uchar>(i,j) : img.at<Vec3b>(i,j)[k];
                //add the average to here
                switch(k){
                case 0://red
                    averageRed += val;
                    break;
                case 1://green
                    averageGreen += val;
                    break;
                case 2://blue
                    averageBlue += val;
                    break;
                }
            }
        }
    }

    //calculate averages
    averageRed /=(img.rows*img.cols);
    averageBlue /=(img.rows*img.cols);
    averageGreen /=(img.rows*img.cols);
    averageVal = (averageRed + averageBlue + averageGreen) / 3;
    // printf("average red: %d\n", averageRed);
    // printf("average green: %d\n", averageGreen);
    // printf("average blue: %d\n", averageBlue);


}







void findContour(Mat img)
{

    Mat src; Mat src_gray;
    int thresh = 100;
    int max_thresh = 255;
    RNG rng(12345);

    Mat canny_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    /// Load source image and convert it to gray
     src = img;

     /// Convert image to gray and blur it
     cvtColor( src, src_gray, CV_BGR2GRAY );
     blur( src_gray, src_gray, Size(3,3) );




    /// Detect edges using canny
    Canny( src_gray, canny_output, thresh, thresh*2, 3 );
    /// Find contours
    findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    /// Draw contours
    Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
    for( int i = 0; i< contours.size(); i++ )
    {
        Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
    }

    /// Show in a window
    namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
    imshow( "Contours", drawing );
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
    //while (true)
    //{

        Mat imgHSV;

        cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

        Mat imgThresholded;

        inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

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
Point2d getCenterOfMass(Moments moment)
{
    double m10 = moment.m10;
    double m01 = moment.m01;
    double mass = moment.m00;
    double x = m10 / mass;
    double y = m01 / mass;
    return Point2d(x, y);
}

bool compareMomentAreas(Moments moment1, Moments moment2)
{
    double area1 = moment1.m00;
    double area2 = moment2.m00;
    return area1 < area2;
}




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


bool Vision::isInYellowRange(int hue, int sat)
{
  return (hue > YELLOW_MIN && hue < YELLOW_MAX && sat > SATURATE_LOW && sat < SATURATE_HIGH);
}



double Distance(float dX0, float dY0, float dX1, float dY1)
{
    return sqrt((dX1 - dX0)*(dX1 - dX0) + (dY1 - dY0)*(dY1 - dY0));
}



void Vision::getRobotPose(Mat img)
{
  //crop so we have just the mini robot location
  Rect croppedRectangle;
  croppedRectangle = findYellowRobot(img);
  Mat croppedImg = Mat(img, croppedRectangle);
  Vector3d offsetCenter = findCenterRobot(croppedImg);
  geometry_msgs::Pose2D robot_pos;
  offsetCenter[0] += croppedRectangle.x;
  offsetCenter[1] += croppedRectangle.y;
  offsetCenter[0] /= img.cols;
  offsetCenter[1] /= img.rows;
  offsetCenter[0] *= FIELD_WIDTH;
  offsetCenter[1] *= FIELD_HEIGHT;
  offsetCenter[0] -= FIELD_WIDTH / 2;
  offsetCenter[1] -= FIELD_HEIGHT / 2;
  robot_pos.x = offsetCenter[0];
  robot_pos.y = offsetCenter[1];
  robot_pos.theta =  offsetCenter[2] * 180.0 / M_PI;
  home1_pub.publish(robot_pos);

  printf("the center of the bot is: %f, %f, %f\n", robot_pos.x, robot_pos.y, robot_pos.theta);
  imshow("overhead",img);
}


float findTheta(vector<Vec4f> lines, Vector2d center)
{
  Ptr<LineSegmentDetector> ls = createLineSegmentDetector(LSD_REFINE_NONE);
  int N = lines.size();
  //look for the 2 longest
  int index = 0;
  int index_2 = 1;
  for(int i = 2 ; i < N; i++)
  {
    const Vec4f v = lines.at(i);
    const Vec4f v2 = lines.at(index);
    const Vec4f v3 = lines.at(index_2);

    if(Distance(v[0], v[1], v[2], v[3]) > Distance(v3[0], v3[1], v3[2], v3[3]))
    {
      //then it is greater than our 2nd best
      //now we need to know if it is better than our best as well
      if(Distance(v[0], v[1], v[2], v[3]) > Distance(v2[0], v2[1], v2[2], v2[3]))
      {
        //now we have our best
        index = i;
      }
      else
      {
        index_2 = i;
      }
    }
  }
  const Vec4f v = lines.at(index);
  const Vec4f v2 = lines.at(index_2);
  Vector2d point_1(v[2], v[3]);
  Vector2d point_2(v2[2], v2[3]);
  //find the two far points and then get the angle compared to the box
  if(Distance(v[0], v[1], center[0], center[1]) > Distance(v[2], v[3], center[0], center[1]))
  {
    //the first point is the one we want
    point_1[0] = v[0];
    point_1[1] = v[1];
  }
  if(Distance(v2[0], v2[1], center[0], center[1]) > Distance(v2[2], v2[3], center[0], center[1]))
  {
    //the first point is the one we want
    point_2[0] = v2[0];
    point_2[1] = v2[1];
  }



  point_1[0] = (point_1[0] + point_2[0]) / 2;
  point_1[1] = (point_1[1] + point_2[1]) / 2;


  point_1[0] -= center[0];
  point_1[1] -= center[1];

  return atan2(point_1[1], point_1[0]);
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

  float angle =  atan2(frontPoint[1], frontPoint[0]);
  //printf("angle %f\n", angle);
  Vector3d ret(center[0], center[1], angle);
  return ret;

  //  vector<Vec4f> triangleLines;
  //  // Draw segments
  //  for(int i = 0; i < N; ++i)
  //  {
  //    const Vec4f v = lines.at(i);
  //    int pointCount = 0;
  //    triangleLines.clear();
  //    triangleLines.push_back(v);
  //    for(int l = 0; l < 4; l+=2)
  //    {
  //      for(int j = 1 ; j < N; j++)
  //      {
  //        const Vec4f v2 = lines.at(j);
  //        for(int k = 0; k < 4; k+=2)
  //        {
   //
  //          if(Distance(v[l], v[l+1], v2[k], v2[k+1]) <= 2)
  //          {
  //            triangleLines.push_back(v2);
  //            pointCount++;
  //          }
  //        }
  //      }
  //      if(pointCount >= 5)
  //      {
  //        //if four points agree then this is the center of the bot
  //        indexX = l;
  //        indexY = l + 1;
  //        indexL = i;
  //        centerFound = true;
  //        break;
  //      }
  //    }
  //    if(centerFound)
  //    {
  //      break;
  //    }
  //  }
  //  const Vec4f v = lines.at(indexL);
  //  printf("the center of the bot is: %f, %f\n", v[indexX], v[indexY]);
   //
  //  Vector2d center(v[indexX], v[indexY]);
  //  float angle = findTheta(triangleLines, center);
  //  Vector3d ret(v[indexX], v[indexY], angle);
  //  return ret;

  // Vector3d ret(0, 0, 0);
  // Pose2D robotPose;
  //
  // Mat imgHSV;
  // cvtColor(img, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
  //
  // Mat imgGray;
  // thresholdImage(imgHSV, imgGray, yellow);
  //
  //
  //
  //
  // vector< vector<Point> > contours;
  // vector<Moments> mm;
  // vector<Vec4i> hierarchy;
  // findContours(imgGray, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
  //
  // if (hierarchy.size() != 2){
  //   printf("too small foo\n");
  //     return ret;
  //   }
  //
  // for(int i = 0; i < hierarchy.size(); i++)
  //     mm.push_back(moments((Mat)contours[i]));
  //
  // std::sort(mm.begin(), mm.end(), compareMomentAreas);
  // Moments mmLarge = mm[mm.size() - 1];
  // Moments mmSmall = mm[mm.size() - 2];
  //
  // Point2d centerLarge = getCenterOfMass(mmLarge);
  // Point2d centerSmall = getCenterOfMass(mmSmall);
  //
  // Point2d robotCenter = (centerLarge + centerSmall) * (1.0 / 2);
  // Point2d diff = centerSmall - centerLarge;
  // printf("center sm: %f %f\n", centerSmall.x, centerSmall.y);
  // printf("center lg: %f %f\n", centerLarge.x, centerLarge.y);
  //
  // printf("diff: %f %f\n", diff.y, diff.x);
  // double angle = atan2(diff.y, diff.x);
  //
  // //convert angle to degrees
  // angle = angle *180/M_PI;
  // robotPose.x = robotCenter.x;
  // robotPose.y = robotCenter.y;
  // robotPose.theta = angle;
  //   printf("the center of the bot is: %f, %f, %f\n", robotPose.x, robotPose.y, robotPose.theta);
  //
  // Vector3d retr(robotPose.x, robotPose.y, angle);
  // return retr;
}

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
  for (int i = 0; i < imgHSV.cols; i++)
  {
      for (int j = 0; j < imgHSV.rows; j++)
      {
        int hue = channels[0].at<uchar>(j,i);
        int sat = channels[1].at<uchar>(j,i);
          if( isInYellowRange(hue, sat))
          {
            //this means we have found yellow and want to create a box from this point
            int count = 0;
            //from here we want to do a search around the immediate block
            for(int k = max(0, i - 8); k < min(imgHSV.cols, i + 8); k++)
            {
              for(int l = max(0, j - 8); l < min(imgHSV.rows, j + 8); l++)
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

Rect Vision::crop(Mat img)
{
  img = smoothing(img, 50);
  Rect croppedRectangle;
  Mat croppedImg;

  int croppingOffset = 70;

  //get the lines for portions of the field
  //top
  croppedRectangle.x = croppingOffset;
  croppedRectangle.y = 10;
  croppedRectangle.width = img.cols - (croppingOffset + 10);
  croppedRectangle.height = VERT_BOUND;
  // printf("x %d\n", croppedRectangle.x);
  // printf("y %d\n", croppedRectangle.y);
  // printf("height %d\n", croppedRectangle.height);
  // printf("width %d\n", croppedRectangle.width);
  croppedImg = Mat(img, croppedRectangle);
  vector<Vec4f> linesTop = LSD(croppedImg);


  //left
  croppedRectangle.width = HOR_BOUND - 100;
  croppedRectangle.height = img.rows -10;
  // printf("x %d\n", croppedRectangle.x);
  // printf("y %d\n", croppedRectangle.y);
  // printf("height %d\n", croppedRectangle.height);
  // printf("width %d\n", croppedRectangle.width);
  croppedImg = Mat(img, croppedRectangle);
  vector<Vec4f> linesLeft = LSD(croppedImg);

  //right
  croppedRectangle.x = (img.cols - HOR_BOUND);
  croppedRectangle.y = 10;
  croppedRectangle.width = HOR_BOUND - croppingOffset;
  croppedRectangle.height = img.rows -15;
  // printf("x %d\n", croppedRectangle.x);
  // printf("x %d\n", croppedRectangle.y);
  // printf("height %d\n", croppedRectangle.height);
  // printf("width %d\n", croppedRectangle.width);
  croppedImg = Mat(img, croppedRectangle);
  vector<Vec4f> linesRight = LSD(croppedImg);

  //bottom
  croppedRectangle.x = croppingOffset;
  croppedRectangle.y = img.rows - VERT_BOUND;
  croppedRectangle.width = img.cols - (croppingOffset + 10);
  croppedRectangle.height = VERT_BOUND;
  // printf("x %d\n", croppedRectangle.x);
  // printf("x %d\n", croppedRectangle.y);
  // printf("height %d\n", croppedRectangle.height);
  // printf("width %d\n", croppedRectangle.width);
  croppedImg = Mat(img, croppedRectangle);
  vector<Vec4f> linesBottom = LSD(croppedImg);


  //so we can find the long line and the biggest concentration of lines
  //we know that there should be several points nearby each of the lines


  //get the rectangle of the crop
  croppedRectangle.x = linesLeft.at(findLongestLine(removeStrayLines(linesLeft)))[0] + croppingOffset;
  croppedRectangle.y = linesTop.at(findLongestLine(removeStrayLines(linesTop)))[1];
  croppedRectangle.width = ((img.cols - HOR_BOUND) + linesRight.at(findLongestLine(removeStrayLines(linesRight)))[0] - croppedRectangle.x );
  croppedRectangle.height = ((img.rows - VERT_BOUND) + linesBottom.at(findLongestLine(removeStrayLines(linesBottom)))[1] - croppedRectangle.y);
  // printf("x %d\n", croppedRectangle.x);
  // printf("x %d\n", croppedRectangle.y);
  // printf("height %d\n", croppedRectangle.height);
  // printf("width %d\n", croppedRectangle.width);

  croppedImg = Mat(img, croppedRectangle);


  return croppedRectangle;

}

void Vision::visionCallback(const sensor_msgs::ImageConstPtr& msg)
{
  static bool cropped = false;
  static Rect croppedRect;
    try
    {
        Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        Mat img;
        img = frame;
        if(!cropped)
        {

          //blur the image before we pass it in
          croppedRect = Vision::crop(img);
          cropped = true;

        }
        imshow("original view", img);
        img = Mat(img, croppedRect);
        //LSD(img);
        getRobotPose(img);
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

  // Subscribe to camera
  image_transport::ImageTransport it(nh_);
  //ally2_state_sub_ = nh_.subscribe<slash_dash_bang_hash::State>("ally2_state", 1, boost::bind(&AI::stateCallback, this, _1, "ally2"));

  image_sub = it.subscribe("/usb_cam_away/image_raw", 1, &Vision::visionCallback, this);

  //image_sub = it.subscribe("/camera1/image_raw", 1, Vision::visionCallback);
  //velocity_sub_ = nh_.subscribe<geometry_msgs::Twist>("vel_command", 1, &MotionControl::velocityCallback, this);

  home1_pub = nh_.advertise<geometry_msgs::Pose2D>("/vision/home1", 5);
  home2_pub = nh_.advertise<geometry_msgs::Pose2D>("/vision/home2", 5);
  away1_pub = nh_.advertise<geometry_msgs::Pose2D>("/vision/away1", 5);
  away2_pub = nh_.advertise<geometry_msgs::Pose2D>("/vision/away2", 5);
  ball_pub = nh_.advertise<geometry_msgs::Pose2D>("/vision/ball", 5);

}





int main(int argc, char **argv)
{
    ros::init(argc, argv, "home");
    //ros::NodeHandle nh_;

    Vision Vision_node;
    ros::spin();

}
