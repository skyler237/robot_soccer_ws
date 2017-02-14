#include "Vision/vision.h"
#include "Utilities/utilities.h"

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


Mat crop(Mat img)
{
    //check to see if it is a valid image
    if ( !img.data )
    {
        printf("No image data \n");
        return img;
    }

    //find the first row that has a high majority of the background color
    int t,b,l,r = 0;
    //top
    for(int i = 0 ; i < img.rows; i++)
    {
        int length = 0;
        for(int j = 0 ; j < img.cols; j++)
        {
            if(abs(((img.at<Vec3b>(i,j)[0] +
                img.at<Vec3b>(i,j)[1] +
                img.at<Vec3b>(i,j)[2]) / 3) - averageVal) < 30)
            {
                //if the value is near enough to the average color
                length++;
            }
        }
        if(length >= (int)((double)img.cols * .8))
        {
            //if the average is atleast 80% of the image
            t = i;
            break;
        }
    }

    //right side
    for(int i = img.cols - 1; i >= 0; i--)
    {
        int length = 0;
        for(int j = 0 ; j < img.rows; j++)
        {
            if(abs(((img.at<Vec3b>(j,i)[0] +
                img.at<Vec3b>(j,i)[1] +
                img.at<Vec3b>(j,i)[2]) / 3) - averageVal) < 30)
            {
                //if the value is near enough to the average color
                length++;
            }
        }
        if(length >= (int)((double)img.rows * .8))
        {
            //if the average is atleast 80% of the image
            r = i;
            break;
        }
    }
    //left side
    for(int i = 0 ; i < img.cols; i++)
    {
        int length = 0;
        for(int j = 0 ; j < img.rows; j++)
        {
            if(abs(((img.at<Vec3b>(j,i)[0] +
                img.at<Vec3b>(j,i)[1] +
                img.at<Vec3b>(j,i)[2]) / 3) - averageVal) < 30)
            {
                //if the value is near enough to the average color
                length++;
            }
        }
        if(length >= (int)((double)img.rows * .8))
        {
            //if the average is atleast 80% of the image
            l = i;
            break;
        }
    }

    //bottom

    for(int i = img.rows - 1 ; i >= 0; i--)
    {
        int length = 0;
        for(int j = 0 ; j < img.cols; j++)
        {
            if(abs(((img.at<Vec3b>(i,j)[0] +
                img.at<Vec3b>(i,j)[1] +
                img.at<Vec3b>(i,j)[2]) / 3) - averageVal) < 30)
            {
                //if the value is near enough to the average color
                length++;
            }
        }
        if(length >= (int)((double)img.cols * .8))
        {
            //if the average is atleast 80% of the image
            b = i;
            break;
        }
    }

    // printf("top side: %d\n", t);
    // printf("bottom side: %d\n", b);
    // printf("left side: %d\n", l);
    // printf("right side: %d\n", r);

    //now we want to set a new rectangle so that we can crop the image
    Rect croppedRectangle;
    croppedRectangle.x = l;
    croppedRectangle.y = t;
    croppedRectangle.width = r - l;
    croppedRectangle.height = b - t;
    Mat croppedImg = Mat(img, croppedRectangle);
    return croppedImg;
}

Mat smoothing(Mat img)
{
    Mat blurredImage;
    for(int i = 1; i < 10; i = i+2)
    {
        GaussianBlur(img, blurredImage, Size(i, i), 0, 0);
    }

    return blurredImage;
}

void displayImage(Mat img)
{
     //check to see if it is a valid image
    if ( !img.data )
    {
        printf("No image data \n");
        return;
    }
    //make a window for the image
    namedWindow("Display Image", WINDOW_AUTOSIZE );
    imshow("Display Image", img);
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


void histogramLocation(Mat img)
{
    //check to see if it is a valid image
    if ( !img.data )
    {
        printf("No image data \n");
        return;
    }

    //blue is channel 0
    //green is channel 1
    //red is channel 2
    int bins = 256;             // number of bins
    int nc = img.channels();    // number of channels

        int redspotx = 0;
        int redspoty = 0;
    // Calculate the histogram of the image
    for (int i = 0; i < img.rows; i++)
    {
        for (int j = 0; j < img.cols; j++)
        {
            //here we look for areas with high red concentration
            if((img.at<Vec3b>(i,j)[2] * .8) > averageRed)
            {
                if(img.at<Vec3b>(i,j)[1] < (averageGreen * 1.5) && ((img.at<Vec3b>(i,j)[0]) < (averageBlue * 1.5)))
                {

                    //for now lets see if we can get a spot
                    //this means it is a good point for red
                    //so we want to store it and get a the highest concentration for this color
                    redspotx = i;
                    redspoty = j;
                }
            }
        }
    }
    // printf("y: %d\n", img.rows);
    // printf("x: %d\n", img.cols);
    //
    // printf("x: %d\n", redspotx);
    // printf("y: %d\n", redspoty);


}


void histogram(Mat img)
{
    //check to see if it is a valid image
    if ( !img.data )
    {
        printf("No image data \n");
        return;
    }

    int bins = 256;             // number of bins
    int nc = img.channels();    // number of channels


    vector<Mat> hist(nc);       // histogram arrays

    // Initalize histogram arrays
    for (int i = 0; i < hist.size(); i++)
        hist[i] = Mat::zeros(1, bins, CV_32SC1);

    // Calculate the histogram of the image
    for (int i = 0; i < img.rows; i++)
    {
        for (int j = 0; j < img.cols; j++)
        {
            for (int k = 0; k < nc; k++)
            {
                uchar val = nc == 1 ? img.at<uchar>(i,j) : img.at<Vec3b>(i,j)[k];
                hist[k].at<int>(val) += 1;
            }
        }
    }
    int hmax[3] = {0,0,0};
    for (int i = 0; i < nc; i++)
    {
        for (int j = 0; j < bins-1; j++)
            hmax[i] = hist[i].at<int>(j) > hmax[i] ? hist[i].at<int>(j) : hmax[i];
    }

    const char* wname[3] = { "blue", "green", "red" };
    Scalar colors[3] = { Scalar(255,0,0), Scalar(0,255,0), Scalar(0,0,255) };

    //here we want to look at each color and try to find the best location
    //for the color ie the area that has the greatest concentraction








    vector<Mat> canvas(nc);

    // Display each histogram in a canvas
    for (int i = 0; i < nc; i++)
    {
        canvas[i] = Mat::ones(125, bins, CV_8UC3);

        for (int j = 0, rows = canvas[i].rows; j < bins-1; j++)
        {
            line(
                canvas[i],
                Point(j, rows),
                Point(j, rows - (hist[i].at<int>(j) * rows/hmax[i])),
                nc == 1 ? Scalar(200,200,200) : colors[i],
                1, 8, 0
            );
        }

        imshow(nc == 1 ? "value" : wname[i], canvas[i]);
    }
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

void getRobotPose(Mat img)
{
  Mat imgHSV;

  cvtColor(img, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

  vector< vector<Point> > contours;
  vector<Moments> mm;
  vector<Vec4i> hierarchy;
  //findContours(imgHSV, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
  //
  // if (hierarchy.size() != 2)
  //     return;
  //
  // for(int i = 0; i < hierarchy.size(); i++)
  //     mm.push_back(moments((Mat)contours[i]));
  //
  // std::sort(mm.begin(), mm.end(), compareMomentAreas);
  // Moments mmLarge = mm[mm.size() - 1];
  // Moments mmSmall = mm[mm.size() - 2];

  // Point2d centerLarge = imageToWorldCoordinates(getCenterOfMass(mmLarge), imgHSV.size());
  // Point2d centerSmall = imageToWorldCoordinates(getCenterOfMass(mmSmall), imgHSV.size());
  //
  // Point2d robotCenter = (centerLarge + centerSmall) * (1.0 / 2);
  // Point2d diff = centerSmall - centerLarge;
  // double angle = atan2(diff.y, diff.x);
}


vector<Vec4f> LSD(Mat img)
{
  Mat src_gray;
   cvtColor( img, src_gray, CV_BGR2GRAY );
   Ptr<LineSegmentDetector> ls = createLineSegmentDetector(LSD_REFINE_NONE);
  vector<Vec4f> lines_std;



  // Detect the lines
  ls->detect(src_gray, lines_std);
  // Show found lines
   Mat drawnLines(src_gray);
   ls->drawSegments(drawnLines, lines_std);
   imshow("Standard refinement", drawnLines);

  //  Mat _lines;
  //  _lines = lines.getMat();
  //  int N = _lines.checkVector(4);

  //  // Draw segments
  //  for(int i = 0; i < N; ++i)
  //  {
  //      const Vec4f& v = _lines.at<Vec4f>(i);
  //      Point2f b(v[0], v[1]);
  //      Point2f e(v[2], v[3]);
  //      line(_image.getMatRef(), b, e, Scalar(0, 0, 255), 1);
  //  }

  return lines_std;

}


bool isInYellowRange(int hue, int sat)
{
  if(hue > 19 && hue < 40 && sat > 30 && sat < 255)
  {
    return true;
  }
  return false;
}

void segmentImage(Mat img)
{
  Mat imgHSV;
  cvtColor(img, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
  vector<Mat> channels;
  split(imgHSV, channels);

  int x = 0;
  int y = 0;
  // search through the image to find yellow
  for (int i = 0; i < imgHSV.rows; i++)
  {
      for (int j = 0; j < imgHSV.cols; j++)
      {
        int hue = channels[0].at<uchar>(j,i);
        int sat = channels[1].at<uchar>(j,i);
          if( isInYellowRange(hue, sat))
          {
            //this means we have found yellow and want to create a box from this point
            int count = 0;
            //from here we want to do a search around the immediate block
            for(int k = i; k < min(863, i + 10); k++)
            {
              for(int l = j; l < min(479, j + 10); l++)
              {
                hue = channels[0].at<uchar>(l, k);
                sat = channels[1].at<uchar>(l,k);
                if( isInYellowRange(hue, sat))
                {
                  count++;
                }
              }
            }
            if(count > 20)
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

  //create a cropped image of our segment


  printf("xorg: %d\n", x);
  printf("yorg: %d\n", y);
  Rect croppedRectangle;
  croppedRectangle.x = max(1, x - 50);
  croppedRectangle.y = max(1, y - 50);
  croppedRectangle.width = min(850 - croppedRectangle.x, 100);
  croppedRectangle.height = min(479 - croppedRectangle.y, 100);
  printf("x: %d\n", croppedRectangle.x);
  printf("y: %d\n", croppedRectangle.y);
  printf("h: %d\n", croppedRectangle.height);
  printf("w: %d\n", croppedRectangle.width);
  Mat croppedImg = Mat(img, croppedRectangle);
  vector<Vec4f> lines = LSD(croppedImg);
  Mat drawnLines(img);
  Ptr<LineSegmentDetector> ls = createLineSegmentDetector(LSD_REFINE_NONE);

  ls->drawSegments(drawnLines, lines);
  //imshow("on box", drawnLines);

}

void processImage(Mat img)
{
  //img = smoothing(img);
  //calculateAverages(img);
  //img = crop(img);
  //LSD(img);
  //getRobotPose(img);
  segmentImage(img);
  //colorSlider(img);
  //imshow("Original", img); //show the original image
}


void visionCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
      //ROS_INFO("trying to do vision");
        Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        Mat img;
        img = frame;


        processImage(img);





        waitKey(60);

        //calculateAverages(img);
     //histogramLocation(img);
      //histogram(img);
      //findContours(img);
      //colorSlider(img);
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

  image_sub = it.subscribe("/usb_cam_away/image_raw", 1, visionCallback);

  //image_sub = it.subscribe("/camera1/image_raw", 1, Vision::visionCallback);
  //velocity_sub_ = nh_.subscribe<geometry_msgs::Twist>("vel_command", 1, &MotionControl::velocityCallback, this);

  home1_pub = nh_.advertise<geometry_msgs::Pose2D>("/vision/home1", 5);
  home2_pub = nh_.advertise<geometry_msgs::Pose2D>("/vision/home2", 5);
  away1_pub = nh_.advertise<geometry_msgs::Pose2D>("/vision/away1", 5);
  away2_pub = nh_.advertise<geometry_msgs::Pose2D>("/vision/away2", 5);
  ball_pub = nh_.advertise<geometry_msgs::Pose2D>("/vision/ball", 5);

}



// void Vision::visionCallback(const sensor_msgs::ImageConstPtr& msg)
// {
//     try
//     {
//       ROS_INFO("trying to do vision");
//         Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
//         imshow("camera", frame);
//         waitKey(30);
//     }
//     catch (cv_bridge::Exception& e)
//     {
//         ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
//     }
// }


int main(int argc, char **argv)
{
    ros::init(argc, argv, "home");
    //ros::NodeHandle nh_;

    Vision Vision_node;
    ros::spin();
    // ros::Rate loop_rate(30);
    // while(ros::ok())
    // {
    //     // process any callbacks
    //     ros::spinOnce();
    //
    //     // force looping at a constant rate
    //     loop_rate.sleep();
    // }
}
