#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

cv::Mat frame;
int count = 0;
int minH = 0, maxH = 20, minS = 30, maxS = 150, minV = 60, maxV = 255;

float innerAngle(float xt1, float yt1, float xt2, float yt2, float cx1, float cy1)
{

 float distance1 = std::sqrt((xt1-cx1)*(xt1-cx1) + (yt1-cy1)*(yt1-cy1));
 float distance2 = std::sqrt((xt2-cx1)*(xt2-cx1) + (yt2-cy1)*(yt2-cy1));

 float Ax, Ay;
 float Bx, By;
 float Cx, Cy;

 Cx = cx1;
 Cy = cy1;
 if(distance1 < distance2)
 {
  Bx = xt1;
  By = yt1;
  Ax = xt2;
  Ay = yt2;

 }else
 {
  Bx = xt2;
  By = yt2;
  Ax = xt1;
  Ay = yt1;
 }

 float R1 = Cx - Ax;
 float R2 = Cy - Ay;
 float P1 = Bx - Ax;
 float P2 = By - Ay;

 float Z = std::acos( (P1*R1 + P2*R2) / ( std::sqrt(P1*P1+P2*P2) * std::sqrt(R1*R1+R2*R2) ) );

 Z = Z*180/CV_PI;

 return Z;
}

void CallbackFunc(int event, int x, int y, int flags, void* userdata)
{
  cv::Mat RGB = frame(cv::Rect(x, y, 1, 1));
  cv::Mat HSV;
  cv::cvtColor(RGB, HSV, CV_BGR2HSV);
  cv::Vec3b pixel = HSV.at<cv::Vec3b>(0, 0);
  if (event == cv::EVENT_LBUTTONDBLCLK)
  {
      std::cout << "Click" << std::endl;
      int h = pixel.val[0];
      int s = pixel.val[1];
      int v = pixel.val[2];
      if (count == 0)
      {
          minH = h;
          maxH = h;
          minS = s;
          maxS = s;
          minV = v;
          maxV = v;
      }
      else
      {
          if (h < minH)
          {
              minH = h;
          }
          else if (h > maxH)
          {
              maxH = h;
          }
          if (s < minS)
          {
              minS = s;
          }
          else if (s > maxS)
          {
              maxS = s;
          }
          if (v < minV)
          {
              minV = v;
          }
          else if (v > maxV)
          {
              maxV = v;
          }

      }
      count++;
  }
  std::cout << pixel << std::endl;
}

int main()
{
  cv::VideoCapture cap(0);
  const char* windowName = "Fingertip detection";
  cv::namedWindow(windowName);
  cv::setMouseCallback(windowName, CallbackFunc, NULL);
  int inAngleMin = 200, inAngleMax = 300, angleMin = 180, angleMax = 359, length1Min = 10, length1Max = 80;

  //Create trackbars for adjusting the angle
  //cv::createTrackbar("Inner angle min", windowName, &inAngleMin, 360);
  //cv::createTrackbar("Inner angle max", windowName, &inAngleMax, 360);
  //cv::createTrackbar("Angle min", windowName, &angleMin, 360);
  //cv::createTrackbar("Angle max", windowName, &angleMax, 360);
  //cv::createTrackbar("length1 min", windowName, &length1Min, 100);
  //cv::createTrackbar("length1 max", windowName, &length1Max, 100);

  while (1)
  {
      cap >> frame;
      cv::Mat hsv;
      cv::cvtColor(frame, hsv, CV_BGR2HSV);
      cv::inRange(hsv, cv::Scalar(minH, minS, minV), cv::Scalar(maxH, maxS, maxV), hsv);

      // Morphoogical Techniques: Blurring, Dilation
      int elementSize = 5;
      int blurSize = 5;
      cv::medianBlur(hsv, hsv, blurSize);
      cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * elementSize + 1, 2 * elementSize + 1), cv::Point(elementSize, elementSize));
      //cv::erode(hsv,hsv,element);
      cv::dilate(hsv, hsv, element);

      // Contour detection
      std::vector<std::vector<cv::Point> > contours;
      std::vector<cv::Vec4i> hierarchy;
      cv::findContours(hsv, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
      size_t largestContour = 0;
      for (size_t i = 1; i < contours.size(); i++)
      {
          if (cv::contourArea(contours[i]) > cv::contourArea(contours[largestContour]))
              largestContour = i;
      }
      cv::drawContours(frame, contours, largestContour, cv::Scalar(0, 0, 255), 1);

      // Convex hull to dectect fingertips
      if (!contours.empty())
      {
          std::vector<std::vector<cv::Point> > hull(1);
          cv::convexHull(cv::Mat(contours[largestContour]), hull[0], false);
          cv::drawContours(frame, hull, 0, cv::Scalar(255, 0, 0), 3);
          if (hull[0].size() > 2)
          {
              std::vector<int> hullIndexes;
              cv::convexHull(cv::Mat(contours[largestContour]), hullIndexes, true);
              std::vector<cv::Vec4i> convexityDefects;
              cv::convexityDefects(cv::Mat(contours[largestContour]), hullIndexes, convexityDefects);

              //Bounding box to find centre of palm
              cv::Rect boundingBox = cv::boundingRect(hull[0]);
              cv::rectangle(frame, boundingBox, cv::Scalar(0,255,0));
              cv::Point center = cv::Point(boundingBox.x + boundingBox.width / 2, boundingBox.y + boundingBox.height / 2);
              std::vector<cv::Point> validPoints;
              for (size_t i = 0; i < convexityDefects.size(); i++)
              {
                  cv::Point p1 = contours[largestContour][convexityDefects[i][0]];
                  cv::Point p2 = contours[largestContour][convexityDefects[i][1]];
                  cv::Point p3 = contours[largestContour][convexityDefects[i][2]];
                  double angle = std::atan2(center.y - p1.y, center.x - p1.x) * 180 / CV_PI;
                  double inAngle = innerAngle(p1.x, p1.y, p2.x, p2.y, p3.x, p3.y);
                  double length1 = std::sqrt(std::pow(p1.x - p3.x, 2) + std::pow(p1.y - p3.y, 2));
                  if (angle > angleMin - 180 && angle < angleMax - 180 && inAngle > inAngleMin - 180 && inAngle < inAngleMax - 180 && length1 > length1Min / 100.0 * boundingBox.height && length1 < length1Max / 100.0 * boundingBox.height)
                  {
                      validPoints.push_back(p1);
                  }
              }

             for (size_t i = 0; i < validPoints.size(); i++)
           {

                  //boundRect[i] = boundingRect( Mat(contours_poly[i]) );

                  //rectangle( frame, boundRect[i].tl(), boundRect[i].br(), CV_RGB(0,0,0), 2, 8, 0 );

                  //Point2f rect_points[4];
                  //minRect[i].points( rect_points );

                  //for( int j = 0; j < 4; j++ )
                  //{
                   //line( frame, rect_points[j], rect_points[(j+1)%4], CV_RGB(255,255,0), 2, 8 );
                  //cv::rectangle(frame,cv::Point( 0, 7*i/8.0 ),cv::Point( i, i),cv::Scalar( 0, 255, 255 ),-1,8);

                 //cv::circle(frame, validPoints[i], 20, cv::Scalar(0, 250, 255), 3);
                 cv::rectangle(frame, cv::Point(validPoints[i].x-18, validPoints[i].y-36),cv::Point(validPoints[i].x+18, validPoints[i].y+36),cv::Scalar(255,0,255),3 );

              }
          }
      }
      cv::imshow(windowName, frame);
      cv::imshow("Thresholded_image",hsv);
      if (cv::waitKey(30) >= 0) break;
  }
  return 0;
}

