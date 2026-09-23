#include <gtest/gtest.h>
#include <uav_vision/h_stroke_detector.h>

cv::Mat hMask() {
  cv::Mat m=cv::Mat::zeros(400,600,CV_8UC1);
  cv::rectangle(m,{140,45},{210,355},255,-1);
  cv::rectangle(m,{390,45},{460,355},255,-1);
  cv::rectangle(m,{140,165},{460,235},255,-1);
  return m;
}

TEST(HStroke, UsesInnerEdgesWhenOuterStrokeEndsAreCropped) {
  auto mask=hMask()(cv::Rect(0,100,600,280)).clone();
  uav_vision::HStrokeObservation h;
  ASSERT_TRUE(uav_vision::detectHStrokes(mask,24,h));
  EXPECT_NEAR(h.center.x,300,3);
  EXPECT_NEAR(h.center.y,100,3);
}

TEST(HStroke, WorksWithRotatedH) {
  cv::Mat rotated;cv::rotate(hMask(),rotated,cv::ROTATE_90_CLOCKWISE);
  uav_vision::HStrokeObservation h;
  ASSERT_TRUE(uav_vision::detectHStrokes(rotated,24,h));
  EXPECT_NEAR(h.center.x,199,3);
  EXPECT_NEAR(h.center.y,300,3);
}

TEST(HStroke, RejectsCircleCrossAndU) {
  for(int kind=0;kind<3;++kind){
    cv::Mat mask=cv::Mat::zeros(400,600,CV_8UC1);
    if(kind==0)cv::circle(mask,{300,200},150,255,25);
    if(kind==1){cv::rectangle(mask,{270,40},{330,360},255,-1);cv::rectangle(mask,{140,170},{460,230},255,-1);}
    if(kind==2){cv::rectangle(mask,{140,45},{210,355},255,-1);cv::rectangle(mask,{390,45},{460,355},255,-1);cv::rectangle(mask,{140,285},{460,355},255,-1);}
    uav_vision::HStrokeObservation h;
    EXPECT_FALSE(uav_vision::detectHStrokes(mask,24,h));
  }
}

int main(int argc,char** argv){testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
