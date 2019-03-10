#include "mark_aim.h"

MarkAim::MarkAim(){}
MarkAim::~MarkAim(){}

void MarkAim::init(){
    pnpSolver.clearPoints3D();
    pnpSolver.pushPoints3D(-112, -35, 0);
    pnpSolver.pushPoints3D(112,  -35, 0);
    pnpSolver.pushPoints3D(112, 35, 0);
    pnpSolver.pushPoints3D(-112, 35, 0);
}

float MarkAim::distance(Point2f point1,Point2d point2){
    return sqrt(pow(point2.x-point1.x,2)+pow(point2.y-point1.y,2));
}

int MarkAim::findUndecidedArea(vector<RotatedRect> rect){
    int n = 0;
    if(rect.size()!=0){
        for(int i=0;i<rect.size();i++){
            if(rect[i].size.area()<=rect[n].size.area()) n = i; 
        }
        return n;
    }else{
        return -1;
    }
}

int MarkAim::findUndecidedArea(vector<Rect> rect){
     int n = 0;
    if(rect.size()!=0){
        for(int i=0;i<rect.size();i++){
            if(rect[i].area()<=rect[n].area()) n = i; 
        }
        return n;
    }else{
        return -1;
    }
}

int MarkAim::find_R(vector<Rect> rect, Mat img){
    if(rect.size()!=0){
        for(int i = 0;i<rect.size();i++){
            Mat R,resultImg;
            resize(imread("../8.png"),R,rect[i].size());
            resultImg.create(1,1,CV_32FC1);
            matchTemplate(img(rect[i]), R, resultImg, CV_TM_CCOEFF_NORMED);
            //normalize(resultImg, resultImg, 0, 1, NORM_MINMAX);
            cout<<"result: "<<resultImg<<endl;
            if(resultImg.at<float>(0)>0.7) return i;
        }
        return -1;
    }
    return -1;
}

float MarkAim::calAngle(Point2f armor_center, Point2d center){
    float angle;
    if(armor_center.y - center.y!=0){
        if(armor_center.x - center.x!=0) 
            angle= atan((armor_center.x - center.x)/(center.y -armor_center.y))*180/M_PI;
        else if((armor_center.x - center.x==0) && armor_center.y - center.y>0)
                angle = 180;
        else angle = 0;
    }
    else if((armor_center.x - center.x)>0 && armor_center.y - center.y==0)    
        angle = 90;
    else if((armor_center.x - center.x)<0 && armor_center.y - center.y==0)
        angle = -90;
    return angle;    
}

Point2d MarkAim::calLocation(float angle,float radius,Point2d center){
    if(angle == 180){
        return Point2d(center.x,center.y + radius);
    }
    else if(angle == 0){
        return Point2d(center.x , center.y - radius);
    }
    else{
        int diff_y = radius * cos(angle*M_PI/180);
        int diff_x = radius * sin(angle*M_PI/180);
        if(angle > 0) return Point2d(center.x + diff_x,center.y - diff_y);
        else return Point2d(center.x + diff_x,center.y + diff_y); 
    }
}

void MarkAim::setImage(Mat &input, Mat &Output){
    Mat channel[3];
    split(input, channel);
    Mat Mask = channel[2];
    Mat diff = channel[2] - channel[1];
    threshold(Mask, Mask, 40, 255, THRESH_BINARY);
    threshold(diff, diff, 100, 255, THRESH_BINARY);
    bitwise_and(Mask, diff, Output);
}

int MarkAim::locateArmor(Mat src,Mat Mask,RotatedRect real_armor,vector<Point2f> &armor_points){
    vector<Rect> undecided_area;
    vector<Rect> unknown;
    vector<RotatedRect> armor;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarcy;
    findContours(Mask, contours, hierarcy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    Rect temp;
    for(int i=0;i<contours.size();i++){
        if(contours[i].size()>5){
            temp = boundingRect(contours[i]);//寻找最小外接矩形
            if(temp.area()<2000&&count==2){
                unknown.push_back(temp);
            }
            if(temp.area()>20000) continue;
            if(temp.area()<20000&&temp.area()>2000)
                undecided_area.push_back(temp); 
        }
    }
    if(count==1){
        center_box_index = find_R(unknown,src);
        if(center_box_index == -1){
            count = 1;
            return -3;
        }
        center = Point2d(unknown[center_box_index].x+unknown[center_box_index].width/2,unknown[center_box_index].y+unknown[center_box_index].height/2);
    }
    count++;
    Mat src;
    RotatedRect temp1;
    Point2f* points = new Point2f[4];
    int n = findUndecidedArea(undecided_area);
    if(n!=-1){
        src = Mask(undecided_area[n]);
        findContours(src, contours, hierarcy, CV_RETR_LIST, CHAIN_APPROX_SIMPLE);
        for(int i=0;i<contours.size();i++){
            temp1 = (minAreaRect(contours[i]));
            if(temp1.size.area()<1000) continue;
            armor.push_back(temp1);
        }
        int m = findUndecidedArea(armor);
        if(m!=-1){
            real_armor = armor[m];
            armor[m].points(points);
            for (int j = 0; j < 4; j++){
                points[j].x += undecided_area[n].x;
                points[j].y += undecided_area[n].y;
                armor_points.push_back(points[j]);
            }
            Point2f center_armor;
            center_armor.x = armor[m].center.x + undecided_area[n].x;
            center_armor.y = armor[m].center.y + undecided_area[n].y;
            radius = distance(center_armor, center);
            return 0;
        }else return -2;
    }else return -1;
}

BaseAim::AimResult MarkAim::Aim(RotatedRect real_armor,vector<Point2f> armor_points,float current_pitch,float current_yaw,Point2f &ctrl_angel){
    if(armor_points.size()!=0){
        float angle;
        angle = calAngle(real_armor.center , center);
        if(real_armor.size.width>real_armor.size.height){
            pnpSolver.pushPoints2D(armor_points[1]);
            pnpSolver.pushPoints2D(armor_points[2]);
            pnpSolver.pushPoints2D(armor_points[3]);
            pnpSolver.pushPoints2D(armor_points[0]);
        }else{
            pnpSolver.pushPoints2D(armor_points[0]);
            pnpSolver.pushPoints2D(armor_points[1]);
            pnpSolver.pushPoints2D(armor_points[2]);
            pnpSolver.pushPoints2D(armor_points[3]);
        }
        pnpSolver.solvePnP();
        pnpSolver.clearPoints2D();
        Point3d tvec = pnpSolver.getTvec();
        float time_delay = tvec.z / 27;
        float angle_offset = 0.06*time_delay;
        
        float predict_angle = angle + angle_offset;
        if(predict_angle>180)   predict_angle = -(360 - predict_angle);
        Point2d predict_location = calLocation(predict_angle,radius,center);
        if(real_armor.size.width>real_armor.size.height){
            pnpSolver.pushPoints2D(Point2d(predict_location.x - real_armor.size.width/2,predict_location.y - real_armor.size.height/2));
            pnpSolver.pushPoints2D(Point2d(predict_location.x + real_armor.size.width/2,predict_location.y - real_armor.size.height/2));
            pnpSolver.pushPoints2D(Point2d(predict_location.x + real_armor.size.width/2,predict_location.y + real_armor.size.height/2));
            pnpSolver.pushPoints2D(Point2d(predict_location.x - real_armor.size.width/2,predict_location.y + real_armor.size.height/2));
        }else
        {
            pnpSolver.pushPoints2D(Point2d(predict_location.x - real_armor.size.height/2,predict_location.y - real_armor.size.width/2));
            pnpSolver.pushPoints2D(Point2d(predict_location.x + real_armor.size.height/2,predict_location.y - real_armor.size.width/2));
            pnpSolver.pushPoints2D(Point2d(predict_location.x + real_armor.size.height/2,predict_location.y + real_armor.size.width/2));
            pnpSolver.pushPoints2D(Point2d(predict_location.x - real_armor.size.height/2,predict_location.y + real_armor.size.width/2));
        }
        pnpSolver.solvePnP();
        pnpSolver.clearPoints2D();
        Point3d predict_tvec = pnpSolver.getTvec();
        ctrl_angel = calPitchAndYaw(predict_tvec.x,predict_tvec.y, predict_tvec.z, predict_tvec.z/63, -120, 170, current_pitch, current_yaw);
        return AimResult::AIM_TARGET_FOUND;
    }
    return AimResult::AIM_IMAGE_ERROR;
}
