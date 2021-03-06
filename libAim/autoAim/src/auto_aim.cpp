#include "auto_aim.h"
#include <string>
#include <iostream>
#include <algorithm>
#define DEBUG
//#define COLLECT_DATA
AutoAim::AutoAim(){}


AutoAim::~AutoAim(){}

void AutoAim::init(Aim_assistant* checker){
    id_checker_ = checker;
    IMG_WIDTH = 1280;
    IMG_HEIGHT = 720;
    resize_count_ = 0;
    
    //初始化三维坐标点(小装甲)
    pnpSolver.clearPoints3D();
    pnpSolver.pushPoints3D(-65, -33, 0);
    pnpSolver.pushPoints3D(65,  -33, 0);
    pnpSolver.pushPoints3D(65, 33, 0);
    pnpSolver.pushPoints3D(-65, 33, 0);

    //初始化相机参数
    pnpSolver.setCameraMatrix(1044.11801, 0., 637.0385, 0.,1046.6575,467.3094, 0., 0., 1.);
    pnpSolver.setDistortionCoef(-0.1018, 0.1015, -0.0135, -0.00073262,0.000241165);
    aim_predict_.modelInit();
    best_center_.x=-1;
    recognize_wiggle_.init();

    
}
Point2d cal_x_y(RotatedRect &rect, int is_up){
    float angle = (90-rect.angle)*CV_PI/180;
    Point2d point;
    if(is_up){
        point.x = rect.center.x + rect.size.height/2*cos(angle);
        point.y = rect.center.y - rect.size.height/2*sin(angle);
    } else {
        point.x = rect.center.x - rect.size.height/2*cos(angle);
        point.y = rect.center.y + rect.size.height/2*sin(angle);
    }
    return point;
}

void AutoAim::resetRoi(){
    rect_roi_.x = 0;
    rect_roi_.y = 0;
    rect_roi_.width = 1280;
    rect_roi_.height = 720;
}

//图像预处理
bool AutoAim::setImage(Mat &img){
    cout<<enemyColor<<"  enemyColor"<<endl;
    if(img.empty()) return false;
    img.copyTo(image_);
    Mat channel[3], diff;
    resetRoi();
    mask_ = img(rect_roi_);
    split(mask_, channel);
    if(img.empty()) return false;
    if(enemyColor==color_red){
        cout<<"red"<<endl;
        int thresh =100 , substract_thresh =    50;
        diff = channel[2] - channel[0];
        
        #ifdef DEBUG
        imshow("diff",diff);
        #endif

        threshold(diff, diff, substract_thresh, 255, THRESH_BINARY);
        Mat element = getStructuringElement( MORPH_RECT, Size(1, 5));
        Mat element2 = getStructuringElement( MORPH_RECT, Size(2,3));
        dilate(diff,mask_,element,Point(-1,-1));
        // morphologyEx(mask_,mask_,MORPH_OPEN,element2);

        #ifdef DEBUG
        imshow("mask_",mask_);
        waitKey(1);
        #endif

        return true;
    }
    else if(enemyColor == color_blue){
        cout<<"blue"<<endl;
        int thresh = 110, substract_thresh = 150;
        threshold(channel[1], channel[1],thresh, 255, cv::THRESH_BINARY);
        diff = channel[0] - channel[2];
        threshold(diff,diff,substract_thresh,255,THRESH_BINARY);
        Mat element = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9, 9));
        Mat element2 = getStructuringElement(MORPH_ELLIPSE,Size(1,4));
        dilate(diff, diff, element,Point(-1,-1),1);

        #ifdef DEBUG
        imshow("diff",diff);
        #endif

        bitwise_and(diff, channel[1], mask_);
        dilate(mask_,mask_,element2,Point(-1,-1));

        #ifdef DEBUG
        imshow("mask_",mask_);
        waitKey(1);
        #endif

        return true;
    }
    // #ifdef DEBUG

    // #endif
}

//寻找灯管
void AutoAim::findLampRect(vector<RotatedRect> &pre_armor_lamps){
    pre_armor_lamps.clear();
    vector<vector<Point> > contours;
    vector<Vec4i> hierarcy;
    //寻找轮廓，将满足条件的轮廓放入待确定的数组中去
    findContours(mask_, contours, hierarcy, CV_RETR_EXTERNAL, CHAIN_APPROX_NONE);
    RotatedRect temp;
    float lastCenterX = 0, lastCenterY = 0;
    cout<<"counter size :"<<contours.size()<<endl;
    if(LIKELY(contours.size()<200)){
        for(int i=0;i<contours.size();i++){
            if(contours[i].size()>5){
                temp = adjustRRect(fitEllipseDirect(contours[i]));
                if(temp.angle >= 90) temp.angle = temp.angle -180;
                if(temp.size.height < 12) continue;
                if(temp.size.area()<50 )continue;
                if(abs(temp.angle)>45) continue;//旋转矩形角度小于45度，则忽略
                if(float(temp.size.height)/float(temp.size.width) <2) continue; 
                ellipse(image_,temp,Scalar(0,255,0),1);
                pre_armor_lamps.push_back(temp);
            }
        }
    }
    #ifdef DEBUG
    imshow("img",image_);
    #endif
}
void AutoAim::matchLamps(vector<RotatedRect> &pre_armor_lamps, vector<RotatedRect> &real_armor_lamps){
    
    //权重
    float inside_angle_ratio;
    float params_max_height_ratio, params_max_distance_divide_height_ratio, params_min_distance_divide_height_ratio, 
            params_max_angle_tolerance, params_max_inside_angle_ratio;
    float angle_difference_weight,height_ratio_weight,inside_angle_ratio_weight;
    //权重
    angle_difference_weight=6; //灯条角度差权重
    height_ratio_weight=2;  //灯条高度之比权重
    inside_angle_ratio_weight=3;  //内角权重
    params_max_height_ratio= 2; //一对灯条的高度之比
    params_min_distance_divide_height_ratio = 1; //最小灯条距离与灯条高度之比
    params_max_distance_divide_height_ratio= 5; //最大灯条距离与灯条高度之比
    params_max_inside_angle_ratio= 0.4; //一对灯条最大内角
    params_max_angle_tolerance=5; //一对灯条最大允许角度差
    vector<float> low_cost( pre_armor_lamps.size(),0x3f3f3f3f);
    vector<float> best_match_index( pre_armor_lamps.size(),-1);
    //计算灯管匹配之间的花费
    float height_divided_ratio,distance, average_height, angle_difference, total_difference,inside_angle,distance_divide_height_ratio;
    int i,j;

    for(i=0; i< pre_armor_lamps.size(); i++){
        int current_index = -1;
        const RotatedRect &current = pre_armor_lamps[i];
        if(current.size.height < 12) continue;
        for(j=i+1;j< pre_armor_lamps.size(); j++){
            //计算比例，筛选灯管
            const RotatedRect &previous = pre_armor_lamps[j];
            if(previous.size.height < 12) continue;
    
            //灯条角度差超过设定角度忽略
            angle_difference = fabsf(previous.angle - current.angle);
     
            if(angle_difference > params_max_angle_tolerance) continue;

            //y差值与x差值超过设定值忽略
            if(current.center.x - previous.center.x == 0) inside_angle_ratio=100;
            else inside_angle_ratio = fabsf(current.center.y-previous.center.y)/fabsf(current.center.x-previous.center.x);
           
            if(inside_angle_ratio > params_max_inside_angle_ratio) continue;
            //内角小于设定角度忽略

            //两灯条高度比例不在范围内则忽略
             if(previous.size.height > current.size.height)
                height_divided_ratio = previous.size.height*1.0f/current.size.height;
            else
                height_divided_ratio = current.size.height*1.0f/previous.size.height;
            
            if(height_divided_ratio > params_max_height_ratio) continue;

            //灯条之间的距离与灯条的平均长度之比需要在一定范围之内
            distance = ImageTool::calc2PointDistance(previous.center, current.center);
            average_height = (previous.size.height + current.size.height) / 2.0;
            distance_divide_height_ratio = distance / average_height;
            
            if(distance_divide_height_ratio > params_max_distance_divide_height_ratio || 
                    distance_divide_height_ratio < params_min_distance_divide_height_ratio) continue;
            //角度差约束会在几度之内，高度比例约束会在1到1.x之内，内角约束大致在几十度到90度(目前70-90)，
            //实际上用90度减去角度应该在0-几十度之内，且越小越好
            //用归一化的值算最后的花费，避免不同值的取值范围不同
            total_difference = angle_difference_weight * (angle_difference/params_max_angle_tolerance) //角度花费
                        + height_ratio_weight * ((height_divided_ratio-1)/(params_max_height_ratio-1)) //高度比例花费
                        + inside_angle_ratio_weight * (inside_angle_ratio/params_max_inside_angle_ratio); //内角花费

              //更新j代表的灯条的最小花费
            if(low_cost.at(j) > total_difference)
                low_cost.at(j) = total_difference;

            //更新i代表的当前灯条的最优匹配
            if(low_cost.at(i) > total_difference){
                low_cost.at(i) = total_difference;
                current_index = j;
            }            
        }
        if(current_index==-1) continue;
        else {
            best_match_index.at(i) = current_index;
            best_match_index.at(current_index) = i;
        }
    }
    
    for(i=0; i< pre_armor_lamps.size(); i++){
        int index = best_match_index[i];
        if(index == -1 || index <= i) continue;
        if(i == best_match_index[index]){
            real_armor_lamps.push_back(pre_armor_lamps[i]);
            real_armor_lamps.push_back(pre_armor_lamps[index]);
        }
    }

    // if(real_armor_lamps.size() == 0){
    //     int i , j;
    //     for(i=0; i< pre_armor_lamps.size(); i++){
    //         int current_index = -1;
    //         const RotatedRect &current = pre_armor_lamps[i];
    //         //灯条角度小于一定阈值排除
    //         if((current.angle < 0 && current.angle > -5) || (current.angle > 0 && current.angle < 5)) continue;
    //         if(current.size.height<100) continue;
    //         for(j=i+1;j< pre_armor_lamps.size(); j++){
    //             //计算比例，筛选灯管
    //             const RotatedRect &previous = pre_armor_lamps[j];
    //             if(previous.size.height<100) continue;
    //             if((previous.angle < 0 && previous.angle > -5) || (previous.angle > 0 && previous.angle < 5)) continue;
    //             //灯条角度必须一正一负
    //             if((current.angle > 0 && previous.angle>0) || (current.angle < 0 && previous.angle < 0)) continue;
                
    //             //灯条角度差小于设定角度忽略
    //             angle_difference = fabsf(previous.angle - current.angle);
    //             if(angle_difference < 10) continue;
    //             if(angle_difference > 35) continue;
    //             //y差值与x差值超过设定值忽略
    //             if(current.center.x - previous.center.x == 0) inside_angle_ratio=100;
    //             else inside_angle_ratio = fabsf(current.center.y-previous.center.y)/fabsf(current.center.x-previous.center.x);
    //             cout<<" peng cheng !!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
    //             if(inside_angle_ratio > params_max_inside_angle_ratio) continue;
    //             //两灯条高度比例不在范围内则忽略
    //             if(previous.size.height > current.size.height)
    //                 height_divided_ratio = previous.size.height*1.0f/current.size.height;
    //             else
    //                 height_divided_ratio = current.size.height*1.0f/previous.size.height;
    //             if(height_divided_ratio > 1.5) continue;

    //             distance = ImageTool::calc2PointDistance(previous.center, current.center);
    //             average_height = (previous.size.height + current.size.height) / 2.0;
    //             distance_divide_height_ratio = distance / average_height;
                
    //             if(distance_divide_height_ratio > params_max_distance_divide_height_ratio ||
    //                      distance_divide_height_ratio < params_min_distance_divide_height_ratio) continue;

    //             total_difference = angle_difference_weight * ((angle_difference -10) /15) //角度花费
    //                     + height_ratio_weight * ((height_divided_ratio-1)/(1.5-1)); //高度比例花费
    //                     //+ inside_angle_ratio_weight * (inside_angle_ratio/params_max_inside_angle_ratio); //内角花费
    //             if(low_cost.at(j) > total_difference)
    //                 low_cost.at(j) = total_difference;

    //             //更新i代表的当前灯条的最优匹配
    //             if(low_cost.at(i) > total_difference){
    //                 low_cost.at(i) = total_difference;
    //                 current_index = j;
    //             }            
    //         }
    //         if(current_index==-1) continue;
    //         else {
    //             best_match_index.at(i) = current_index;
    //             best_match_index.at(current_index) = i;
    //         }
    //     }
    //     for(i=0; i< pre_armor_lamps.size(); i++){
    //         int index = best_match_index[i];
    //         if(index == -1 || index <= i) continue;
    //         if(i == best_match_index[index]){
    //             real_armor_lamps.push_back(pre_armor_lamps[i]);
    //             real_armor_lamps.push_back(pre_armor_lamps[index]);
    //         }
    //     }
        
    //     if(real_armor_lamps.size() != 0){special_condition_ = true;}
    // } 
    // //只能看见两个装甲板中间部分的情况
   
}
void AutoAim::selectArmorH(vector<RotatedRect> real_armor_lamps){
    float pre=1000;//预定义
    int target_index=-1;//目标索引
    bool detect_hero;
    best_center_.x=-1;
    for(int i=0; i<real_armor_lamps.size(); i+=2){
        if(i+1 >= real_armor_lamps.size()) break;

        Rect armor_area;
        if(real_armor_lamps[i].center.x > real_armor_lamps[i+1].center.x){
            swap(real_armor_lamps[i],real_armor_lamps[i+1]);//确保偶数为左灯条，奇数为右灯条
        }
        //优先选择趋近于图像中心的装甲板,达到操作手指示优先级的功能
        float y = fabs(360 - (real_armor_lamps[i].center.y + real_armor_lamps[i+1].center.y)/2);
        float x = fabs(640-(real_armor_lamps[i].center.x + real_armor_lamps[i+1].center.x)/2);
        y = y / 360;
        x = x / 640;
        float score = 0.5 * y + 0.5 * x;
        if(score < pre){
            pre = score;
            target_index = i;
        }
    }   
    if(!special_condition_){
        if(target_index != -1){
            float y0 = (real_armor_lamps[target_index].center.y + real_armor_lamps[target_index+1].center.y)/2;
            float x0 = fabsf(real_armor_lamps[target_index].center.x-real_armor_lamps[target_index+1].center.x);
            if((float(x0)/float(y0))>0.15 && (float(x0)/float(y0))<0.39 && 
                (float(x0)/float(real_armor_lamps[target_index].size.height)) > 3.2 && (float(x0)/float(real_armor_lamps[target_index].size.height) < 4.08)){
                detect_hero = true;
            }
            if(detect_hero){
                pnpSolver.clearPoints3D();
                pnpSolver.pushPoints3D(-112, -35, 0);
                pnpSolver.pushPoints3D(112,  -35, 0);
                pnpSolver.pushPoints3D(112, 35, 0);
                pnpSolver.pushPoints3D(-112, 35, 0);
	            cout<<"aimming hero !!!!!!"<<endl;
            }else{
                cout<<"aimming infantry!!"<<endl;
                pnpSolver.clearPoints3D();
                pnpSolver.pushPoints3D(-65, -33, 0);
                pnpSolver.pushPoints3D(65,  -33, 0);
                pnpSolver.pushPoints3D(65, 33, 0);
                pnpSolver.pushPoints3D(-65, 33, 0);
            }
            best_center_.x = (real_armor_lamps[target_index].center.x + real_armor_lamps[target_index+1].center.x)/2 + rect_roi_.x;
            best_center_.y = (real_armor_lamps[target_index].center.y + real_armor_lamps[target_index+1].center.y)/2 + rect_roi_.y;
        }
        if(best_center_.x!=-1){
            clock_t finish = clock();
            best_lamps_[0] = real_armor_lamps[target_index];
            best_lamps_[1] = real_armor_lamps[target_index+1];
            best_lamps_[0].center.x+=rect_roi_.x;
            best_lamps_[0].center.y+=rect_roi_.y;
            best_lamps_[1].center.x+=rect_roi_.x;
            best_lamps_[1].center.y+=rect_roi_.y;
            rect_roi_.x = (best_lamps_[0].center.x + best_lamps_[1].center.x)/2 - (best_lamps_[1].center.x - best_lamps_[0].center.x);
            rect_roi_.y = (best_lamps_[0].center.y + best_lamps_[1].center.y)/2 - (best_lamps_[0].size.height + best_lamps_[1].size.height)/2;
            rect_roi_.height = best_lamps_[0].size.height + best_lamps_[1].size.height;
            rect_roi_.width = 2*(best_lamps_[1].center.x - best_lamps_[0].center.x);
            if(!makeRectSafe(rect_roi_)){
                resetRoi();
	        }
        }else{
            if(!broadenRect(rect_roi_)){ //|| resize_count_>5){
                resetRoi();
            }
        }
    }
}

void AutoAim::selectArmor(vector<RotatedRect> real_armor_lamps){
    bool possible_hero = false;
    int second_best_score_index = -1;
    float differenciate_score = -1;
    float differenciate_score_min = 0x3f3f3f3f;
    int hero_second_best_score_index = -1;
    float hero_differenciate_score = -1;
    float hero_differenciate_score_min = 0x3f3f3f3f;
    int score_index = -1;
    int hero_score_index = -1;
    vector<float> score_lamps;
    vector<float> hero_score_lamps;
    float lower_y=0x3f3f3f3f;
    float lower_hero = 0x3f3f3f3f;
    int lower_index=-1;
    int hero_index=-1;
    int sp_index=-1;
    best_center_.x=-1;
    vector<int> possible_hero_index;
    is_right_ = false;
    vector<int> armor_detected;
    if(special_condition_){
        cout<<"im in special condition !!!!!!!!!!!!!!"<<endl;
        pnpSolver.clearPoints3D();
        pnpSolver.pushPoints3D(-65, -33, 0);
        pnpSolver.pushPoints3D(65,  -33, 0);
        pnpSolver.pushPoints3D(65, 33, 0);
        pnpSolver.pushPoints3D(-65, 33, 0);
    }else{
        #ifdef DEBUG
        imshow("image_",image_);
        waitKey(1);
        #endif

        //装甲板击打决策
        //若视野中出现
        for(int i=0; i<real_armor_lamps.size(); i+=2){
            if(i+1 >= real_armor_lamps.size()) break;
            if(real_armor_lamps[i].center.x > real_armor_lamps[i+1].center.x){
                swap(real_armor_lamps[i],real_armor_lamps[i+1]);//确保偶数为左灯条，奇数为右灯条
            }
            float y = float(real_armor_lamps[i].size.height + real_armor_lamps[i+1].size.height)/2;
            //float y = 720 - (real_armor_lamps[i].center.y + real_armor_lamps[i+1].center.y)/2;
            float x = fabs(640-float(real_armor_lamps[i].center.x + real_armor_lamps[i+1].center.x)/2);
            float width_difference = ((real_armor_lamps[i].size.width < real_armor_lamps[i+1].size.width)? 
                    float(real_armor_lamps[i].size.width / real_armor_lamps[i+1].size.width) :
                    float(real_armor_lamps[i+1].size.width / real_armor_lamps[i].size.width));
            #ifdef DEBUG
            circle(image_,real_armor_lamps[i].center,2,Scalar(0,255,0),2);
            putText(image_,to_string(i),real_armor_lamps[i].center,2,5,Scalar(0,0,255),5);
            #endif
            y = 1- (y / 720);
            cout<<"    y   ::::::::: "<<y<<endl;
            x = x / 640;
            width_difference = fabsf(1-width_difference);
            float score =x*0.4 + y*0.6;// + x*0.3;
            score_lamps.push_back(score);
            if(score < lower_y){
                lower_y = score;
                lower_index =  i;
                score_index = i/2;
            }
        }
        //如果找到了最优装甲板
        if(score_index!=-1){ 
            for(int i=0; i<score_lamps.size(); i++){
                if(i != score_index){
                    differenciate_score = fabsf(score_lamps[i] - score_lamps[score_index]);
                    if(differenciate_score < differenciate_score_min){
                        differenciate_score_min = differenciate_score;
                        second_best_score_index = i;
                    }
                }
            }
            cout<<"differenciate_score_min: "<<differenciate_score_min<<endl;
            if(differenciate_score_min < 0.3 && second_best_score_index!=-1 && score_index!=-1){
                cout<<"+++++++++++++"<<endl;
                target_in_ = true;
                lower_index =2* score_index;
                if(lock_on_target_){
                    if(real_armor_lamps[2*score_index].center.x <= real_armor_lamps[2*second_best_score_index].center.x)
                        lower_index =2* score_index;
                    else
                        lower_index = 2*second_best_score_index;
                }
                else{
                    if(real_armor_lamps[2*score_index].center.x >= real_armor_lamps[2*second_best_score_index].center.x)
                        lower_index = 2*score_index;
                    else
                        lower_index = 2*second_best_score_index;
                }
                last_choose_center_ = real_armor_lamps[lower_index].center;
            }else{
                target_in_ = false;
            }
            if(last_targe_choose_condition_ == true && target_in_ == false){
                cout<<" change))))))))))))))))))))))))))))))"<<endl;
                if(last_choose_center_.x != -1 && lower_index!= -1){
                    cout<<"aaa"<<fabsf(real_armor_lamps[lower_index].center.x - last_choose_center_.x)<<endl;
                   if(fabsf(real_armor_lamps[lower_index].center.x - last_choose_center_.x) > 50){
                        lock_on_target_ = !lock_on_target_;
                   }
                }
            }
            
            last_targe_choose_condition_ = target_in_;

            //开始装甲板id识别
            if(lower_index!=-1){
                Rect armor_area;
    
                 //计算装甲板灯条四个点像素坐标
                Point2d left_up = cal_x_y(real_armor_lamps[lower_index],1);
                Point2d left_down = cal_x_y(real_armor_lamps[lower_index],0);
                Point2d right_up = cal_x_y(real_armor_lamps[lower_index+1],1);
                Point2d right_down = cal_x_y(real_armor_lamps[lower_index+1],0);

                 //选定装甲板中心区域
                 //计算视野中全部装甲板id
                armor_area.x = left_up.x;

                int y = (left_up.y + right_up.y) / 2 - 0.4 * real_armor_lamps[lower_index].size.height;
                if(y<0) armor_area.y =-y;                                                    
                else armor_area.y = y;                                                       
                if(left_down.y>right_down.y){                                                
                    if((1.9) * (left_down.y - right_up.y)+armor_area.y<720)                     
                        armor_area.height = (1.9) * (left_down.y - right_up.y);
                    else{
                        armor_area.height = 720 - armor_area.y;
                    }
                }else{
                    if((1.9) * (right_down.y - left_up.y)+armor_area.y<720)
                        armor_area.height = (1.9) * (right_down.y - left_up.y);
                    else{
                        armor_area.height = 720 - armor_area.y;
                    }
                }    
                Mat img_a;
                armor_area.width = abs(right_up.x - left_up.x);
                clock_t begin, finish;
                begin = clock() ;
                int number = -1;
                if(armor_area.width != 0 && armor_area.height != 0 ){
                    number = id_checker_->checkArmor(image_(armor_area)); 
                    #ifdef COLLECT_DATA
                    cv::resize(image_(armor_area),img_a,cv::Size(32,32));
                    for(int i = 0;i<img_a.rows;i++){
                        for(int j = 0;j<img_a.cols;j++){
                            for(int k = 0; k<3;k++){
                                int tmp = (uchar)img_a.at<Vec3b>(i,j)[k]*1.2+10;
                                if(tmp>255) img_a.at<Vec3b>(i,j)[k] = 2*255 - tmp;
                                else img_a.at<Vec3b>(i,j)[k] = tmp;
                            }
                        }
                    }
                    cvtColor(img_a,img_a,COLOR_BGR2GRAY); //输入CNN模型预测 0-4 对应 1-5 5种装甲板
                    imwrite("../armor_data/3/"+to_string(path_number_)+"eg.png",img_a);
                    #endif
                }

                path_number_++;
                finish = clock();
                cout<<"CNN time cost: "<<(double)(finish - begin) / CLOCKS_PER_SEC<<endl;
                armor_detected.push_back(number);
                cout<<"number: "<<lower_index<<"   "<<number<<endl;
            }

            #ifdef DEBUG
            imshow("image_",image_);
            #endif

            //通过识别的id与长宽比例共同判断最优解是否为大装甲
            float y0 = (real_armor_lamps[lower_index].center.y + real_armor_lamps[lower_index+1].center.y)/2;
            float x0 = fabsf(real_armor_lamps[lower_index].center.x-real_armor_lamps[lower_index+1].center.x);
            if((float(x0)/float(y0))>0.15 && (float(x0)/float(y0))<1.6 && 
                    (float(x0)/float(real_armor_lamps[lower_index].size.height)) > 2.5 && (float(x0)/float(real_armor_lamps[lower_index].size.height) < 4.08)){
                possible_hero = true;
            }else possible_hero = false;

           if((armor_detected[lower_index] == 1 || possible_hero )&& !hero_lock_)
               hero_index = lower_index;

            //英雄和近距离步兵同时存在，选择步兵
            // if(lower_index != -1){
            //     hero_lock_ = false;
            //     if((float(real_armor_lamps[lower_index].size.height + real_armor_lamps[lower_index+1].size.height)/2>100 &&
            //              fabs(640-(real_armor_lamps[lower_index].center.x + real_armor_lamps[lower_index+1].center.x)/2)<320)||infantry_lock_){
            //         hero_index = -1;
            //         hero_lock_ = true;
            //         infantry_lock_ = true;
            //         count_hero_++;
            //         if(count_hero_ > 150){
            //             count_hero_=0;
            //             hero_lock_ = false;
            //             infantry_lock_ = false;
            //         }
            //     }
            // }   
            if(special_condition_){
                float pre = 10000;
                for(int i=0; i<real_armor_lamps.size(); i+=2){
                    if(i+1 >= real_armor_lamps.size()) break;
                    Rect armor_area;
                    if(real_armor_lamps[i].center.x > real_armor_lamps[i+1].center.x){
                        swap(real_armor_lamps[i],real_armor_lamps[i+1]);//确保偶数为左灯条，奇数为右灯条
                    }

                    float y = 720 - (real_armor_lamps[i].center.y + real_armor_lamps[i+1].center.y)/2;
                    float x = fabs(1280-(real_armor_lamps[i].center.x + real_armor_lamps[i+1].center.x)/2);
                    y = y / 720;
                    x = x / 1280;
                    float score = x;
                    if(score < pre){
                        pre = score;
                        sp_index = i;
                    }
                }
                if(abs(640-real_armor_lamps[sp_index].center.x)<abs(640-real_armor_lamps[sp_index+1].center.x)){
                    best_center_.x = real_armor_lamps[sp_index].center.x - 0.5 * real_armor_lamps[sp_index].size.height;
                    best_center_.y = real_armor_lamps[sp_index].center.y;   
                    is_right_ = false;
                }else{
                    best_center_.x = real_armor_lamps[sp_index+1].center.x + 0.5 * real_armor_lamps[sp_index+1].size.height;
                    best_center_.y = real_armor_lamps[sp_index+1].center.y; 
                    is_right_ = true; 
                }
            }
            if(hero_index!=-1){
                lower_index = hero_index;
                pnpSolver.clearPoints3D();
                pnpSolver.pushPoints3D(-112, -35, 0);
                pnpSolver.pushPoints3D(112,  -35, 0);
                pnpSolver.pushPoints3D(112, 35, 0);
                pnpSolver.pushPoints3D(-112, 35, 0);
                count_++;
                int height = (real_armor_lamps[hero_index].size.height + real_armor_lamps[hero_index+1].size.height)/2;
                //当灯条高度小于5个像素点时放弃锁定，重新寻找合适目标
                if(LIKELY(height > 5)){
                
                    best_center_.x = (real_armor_lamps[hero_index].center.x 
                                + real_armor_lamps[hero_index+1].center.x)/2;
                    best_center_.y = (real_armor_lamps[hero_index].center.y 
                                + real_armor_lamps[hero_index+1].center.y)/2;
                }else{
                    count_=0;
                }
            }
            else if(lower_index == -1){
                count_=0;
            } 
            else if(lower_index != -1) {
	            cout<<" attacking Infantry!!!  :) "<<endl;
                pnpSolver.clearPoints3D();
                pnpSolver.pushPoints3D(-65, -33, 0);
                pnpSolver.pushPoints3D(65,  -33, 0);
                pnpSolver.pushPoints3D(65, 33, 0);
                pnpSolver.pushPoints3D(-65, 33, 0);
                count_++;
                int height = (real_armor_lamps[lower_index].size.height + real_armor_lamps[lower_index+1].size.height)/2;
                //当灯条高度小于5个像素点时放弃锁定，重新寻找合适目标
                if(height > 5){
                    best_center_.x = (real_armor_lamps[lower_index].center.x + real_armor_lamps[lower_index+1].center.x)/2;
                    best_center_.y = (real_armor_lamps[lower_index].center.y + real_armor_lamps[lower_index+1].center.y)/2;
                } else{
                    count_=0;
            	}
            }
    
            if(best_center_.x!=-1){
                if(special_condition_ && sp_index!=-1){
                    best_lamps_[0] = real_armor_lamps[sp_index];
                    best_lamps_[1] = real_armor_lamps[sp_index+1];
                }else{
                    best_lamps_[0] = real_armor_lamps[lower_index];
                    best_lamps_[1] = real_armor_lamps[lower_index+1];
	            }
            }
        }
    }
}

BaseAim::AimResult AutoAim::aim(Mat &src, float curr_pitch, float curr_yaw, Point2f &pitch_yaw,int is_predict,bool &if_shoot,float time_delay){

    bool is_kalman = true;
    if(best_center_.x!=-1){
        circle(src, best_center_, 20, Scalar(255,255,255), 5);
        count_++;
        Rect rect;
        if(!special_condition_){
            Point2d left_up = cal_x_y(best_lamps_[0],1);
            Point2d right_up = cal_x_y(best_lamps_[1],1);
            pnpSolver.pushPoints2D(left_up);//P1
            pnpSolver.pushPoints2D(right_up);//P3
            pnpSolver.pushPoints2D(cal_x_y(best_lamps_[1],0));//P2
            pnpSolver.pushPoints2D(cal_x_y(best_lamps_[0],0));//P4
        }else{
            special_condition_ = false;
            if(!is_right_){
                Point2d up = cal_x_y(best_lamps_[0],1);
                Point2d down = cal_x_y(best_lamps_[0],0);
                pnpSolver.pushPoints2D(Point2d(up.x - 1.2 * best_lamps_[0].size.height,up.y));
                pnpSolver.pushPoints2D(up);//P1
                pnpSolver.pushPoints2D(down);//P3
                pnpSolver.pushPoints2D(Point2d(down.x - 1.2 * best_lamps_[0].size.height,down.y));
            }else{
                Point2d up = cal_x_y(best_lamps_[1],1);
                Point2d down = cal_x_y(best_lamps_[1],0);
                pnpSolver.pushPoints2D(up);//P1
                pnpSolver.pushPoints2D(Point2d(up.x + 1.2 * best_lamps_[1].size.height,up.y));
                pnpSolver.pushPoints2D(Point2d(down.x + 1.2 * best_lamps_[1].size.height,down.y));
                pnpSolver.pushPoints2D(down);//P3               
            }
        }
        pnpSolver.solvePnP();
        pnpSolver.clearPoints2D();
        Point3d tvec = pnpSolver.getTvec();
        cout<<"x: "<<tvec.x<<"y: "<<tvec.y<<"z: "<<tvec.z<<"   current_yaw :"<<curr_yaw<<endl;
        if(is_predict && is_kalman){
	        pitch_yaw = calPitchAndYaw(tvec.x,tvec.y, tvec.z, tvec.z/63, -50, 170, curr_pitch, curr_yaw);
	        measurement_.at<float>(0) = curr_yaw + pitch_yaw.y;           
            if(count_==1){
                Mat state_post=(Mat_<float>(2, 1) << curr_yaw+pitch_yaw.y,0);
                aim_predict_.modelInit();
                aim_predict_.resetKfStatepost(state_post);
            } 
            Mat Predict = this->aim_predict_.predict(measurement_,time_delay);
            float predict_angle=Predict.at<float>(1)*(6*time_delay);
            recognize_wiggle_.getCondition(g_condition);
            cout<<int(g_condition)<<"  *****************g_condition**************"<<endl;

            if(g_condition!=RecognizeWiggle::CONDITION_NOR){
                if(g_condition == RecognizeWiggle::CONDITION_ABN){  //正常模式
                }
                if(g_condition == RecognizeWiggle::CONDITION_NOR_WIGGLE){ //正常扭腰
                    k_add_ = 0.15; //低抑制低通滤波
                    if(predict_angle>0.6)
                        predict_angle = 0;
                    cout<<"************* NOR_WIGGLE ***************** "<<endl;
                }
                if(g_condition == RecognizeWiggle::CONDITION_FAST_WIGGLE){ //快速扭腰
                    k_add_ = 0.14; //高抑制低通滤波
                    if(predict_angle>0.6)
                        predict_angle = 0;
                    cout<<"** FAST_WIGGLE ****************"<<endl;
                }
            }
            if_shoot=aim_predict_.shootLogic(pitch_yaw.y,Predict.at<float>(1),predict_angle,g_condition);
            pitch_yaw.y += predict_angle;
            recognize_wiggle_.setData(pitch_yaw.y); //更新扭腰判断数据输入
            if(g_condition != RecognizeWiggle::CONDITION_NOR && g_condition!= RecognizeWiggle::CONDITION_ABN){ //扭腰模式下低通滤波
                pitch_yaw.y = dataDealer(pitch_yaw.y);
                cout<<"** CNOR **********************"<<endl;
            }
        

        }else{   
            pitch_yaw = calPitchAndYaw(tvec.x, tvec.y, tvec.z, tvec.z/45, -50, 170, curr_pitch, curr_yaw);
        }
        return AIM_TARGET_FOUND;
    }
    count_ = 0;
    return AIM_TARGET_NOT_FOUND;
}

float AutoAim::dataDealer(float new_num){
    if (new_num - old_num_ > 0 )         //判断是否反向
        new_inverse_flags_ = 1;
    else new_inverse_flags_ = 0;
    if (new_inverse_flags_ == old_inverse_flags_)          //持续同向改变
    {
        if (abs (new_num - old_num_) > threshold_min_)    
            num_ += 5;
        if (num_ >= threshold_max_)      
            k_ += k_add_;  //增大对测量数据的置信度
    }
    else                
    {
        num_ = 0;      
        k_ = 0.6;
        old_inverse_flags_ = new_inverse_flags_; 
    } 
    if (k_ >= 0.9)  { 
        k_ = 0.97;    
    }
    new_num = (1-k_) * old_num_ + k_ * new_num;   //correct
    old_num_ = new_num;      
    return old_num_;
}
