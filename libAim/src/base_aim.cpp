#include "base_aim.h"

void BaseAim::openPredict(){
    isPredict = true;
}

void BaseAim::setEnemyColor(int enemyColor){
    this->enemyColor = enemyColor;
}

void BaseAim::setTimeDelay(double timeDelay){
    this->timeDelay = timeDelay;
}

void BaseAim::setImgSize(Size imgSize){
    IMG_WIDTH = imgSize.width;
    IMG_HEIGHT = imgSize.height;

}

Point2f BaseAim::calPitchAndYaw(float x_offset, float y_offset, float z_offset, float currPitch, float currYaw, bool useExtrinsicGuess, int flags){
    pnpSolver.solvePnP(useExtrinsicGuess, flags);
	//pnpSolver.showParams();
    pnpSolver.clearPoints2D();
    Point3d tvec = pnpSolver.getTvec();
    
    Point2f angle;
    angle.x = ImageTool::gravityKiller((tvec.z + z_offset)/1000.0, (tvec.y + y_offset)/1000.0, 15, currPitch);
    angle.y = -atanf((tvec.x + x_offset) / (tvec.z + z_offset))*180/CV_PI;
    return angle;
}

Point2f BaseAim::calPitchAndYaw(float x, float y, float z, float x_offset, float y_offset, float z_offset, float currPitch, float currYaw){
    Point2f angle;
    angle.x = ImageTool::gravityKiller((z + z_offset)/1000.0, (y + y_offset)/1000.0, 15, currPitch);
    angle.y = -atanf((x + x_offset) / (z + z_offset))*180/CV_PI;
    return angle;
}

RotatedRect BaseAim::adjustRRect(const RotatedRect & rect){
    const Size2f & s = rect.size;
    if (s.width < s.height)
        return rect;
    return RotatedRect(rect.center, Size2f(s.height, s.width), rect.angle + 90.0);
}

RotatedRect BaseAim::boundingRRect(const RotatedRect & left, const RotatedRect & right){
    const Point & pl = left.center, & pr = right.center;
    Point2f center = (pl + pr) / 2.0;
    Size2f wh_l = left.size;
    Size2f wh_r = right.size;
    float width = ImageTool::calc2PointDistance(pl, pr) - (wh_l.width + wh_r.width) / 2.0;
    float height = std::max(wh_l.height, wh_r.height);
    float angle = std::atan2(right.center.y - left.center.y, right.center.x - left.center.x);
    return RotatedRect(center, Size2f(width, height), angle * 180 / CV_PI);
}

bool BaseAim::makeRectSafe(Rect & rect){
    if (rect.x < 0)
        rect.x = 0;
    if (rect.x + rect.width > IMG_WIDTH)
        rect.width = IMG_WIDTH - rect.x;
    if (rect.y < 0)
        rect.y = 0;
    if (rect.y + rect.height > IMG_HEIGHT)
        rect.height = IMG_HEIGHT - rect.y;
    if (rect.width <= 0 || rect.height <= 0)
        return false;
    return true;
}

float BaseAim::datafilter(float newdata){
    new_data = newdata;
    if (new_data - old_data > 0 )       
        new_flag = 1;
    else new_flag = 0;
    if (new_flag == old_flag)          
    {
        if (abs (new_data - old_data) > Threshold_1)    
            num_x += 5;
        if (num_x >= Threshold_2)      
            k_x += 0.2; 
    }
    else                
    {
        num_x = 0;      
        k_x = 0.2;
        old_flag = new_flag; 
    } 
    if (k_x > 0.95)  {
        k_x = 0.95;    
        // num_x = 0;
        // k_x = 0.2;
    }
    new_data = (1-k_x) * old_data + k_x * new_data;   
    old_data = new_data;      
    return old_data;
}

float BaseAim::datafilterII(float newdata){
    next_input_value = newdata;
    if(key == 0){
        memset(xv,0,3);
        memset(yv,0,3);
        key = 0;
    }
    xv[0] = xv[1]; xv[1] = xv[2]; 
    xv[2] = next_input_value / gain;
    yv[0] = yv[1]; yv[1] = yv[2]; 
    yv[2] =   (xv[0] + xv[2]) + 2 * xv[1] + ( -0.4128015981 * yv[0]) + (  1.1429805025 * yv[1]);
    key++;
    return yv[2];
}

bool BaseAim::broadenRect(Rect & rect){
    rect.x -= rect.width/2;
    rect.width += rect.width;
    rect.y -= rect.height/2;
    rect.height += rect.height;
    return makeRectSafe(rect);
}
