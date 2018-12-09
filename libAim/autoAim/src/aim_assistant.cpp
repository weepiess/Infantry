#include "aim_assistant.h"
using namespace std;
using namespace cv;
Aim_assistant::Aim_assistant(){}
Aim_assistant::~Aim_assistant(){}
void  cvmat_to_tensor(cv::Mat  img,Tensor* tensor,int rows,int cols){

    cv::resize(img,img,cv::Size(rows,cols));
    GaussianBlur(img, img, Size(3,3), 0);
    cvtColor(img,img,COLOR_BGR2GRAY);
    threshold(img, img, 0, 255, CV_THRESH_OTSU);
    imshow("mask",img);
    float *p=tensor->flat<float>().data();
    cv::Mat imagePixels(rows,cols,CV_32FC1,p);
    img.convertTo(imagePixels,CV_32FC1);
}
int Aim_assistant::init(string model_path){
    Status status = NewSession(SessionOptions(), &session);
    if (!status.ok()) {
        std::cerr << status.ToString() << std::endl;
        return -1;
    } else {
        std::cout << "Session created successfully" << std::endl;
    }
    status = ReadBinaryProto(Env::Default(), model_path, &graph_def);
    if (!status.ok()) {
        std::cerr << status.ToString() << std::endl;
        return -1;
    } else {
        std::cout << "Load graph protobuf successfully" << std::endl;
    }
    status = session->Create(graph_def);
    if (!status.ok()) {
        std::cerr << status.ToString() << std::endl;
        return -1;
    } else {
        std::cout << "Add graph to session successfully" << std::endl;
    }
    Mat black=Mat(Size(32,32),CV_8UC1,Scalar(0));
    cout<<black.size()<<endl;
    
    TensorInit(session,black);
    return 1;


} 
void Aim_assistant::TensorInit(Session* session,Mat& img){
    Tensor x(tensorflow::DT_FLOAT, tensorflow::TensorShape({1,32, 32,1}));
    threshold(img, img, 0, 255, CV_THRESH_OTSU);
    std::vector<std::pair<string, tensorflow::Tensor>> inputs;
    inputs.push_back(std::pair<std::string, tensorflow::Tensor>("test-input/input", x));
    Tensor tensor_out(tensorflow::DT_FLOAT, TensorShape({1,5}));
    std::vector<tensorflow::Tensor> outputs={{ tensor_out }};
    Status status= session->Run(inputs, {"softmax"}, {}, &outputs);
    if (!status.ok()) {
        cout<<"failure"<<endl;
        std::cout << status.ToString() << "\n";
    }else cout<<"ok"<<endl;

}
int Aim_assistant::check_armor(cv::Mat frame){
    Tensor x(tensorflow::DT_FLOAT, tensorflow::TensorShape({1,32, 32,1}));
    cvmat_to_tensor(frame,&x,32,32);
    std::vector<std::pair<string, tensorflow::Tensor>> inputs;
    inputs.push_back(std::pair<std::string, tensorflow::Tensor>("test-input/input", x));
    Tensor tensor_out(tensorflow::DT_FLOAT, TensorShape({1,5}));
    std::vector<tensorflow::Tensor> outputs={{ tensor_out }};
    Status status= session->Run(inputs, {"softmax"}, {}, &outputs);
    if (!status.ok()) {
        cout<<"failure"<<endl;
        std::cout << status.ToString() << "\n";
        return -1;
    }else{

        cout<<"ok...."<<endl;

        cout << "Output tensor size:" << outputs.size() << std::endl;
        cout << outputs[0].DebugString()<<endl;


        Tensor t = outputs[0];

        //int output_dim = t.shape().dim_size(0);

        cout<<t.shape()<<endl;
        auto tmap = t.tensor<float, 2>();
        int m=0;
        for(int i=0;i<5;++i){
            cout<<i<<"-->"<<tmap(0,i)<<" ";
            if(tmap(0,i)>=tmap(0,m))
             m=i;
        }
        is_checked=true;
        if(tmap(0,m)>0.5){
            cout<<"result: "<<m+1<<endl;
            return m+1;
        }else return -1;
        

        
    }
}