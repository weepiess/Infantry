#include <iostream>
#include <vector>
#include <string>
#include <dirent.h>
using namespace std;
int main(int argc, char * argv[])
{
    
    struct dirent *ptr;    
    DIR *dir;
    string PATH = "/home/pc";
    dir = opendir(PATH.c_str());
    vector<string> files;
    cout << "文件列表: "<< endl;
    while((ptr=readdir(dir))!=NULL)
    {
        cout<<"1"<<endl;
        //跳过'.'和'..'两个目录
        if(ptr->d_name[0] == '.')
            continue;
        //cout << ptr->d_name << endl;
        
        files.push_back(ptr->d_name);
        
    }
    
    for (int i = 0; i < files.size(); ++i)
    {
        cout << files[i] << endl;
        
    }
 
    closedir(dir);
    return 0;
}