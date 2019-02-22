#include "datathread.h"
#include <iostream>

NYdector::NYdector(){}

NYdector::~NYdector(){}

void NYdector::init(AutoAim *pAutoaim){
    autoaim = pAutoaim;
}

void NYdector::run(){
    while(true)
        autoaim -> datajudger();
}