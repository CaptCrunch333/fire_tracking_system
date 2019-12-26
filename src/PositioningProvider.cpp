#include "PositioningProvider.hpp"

PositioningProvider::PositioningProvider(){

}

PositioningProvider::~PositioningProvider(){
    
}

void PositioningProvider::setHomePosition(Vector3D<float> data){


}

Vector3D<float> PositioningProvider::getRelativePosition(){
    return _homePos;
}
