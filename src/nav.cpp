/* nav.cpp
 *
 *
 */
#include "nav.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
using std::vector;


void Nav::loadMap(string file){
	Map tmp(file);
	universalMap = tmp;
}

Map* Nav::getMap(){ 
	return &universalMap;
	
}
