#include "map.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstdlib>
using std::vector;
using std::atof;

Map::Map(){

}

Map::Map(string file){
	
	std::ifstream mapFile;
        mapFile.open(file.c_str());
        string line = "";

        vector<string> nums;
        if (mapFile.is_open()){
                while(getline(mapFile, line, ',')){
                        std::cout<<line<<"\n";
                        nums.push_back(line);
                }
                mapFile.close();
        }

	while(nums.size()>=7){
		vector<int> temp;
		// nums[3] and nums[4] are definite and potential corner types, ignored here	
		if(nums[5].compare("x") != 0) temp.push_back(std::atof(nums[5].c_str()));  // neighbor 1
		if(nums[6].compare("x") != 0) temp.push_back(std::atof(nums[6].c_str()));  // neighbor 2 
		EndPoint tmp(std::atof(nums[0].c_str()), std::atof(nums[1].c_str()),
		    std::atof(nums[2].c_str()),temp);
		mapPoints.push_back(tmp);
		for(int i=0; i<7; i++) nums.erase(nums.begin()+0);
	}
}

void Map::printCode(int code){
	std::cout<<"the map code is: "<<code<<" \n";
}




