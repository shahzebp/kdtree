#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>

#include "kd.h"

#define PRECISION      double   // change this to float or int

std::vector<std::vector<PRECISION>> getNextLineAndSplitIntoTokens(std::istream& str)
{
    std::vector<std::vector<PRECISION>> data;

    std::string                	line;
    while(std::getline(str,line)) {

        std::vector<PRECISION>   		result;
        std::stringstream          lineStream(line);
        std::string                cell;

        while(std::getline(lineStream,cell, ',')) {
            std::string::size_type sz; 
            PRECISION mars = std::stod (cell,&sz);

            result.push_back(mars);
        }
        data.push_back(result);
    }

    return data;
}

int main(int argc, char* argv[])
{
    std::cout << "Enter the file name containing sample data file" 
                    << std::endl;
    std::string sampleFile;
    std::cin >> sampleFile;

    std::cout << "Enter the name of file to dump the tree" 
                    << std::endl;
    std::string dumpFile;
    std::cin >> dumpFile;

	std::ifstream file(sampleFile);

    std::vector<std::vector<PRECISION>> data = 
                        getNextLineAndSplitIntoTokens(file);	
    
    unsigned int dim = data.front().size();

	KDTree<PRECISION> *Tree = new KDTree<PRECISION>(dim);

    std::vector<KDTree<PRECISION>::KDPoint> Points, nearPoints;

    std::vector<std::vector<PRECISION> >::iterator row;
    std::vector<PRECISION>::iterator col;

    unsigned index = 0;

    for (row = data.begin(); row != data.end(); row++) {
		
        KDTree<PRECISION>::KDPoint Point;
		int i = 0;
		for (col = row->begin(); col != row->end(); col++) {
		  
            Point.first.push_back((*col));
			i++;
		}
        Point.second = index;
        index++;

        Points.push_back(Point);
	}
	
    Tree->insert(Points);

    Tree->printKDTree(dumpFile);
   
    return 0;
}