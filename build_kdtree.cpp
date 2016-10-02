#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>

#include "kd.h"

std::vector<std::vector<double>> getNextLineAndSplitIntoTokens(std::istream& str)
{
    std::vector<std::vector<double>> data;

    std::string                	line;
    while(std::getline(str,line)) {

        std::vector<double>   		result;
        std::stringstream          lineStream(line);
        std::string                cell;

        while(std::getline(lineStream,cell, ',')) {
            std::string::size_type sz; 
            double mars = std::stod (cell,&sz);

            result.push_back(mars);
        }
        data.push_back(result);
    }

    return data;
}

int main(int argc, char* argv[])
{

    std::cout << "Enter the file name containing sample data file" << std::endl;
    std::string sampleFile;
    std::cin >> sampleFile;

	std::ifstream file(sampleFile);

    std::vector<std::vector<double>> data = getNextLineAndSplitIntoTokens(file);	

    constexpr unsigned int dim = 3;//data.front().size();

	KDTree<double> *Tree = new KDTree<double>(3);

    std::vector<KDTree<double>::KDPoint> Points, nearPoints;

    std::vector<std::vector<double> >::iterator row;
    std::vector<double>::iterator col;

    for (row = data.begin(); row != data.end(); row++) {
		
        KDTree<double>::KDPoint Point;
		int i = 0;
		for (col = row->begin(); col != row->end(); col++) {
		
            Point.push_back((*col));
			i++;
		}

        Points.push_back(Point);
	}
	
    Tree->insert(Points);

    std::cout << "Enter the name of file to dump the tree" << std::endl;
    std::string dumpFile;

    std::cin >> dumpFile;

    Tree->printKDTree(dumpFile);
    
    return 0;
}
