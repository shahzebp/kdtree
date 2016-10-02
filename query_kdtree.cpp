#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include "kd.h"

using namespace std;

std::vector<std::vector<double>> getNextLineAndSplitIntoTokens(std::istream& str, char c)
{
    std::vector<std::vector<double>> data;

    std::string                	line;
    while(std::getline(str,line)) {

        std::vector<double>   		result;
        std::stringstream          lineStream(line);
        std::string                cell;

        while(std::getline(lineStream,cell, c)) {
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
    std::string treeFile, queryFile;

    std::cout << "Enter the file name to read tree from" << std::endl;
    std::cin >> treeFile;

    std::cout << "Enter the query points file " << endl;
    std::cin >> queryFile;


    std::ifstream       treeFileHandle(treeFile);
	std::ifstream       queryFileHandle(queryFile);

    std::vector<std::vector<double>> data = getNextLineAndSplitIntoTokens(
            treeFileHandle,' ');	

    const unsigned int dim = data.front().size();

    kd_tree<double> *Tree = new kd_tree<double>(dim);

    vector<kd_tree<double>::kd_point> Points, nearPoints;

	Tree->rebuild(data);

    std::vector<std::vector<double>> querydata = getNextLineAndSplitIntoTokens(
            queryFileHandle, ',');	

	vector< vector<double> >::iterator row;
	vector<double>::iterator col;

    for (row = data.begin(); row != data.end(); row++) {
		
        kd_tree<double>::kd_point Point;
		int i = 0;
		for (col = row->begin(); col != row->end(); col++) {
		
            Point.push_back(*col);
			i++;
		}
		
		kd_tree<double>::kd_point nearPoint;

		Tree->nearestNeighbor(Point, nearPoint);
		
        for(unsigned int i = 0; i < nearPoint.size(); i++) {
            std::cout << std::setprecision(17) << nearPoint[i] << " ";
        }
        std::cout << std::endl;
	}

    return 0;
}
