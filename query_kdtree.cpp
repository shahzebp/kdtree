#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include "kd.h"


std::vector<std::vector<double>> getNextLineAndSplitIntoTokens(
                        std::istream& str, char c)
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
    std::string treeFile, queryFile, answerFile;
    
    std::cout << "Enter the file name to read tree from" << std::endl;
    std::cin >> treeFile;

    std::cout << "Enter the query points file " << std::endl;
    std::cin >> queryFile;

    std::cout << "Enter the file to write query anwers " << std::endl;
    std::cin >> answerFile;

    std::ifstream       treeFileHandle(treeFile);
    
    std::ifstream       queryFileHandle(queryFile);

    std::vector<std::vector<double>> querydata = 
                        getNextLineAndSplitIntoTokens(queryFileHandle, ',');    

    std::vector<std::vector<double>> data = 
                        getNextLineAndSplitIntoTokens(treeFileHandle,' ');	


 
    const unsigned int dim = data.front().size() - 1;

    KDTree<double> *Tree = new KDTree<double>(dim);

    std::vector<KDTree<double>::KDPoint> nearPoints;

    std::vector<KDTree<double>::KDData> *points = 
                new std::vector<KDTree<double>::KDData>();

    std::vector<std::vector<double> >::iterator row;
    std::vector<double>::iterator col;

    for (row = data.begin(); row != data.end(); row++) {

        KDTree<double>::KDData Point;
        Point.second = *(row->begin());

        for (col = row->begin() + 1; col != row->end(); col++) {

            Point.first.push_back(*col);
        }
        (*points).push_back(Point);
    }

	Tree->rebuild((*points));

    std::fstream fs;
    fs.open (answerFile.c_str(), std::fstream::out);  
   
    for (row = querydata.begin(); row != querydata.end(); row++) {
	
        KDTree<double>::KDData Point;
		int i = 0;
		for (col = row->begin(); col != row->end(); col++) {
		
            Point.first.push_back(*col);
			i++;
		}
		
		KDTree<double>::KDData nearPoint;

		Tree->nearestNeighbor(Point, nearPoint);

        fs << nearPoint.second << "," << std::setprecision(17)
                        << Tree->getDistance(Point, nearPoint) 
                        << std::endl;
		}

    return 0;
}