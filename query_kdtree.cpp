#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include "kd.h"

#define PRECISION      double   // change this to float or int

std::vector<std::vector<PRECISION>> getNextLineAndSplitIntoTokens(
                        std::istream& str, char c)
{
    std::vector<std::vector<PRECISION>> data;

    std::string                	line;
    
    while(std::getline(str,line)) {

        std::vector<PRECISION>   		result;
        std::stringstream          lineStream(line);
        std::string                cell;

        while(std::getline(lineStream,cell, c)) {
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
    std::string treeFile, queryFile, answerFile;
    
    std::cout << "Enter the file name to read tree from" << std::endl;
    std::cin >> treeFile;

    std::cout << "Enter the query points file " << std::endl;
    std::cin >> queryFile;

    std::cout << "Enter the file to write query anwers " << std::endl;
    std::cin >> answerFile;

    std::ifstream       treeFileHandle(treeFile);
    
    std::ifstream       queryFileHandle(queryFile);

    std::vector<std::vector<PRECISION>> querydata = 
                        getNextLineAndSplitIntoTokens(queryFileHandle, ',');    

    std::vector<std::vector<PRECISION>> data = 
                        getNextLineAndSplitIntoTokens(treeFileHandle,' ');	

 
    const unsigned int dim = data.front().size() - 1;

    KDTree<PRECISION> *Tree = new KDTree<PRECISION>(dim);

    std::vector<KDTree<PRECISION>::KDPoint> nearPoints;

    std::vector<KDTree<PRECISION>::KDPoint> *points = 
                new std::vector<KDTree<PRECISION>::KDPoint>();

    std::vector<std::vector<PRECISION> >::iterator row;
    std::vector<PRECISION>::iterator col;

    for (row = data.begin(); row != data.end(); row++) {

        KDTree<PRECISION>::KDPoint Point;
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
	
        KDTree<PRECISION>::KDPoint Point;
		int i = 0;
		for (col = row->begin(); col != row->end(); col++) {
		
            Point.first.push_back(*col);
			i++;
		}
		
		KDTree<PRECISION>::KDPoint nearPoint;

		Tree->nearestNeighbor(Point, nearPoint);

        fs << nearPoint.second << "," << PRINTPRECISE
                        << Tree->getDistance(Point, nearPoint) 
                        << std::endl;
		}

    return 0;
}