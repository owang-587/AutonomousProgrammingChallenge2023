#include <iostream>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <iomanip>
#include <string>
#include <sstream>

using namespace std;

// Creating a coordinate type
struct Coordinate {
    double x, y, z;
};

// function to calculate the distance
double DistanceCalc(const Coordinate point1, const Coordinate point2) {
    double xDist = point2.x - point1.x;
    double yDist = point2.y - point1.y;
    double zDist = point2.z - point1.z;
    return sqrt(xDist*xDist + yDist*yDist + zDist*zDist);
}


int main() {

    Coordinate startPoint = {-171.60, 4.00, 0.00};

    // vector holding all the goal points
    vector<Coordinate> goalPoints = {
        {-206.40,4.20,0.00},
        {-255.90,0.20,0.00},
        {-272.10,-43.90,0.00},
        {-205.50,-95.00,0.00},
        {-185.50,-142.40,0.00},
        {-151.10,-151.00,0.00},
        {-101.40,-154.70,0.00},
        {-47.80,-117.20,0.00},
        {-43.80,-56.80,0.00},
        {-43.90,-17.10,0.00},
        {3.00,-2.70,0.00},
        {47.80,-1.80,0.00},
        {89.00,-5.50,0.00},
        {45.90,-84.90,0.00},
        {31.30,19.30,0.00},
        {36.30,67.20,0.00},
        {38.60,155.10,0.00},
        {74.00,190.20,0.00},
        {154.10,177.30,0.00},
        {189.20,52.80,0.00},
        {174.40,-148.00,0.00},
        {10.20,-187.90,0.00},
        {-145.80,-190.90,8.60},
        {-232.60,28.10,10.00},
        {-119.40,186.60,10.00},
        {84.70,144.10,0.00},
        {148.10,112.20,0.00},
        {151.40,15.20,0.00},
        {124.70,1.90,0.00},
        {96.20,-28.60,0.00},
        {-9.50,-88.30,0.00},
        {-83.20,-87.70,0.00},
        {-124.30,-42.40,0.00},
        {-121.80,28.10,0.00},
        {-124.40,106.30,0.00},
        {-80.20,133.30,0.00},
        {-20.70,87.90,0.00},
        {25.70,65.40,0.00},
        {24.60,-30.70,0.00}
    };


    // initalizing unordered map
    unordered_map<double, Coordinate> distanceLookup; 
    // creating unordered map
    for (int i = 0; i < goalPoints.size(); i++) {
        double distance = DistanceCalc(startPoint, goalPoints[i]);
        // key (distance to goal) : value (coordinate of goal)
        distanceLookup[distance] = goalPoints[i];
    }


    for (auto it = distanceLookup.begin(); it != distanceLookup.end(); ++it) {
        cout << setprecision(2) << "Distance: " << it->first;
        cout << fixed << setprecision(1) << " ; Goal Point: (" << it->second.x << ", " << it->second.y << ", " << it->second.z << ")" << endl;
    }

/*
getline(INPUT STRING, STRING VARIABLE, DELIMITER)
*/
    string inputString;
    cout << "Enter comma-separated list of distances: ";
    getline(cin, inputString); // read input as a string

    // stringstream lets reading and writing of input like a file

    stringstream input(inputString); // object called input of type stringstream with content inputString
    string value;
    while (getline(input, value, ',')) {
        double inputDistance = stod(value); // stod (string to double)
        bool found = false;
        for (auto it = distanceLookup.begin(); it != distanceLookup.end(); ++it) {
            double distance = it->first;
            if (abs(distance - inputDistance) < 1e-0) {
                cout << setprecision(2) << "Point for Distance (" << inputDistance << ") = (" << setprecision(1) << it->second.x << ", " << it->second.y << ", " << it->second.z << ")" << endl;
                found = true;
                break;
            }
        }
        if (!found) {
            cout << "Distance " << inputDistance << " not found" << endl;
        }
    }   
    

    return 0;
}